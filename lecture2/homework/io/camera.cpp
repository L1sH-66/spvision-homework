#include "camera.hpp"

#include <yaml-cpp/yaml.h>

#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "hikrobot/include/MvCameraControl.h"

namespace {
std::runtime_error camera_error(const std::string& operation, int code) {
  std::ostringstream message;
  message << operation << " failed, error=0x" << std::hex << std::uppercase
          << code;
  return std::runtime_error(message.str());
}

std::pair<unsigned short, unsigned short> parse_vid_pid(
    const std::string& value) {
  const auto separator = value.find(':');
  if (separator == std::string::npos) {
    throw std::invalid_argument("vid_pid must have the form vvvv:pppp");
  }

  try {
    const auto vendor = static_cast<unsigned short>(
        std::stoul(value.substr(0, separator), nullptr, 16));
    const auto product = static_cast<unsigned short>(
        std::stoul(value.substr(separator + 1), nullptr, 16));
    return {vendor, product};
  } catch (const std::exception&) {
    throw std::invalid_argument("invalid hexadecimal vid_pid: " + value);
  }
}
}  // namespace

namespace io {
Camera::Camera(const std::string& config_path) {
  const YAML::Node config = YAML::LoadFile(config_path);
  const std::string camera_name = config["camera_name"].as<std::string>();
  const double exposure_ms = config["exposure_ms"].as<double>();
  const double gain = config["gain"].as<double>();
  const auto [vendor_id, product_id] =
      parse_vid_pid(config["vid_pid"].as<std::string>());

  if (camera_name != "hikrobot") {
    throw std::invalid_argument("unsupported camera_name: " + camera_name);
  }

  MV_CC_DEVICE_INFO_LIST device_list{};
  int result = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  if (result != MV_OK) {
    throw camera_error("MV_CC_EnumDevices", result);
  }

  MV_CC_DEVICE_INFO* selected_device = nullptr;
  for (unsigned int index = 0; index < device_list.nDeviceNum; ++index) {
    MV_CC_DEVICE_INFO* device = device_list.pDeviceInfo[index];
    if (device == nullptr || device->nTLayerType != MV_USB_DEVICE) {
      continue;
    }
    const auto& usb = device->SpecialInfo.stUsb3VInfo;
    if (usb.idVendor == vendor_id && usb.idProduct == product_id) {
      selected_device = device;
      break;
    }
  }

  if (selected_device == nullptr) {
    std::ostringstream message;
    message << "HikRobot camera " << std::hex << std::setfill('0')
            << std::setw(4) << vendor_id << ':' << std::setw(4) << product_id
            << " was not found";
    throw std::runtime_error(message.str());
  }

  result = MV_CC_CreateHandle(&handle_, selected_device);
  if (result != MV_OK) {
    throw camera_error("MV_CC_CreateHandle", result);
  }

  try {
    result = MV_CC_OpenDevice(handle_);
    if (result != MV_OK) {
      throw camera_error("MV_CC_OpenDevice", result);
    }
    opened_ = true;

    if ((result = MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto",
                                     MV_BALANCEWHITE_AUTO_CONTINUOUS)) !=
        MV_OK) {
      throw camera_error("set BalanceWhiteAuto", result);
    }
    if ((result = MV_CC_SetEnumValue(handle_, "ExposureAuto",
                                     MV_EXPOSURE_AUTO_MODE_OFF)) != MV_OK) {
      throw camera_error("set ExposureAuto", result);
    }
    if ((result = MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF)) !=
        MV_OK) {
      throw camera_error("set GainAuto", result);
    }
    if ((result = MV_CC_SetFloatValue(
             handle_, "ExposureTime",
             static_cast<float>(exposure_ms * 1000.0))) != MV_OK) {
      throw camera_error("set ExposureTime", result);
    }
    if ((result = MV_CC_SetFloatValue(handle_, "Gain",
                                      static_cast<float>(gain))) != MV_OK) {
      throw camera_error("set Gain", result);
    }

    result = MV_CC_StartGrabbing(handle_);
    if (result != MV_OK) {
      throw camera_error("MV_CC_StartGrabbing", result);
    }
    grabbing_ = true;
  } catch (...) {
    if (grabbing_) {
      MV_CC_StopGrabbing(handle_);
    }
    if (opened_) {
      MV_CC_CloseDevice(handle_);
    }
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
    grabbing_ = false;
    opened_ = false;
    throw;
  }
}

Camera::~Camera() {
  if (handle_ == nullptr) {
    return;
  }
  if (grabbing_) {
    MV_CC_StopGrabbing(handle_);
  }
  if (opened_) {
    MV_CC_CloseDevice(handle_);
  }
  MV_CC_DestroyHandle(handle_);
}

void Camera::read(cv::Mat& img,
                  std::chrono::steady_clock::time_point& timestamp) {
  MV_FRAME_OUT raw{};
  const int get_result = MV_CC_GetImageBuffer(handle_, &raw, 1000);
  if (get_result != MV_OK) {
    throw camera_error("MV_CC_GetImageBuffer", get_result);
  }
  timestamp = std::chrono::steady_clock::now();

  try {
    img.create(static_cast<int>(raw.stFrameInfo.nHeight),
               static_cast<int>(raw.stFrameInfo.nWidth), CV_8UC3);

    MV_CC_PIXEL_CONVERT_PARAM_EX convert{};
    convert.nWidth = raw.stFrameInfo.nWidth;
    convert.nHeight = raw.stFrameInfo.nHeight;
    convert.enSrcPixelType = raw.stFrameInfo.enPixelType;
    convert.pSrcData = raw.pBufAddr;
    convert.nSrcDataLen = raw.stFrameInfo.nFrameLen;
    convert.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
    convert.pDstBuffer = img.data;
    convert.nDstBufferSize =
        static_cast<unsigned int>(img.total() * img.elemSize());

    const int convert_result = MV_CC_ConvertPixelTypeEx(handle_, &convert);
    if (convert_result != MV_OK) {
      throw camera_error("MV_CC_ConvertPixelTypeEx", convert_result);
    }
  } catch (...) {
    MV_CC_FreeImageBuffer(handle_, &raw);
    throw;
  }

  const int free_result = MV_CC_FreeImageBuffer(handle_, &raw);
  if (free_result != MV_OK) {
    throw camera_error("MV_CC_FreeImageBuffer", free_result);
  }
}
}  // namespace io
