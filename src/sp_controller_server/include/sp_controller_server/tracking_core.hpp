#pragma once
// ROS-independent path-tracking core.  The ROS plugin (plugins/pid_controller.cpp),
// the unit test (tests/core_test.cpp) and the offline plant test
// (tests/offline_plant.py via tests/core_bridge.cpp) all use exactly this code.
//
// Pipeline, run at the controller rate:
//   plan  -> arc length + curvature/braking speed profile        (once per plan)
//   odom  -> predicted state after the actuation dead time       (Smith predictor)
//   mpc   -> linear MPC on the identified first-order velocity model
//            (fallback "pursuit": look-ahead direction + cross-track P + damping)
//   goal  -> Euclidean position servo inside goal_radius
//   out   -> plant inverse (measured steady-state table) + spin offset compensation
// The caller rotates the returned map-frame command into base_link.
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <stdexcept>
#include <vector>

namespace sp_tracking {

constexpr double kPi = 3.14159265358979323846;

struct Vec {
  double x{}, y{};
  Vec operator+(Vec b) const { return {x + b.x, y + b.y}; }
  Vec operator-(Vec b) const { return {x - b.x, y - b.y}; }
  Vec operator*(double a) const { return {a * x, a * y}; }
};
inline double dot(Vec a, Vec b) { return a.x * b.x + a.y * b.y; }
inline double norm(Vec a) { return std::hypot(a.x, a.y); }
inline Vec limit(Vec a, double n) {
  const double d = norm(a);
  return d > n && d > 0 ? a * (n / d) : a;
}
inline bool finite(Vec a) { return std::isfinite(a.x) && std::isfinite(a.y); }
inline Vec rotate(Vec a, double angle) {
  const double c = std::cos(angle), s = std::sin(angle);
  return {c * a.x - s * a.y, s * a.x + c * a.y};
}

// Steady-state response of the supplied simulator plant while the chassis spins
// at its default 3 rad/s, measured with constant commands u = 0.0 ... 2.0 m/s:
// the speed follows the command up to ~1.1 m/s and is then compressed towards
// the ~1.44 m/s cap; the velocity vector is also turned counter-clockwise by
// roughly one chassis-spin step (~5 deg), less once the cap is active.
namespace plant {
// Commands above 2 m/s still raise the real speed (1.59 m/s at u = 3, 1.68 at
// u = 4, 1.82 at u = 8): the spin only compresses the response, and the
// simulator separately clamps the real velocity to v_max = 2 m/s.
constexpr int kTable = 29;
constexpr double kU[kTable] = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0,
                               1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0,
                               2.2, 2.5, 2.8, 3.0, 3.5, 4.0, 5.0, 6.0};
constexpr double kV[kTable] = {0.0,   0.098, 0.196, 0.295, 0.393, 0.491, 0.590,
                               0.684, 0.794, 0.894, 1.000, 1.123, 1.203, 1.249,
                               1.280, 1.310, 1.339, 1.366, 1.391, 1.414, 1.435,
                               1.474, 1.523, 1.563, 1.587, 1.636, 1.675, 1.732, 1.772};
constexpr double kAngleDeg[kTable] = {5.34, 5.34, 5.33, 5.31, 5.28, 5.24, 5.26,
                                      5.73, 5.78, 5.15, 5.40, 5.18, 3.94, 3.03,
                                      2.34, 1.84, 1.50, 1.29, 1.15, 1.04, 0.94,
                                      0.77, 0.60, 0.48, 0.42, 0.31, 0.24, 0.15, 0.10};
inline double interp(const double* xs, const double* ys, double x) {
  if (x <= xs[0]) return ys[0];
  if (x >= xs[kTable - 1]) return ys[kTable - 1];
  const int hi = static_cast<int>(std::upper_bound(xs, xs + kTable, x) - xs);
  const double r = (x - xs[hi - 1]) / (xs[hi] - xs[hi - 1]);
  return ys[hi - 1] + r * (ys[hi] - ys[hi - 1]);
}
inline double steadySpeed(double u) { return interp(kU, kV, u); }
inline double commandFor(double v) { return interp(kV, kU, v); }
inline double offset(double u) { return interp(kU, kAngleDeg, u) * kPi / 180.0; }
inline double maxSpeed() { return kV[kTable - 1]; }
}  // namespace plant

struct Config {
  // speed profile
  double max_speed = 1.42;         // cruise limit (real speed; steady state of a 2 m/s command)
  double lateral_accel = 2.0;      // v <= sqrt(lateral_accel / curvature); the plant cannot turn harder
  double brake_decel = 1.2;        // deceleration used by the profile (< a_max: the plant lags)
  double min_corner_speed = 0.35;
  double curvature_window = 0.30;
  // plant model
  double dead_time = 0.24;         // command -> velocity response delay (s)
  double lag_gain = 3.4;           // v' = lag_gain * (v_ss(u) - v)
  double max_acceleration = 2.0;   // simulator a_max
  // linear MPC
  bool use_mpc = true;
  int mpc_horizon = 16;
  double mpc_step = 0.05;
  double mpc_q_cross = 400.0, mpc_q_along = 4.0, mpc_q_vel = 1.0;
  double mpc_r_delta = 1.0, mpc_r_ref = 0.6;
  // pursuit fallback
  double lookahead = 0.30, lookahead_time = 0.10;
  double position_gain = 2.0, lateral_damping = 0.18;
  // velocity loop (pursuit and goal servo)
  double velocity_kp = 1.0, velocity_ki = 0.6, velocity_integral_limit = 0.35;
  // goal
  double goal_radius = 0.40;       // remaining path below which the Euclidean servo is used
  double goal_gain = 4.0;
  double stop_tolerance = 0.02, stop_speed = 0.05;
  // output
  double command_slew = 12.0, max_command = 2.0;  // = simulator v_max
  // re-planning hysteresis: keep following the current plan while new plans stay this close
  double plan_keep_distance = 0.30;

  void validate() const {
    const double positive[] = {max_speed, lateral_accel, brake_decel, curvature_window,
                               lag_gain, max_acceleration, mpc_step, lookahead,
                               goal_radius, goal_gain, stop_tolerance, stop_speed,
                               command_slew, max_command};
    for (double v : positive)
      if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("tracker parameter must be positive");
    const double non_negative[] = {min_corner_speed, dead_time, mpc_q_cross, mpc_q_along,
                                   mpc_q_vel, mpc_r_delta, mpc_r_ref, lookahead_time,
                                   position_gain, lateral_damping, velocity_kp, velocity_ki,
                                   velocity_integral_limit, plan_keep_distance};
    for (double v : non_negative)
      if (!std::isfinite(v) || v < 0) throw std::invalid_argument("tracker parameter must be non-negative");
    if (mpc_horizon < 2 || mpc_horizon > 40) throw std::invalid_argument("mpc_horizon must be in [2, 40]");
    if (dead_time > 1.0) throw std::invalid_argument("dead_time must be <= 1 s");
  }
};

class Tracker {
 public:
  Config cfg;

  // Returns true if the new plan replaced the current one (false: kept, see plan_keep_distance).
  bool setPlan(const std::vector<Vec>& input) {
    std::vector<Vec> clean;
    for (auto p : input) {
      if (!finite(p)) {
        clearPlan();
        throw std::invalid_argument("non-finite path");
      }
      if (clean.empty() || norm(p - clean.back()) > 1e-6) clean.push_back(p);
    }
    if (clean.empty()) {
      clearPlan();
      return true;
    }
    clean = densify(clean, 0.10);  // the curvature estimate needs way-points <= 0.1 m apart
    if (!points_.empty() && !force_replace_ && keepCurrent(clean)) return false;
    points_ = std::move(clean);
    buildProfile();
    progress_ = 0.0;
    force_replace_ = false;
    return true;
  }

  void clearPlan() {
    points_.clear();
    arc_.clear();
    profile_.clear();
    progress_ = 0.0;
    reset();
  }

  // Forget the command history and integrators (not the path progress).
  void reset() {
    history_.clear();
    last_cmd_ = {};
    last_w_ = {};
    vel_integral_ = {};
    have_time_ = false;
  }

  bool hasPlan() const { return !points_.empty(); }
  double length() const { return arc_.empty() ? 0.0 : arc_.back(); }
  double lastCrossTrackError() const { return last_cross_; }
  // For visualisation: the state after the dead time followed by the MPC's
  // predicted positions, and the reference points it tracked (empty otherwise).
  const std::vector<Vec>& lastPrediction() const { return prediction_; }
  const std::vector<Vec>& lastReference() const { return reference_; }

  Vec sample(double s) const {
    if (points_.empty()) return {};
    if (s <= 0) return points_.front();
    if (s >= arc_.back()) return points_.back();
    const std::size_t i = std::upper_bound(arc_.begin(), arc_.end(), s) - arc_.begin();
    return points_[i - 1] + (points_[i] - points_[i - 1]) * ((s - arc_[i - 1]) / (arc_[i] - arc_[i - 1]));
  }

  double profileSpeed(double s) const {
    if (points_.empty() || s >= arc_.back()) return 0.0;
    if (s <= 0) return profile_.front();
    const std::size_t i = std::upper_bound(arc_.begin(), arc_.end(), s) - arc_.begin();
    const double r = (s - arc_[i - 1]) / (arc_[i] - arc_[i - 1]);
    return profile_[i - 1] + r * (profile_[i] - profile_[i - 1]);
  }

  // t: monotonic time in seconds; pos/vel: map frame.  Returns the map-frame
  // command (what to send after rotating into base_link).
  Vec compute(double t, Vec pos, Vec vel) {
    if (points_.empty() || !finite(pos) || !finite(vel) || !std::isfinite(t)) {
      reset();
      return {};
    }
    // A gap means the server stopped calling us (control disabled, stale plan or
    // TF) and published zero velocity meanwhile: the stored commands are void.
    if (have_time_ && t - last_time_ > 0.1) reset();
    double dt = 0.02;
    if (have_time_) dt = std::clamp(t - last_time_, 0.002, 0.1);
    last_time_ = t;
    have_time_ = true;

    const Vec goal = points_.back();
    if (norm(goal - pos) < cfg.stop_tolerance && norm(vel) < cfg.stop_speed) {
      last_cmd_ = {};
      last_w_ = {};
      vel_integral_ = {};
      remember(t, {});
      return {};
    }

    // 1. state after the dead time
    Vec p = pos, v = vel;
    predict(t, p, v);

    // 2. projection (search window: progress so far + 2 m, so a parallel
    //    corridor behind a wall is never picked)
    const double s = project(p);
    progress_ = s;
    const Vec closest = sample(s);
    if (norm(closest - p) > 0.6) force_replace_ = true;
    const double remaining = std::max(arc_.back() - s, 0.0);
    const Vec tangent = tangentAt(s);
    const Vec normal{-tangent.y, tangent.x};
    last_cross_ = dot(closest - p, normal);

    // 3. desired velocity
    Vec v_des{};
    double kp = cfg.velocity_kp;
    prediction_.clear();
    reference_.clear();
    const bool capture = remaining < cfg.goal_radius;
    if (capture) {
      const Vec to_goal = goal - p;
      const double d = norm(to_goal);
      const double mag = std::min({cfg.max_speed, cfg.goal_gain * d, std::sqrt(2.0 * cfg.brake_decel * d)});
      v_des = d > 1e-6 ? to_goal * (mag / d) : Vec{};
      last_w_ = v;
    } else if (cfg.use_mpc) {
      Vec w{};
      if (solveMpc(s, p, v, w)) {
        v_des = limit(w, plant::steadySpeed(cfg.max_command));
        last_w_ = v_des;
        kp = 0.0;  // the MPC already plans through the lag
      } else {
        v_des = pursuit(s, p, v, tangent, normal);
      }
    } else {
      v_des = pursuit(s, p, v, tangent, normal);
    }

    // 4. velocity loop + plant inverse
    const Vec err = v_des - v;
    const double vd = norm(v_des);
    Vec u = vd > 1e-6 ? v_des * (plant::commandFor(vd) / vd) : Vec{};
    u = u + err * kp + vel_integral_ * cfg.velocity_ki;
    if (norm(u) > cfg.max_command) {
      u = limit(u, cfg.max_command);
    } else {
      vel_integral_.x = std::clamp(vel_integral_.x + err.x * dt, -cfg.velocity_integral_limit, cfg.velocity_integral_limit);
      vel_integral_.y = std::clamp(vel_integral_.y + err.y * dt, -cfg.velocity_integral_limit, cfg.velocity_integral_limit);
    }
    u = rotate(u, -plant::offset(norm(u)));
    const Vec step = u - last_cmd_;
    const double max_step = cfg.command_slew * dt;
    if (norm(step) > max_step) u = last_cmd_ + step * (max_step / norm(step));
    last_cmd_ = u;
    remember(t, u);
    return u;
  }

 private:
  struct Sample {
    double t;
    Vec u;
  };

  static std::vector<Vec> densify(const std::vector<Vec>& in, double spacing) {
    std::vector<Vec> out{in.front()};
    for (std::size_t i = 1; i < in.size(); ++i) {
      const Vec d = in[i] - in[i - 1];
      const int pieces = std::max(1, static_cast<int>(std::ceil(norm(d) / spacing)));
      for (int k = 1; k <= pieces; ++k) out.push_back(in[i - 1] + d * (static_cast<double>(k) / pieces));
    }
    return out;
  }

  void remember(double t, Vec u) {
    history_.push_back({t, u});
    while (history_.size() > 2 && history_[1].t < t - cfg.dead_time - 0.2) history_.pop_front();
  }

  // Model: the command issued at time t_c starts acting at t_c + dead_time;
  // afterwards v' = lag_gain (v_ss(u) R(offset) - v) with |v'| <= max_acceleration.
  void predict(double now, Vec& p, Vec& v) const {
    if (cfg.dead_time <= 1e-6) return;
    const double h = 0.01;
    const int steps = static_cast<int>(std::ceil(cfg.dead_time / h));
    for (int j = 0; j < steps; ++j) {
      const double query = now + j * h - cfg.dead_time;
      Vec u{};  // nothing issued that long ago -> no input yet
      for (auto it = history_.rbegin(); it != history_.rend(); ++it) {
        if (it->t <= query) {
          u = it->u;
          break;
        }
      }
      const double um = norm(u);
      const Vec target = um > 1e-9 ? rotate(u * (plant::steadySpeed(um) / um), plant::offset(um)) : Vec{};
      const Vec a = limit((target - v) * cfg.lag_gain, cfg.max_acceleration);
      v = v + a * h;
      p = p + v * h;
    }
  }

  double project(Vec p) const {
    if (points_.size() == 1) return 0.0;
    double best = std::numeric_limits<double>::infinity(), s = 0.0;
    const double window_end = progress_ + 2.0;
    for (std::size_t i = 1; i < points_.size(); ++i) {
      if (arc_[i - 1] > window_end) break;
      const Vec a = points_[i - 1], d = points_[i] - a;
      const double t = std::clamp(dot(p - a, d) / dot(d, d), 0.0, 1.0);
      const double e = norm(p - (a + d * t));
      if (e < best) {
        best = e;
        s = arc_[i - 1] + t * (arc_[i] - arc_[i - 1]);
      }
    }
    return s;
  }

  Vec tangentAt(double s) const {
    const double len = length();
    const Vec d = sample(std::min(s + 0.15, len)) - sample(std::max(s - 0.05, 0.0));
    const double n = norm(d);
    if (n > 1e-9) return d * (1.0 / n);
    if (points_.size() >= 2) {
      const Vec e = points_.back() - points_[points_.size() - 2];
      return e * (1.0 / norm(e));
    }
    return {1.0, 0.0};
  }

  Vec pursuit(double s, Vec p, Vec v, Vec tangent, Vec normal) {
    const double look = cfg.lookahead + cfg.lookahead_time * norm(v);
    Vec dir = sample(std::min(s + look, length())) - p;
    const double dn = norm(dir);
    dir = dn > 1e-6 ? dir * (1.0 / dn) : tangent;
    const double speed = std::min(profileSpeed(s), cfg.max_speed);
    const double lateral = cfg.position_gain * last_cross_ - cfg.lateral_damping * dot(v, normal);
    last_w_ = v;
    return limit(dir * speed + normal * lateral, cfg.max_speed);
  }

  void buildProfile() {
    const std::size_t n = points_.size();
    arc_.assign(n, 0.0);
    for (std::size_t i = 1; i < n; ++i) arc_[i] = arc_[i - 1] + norm(points_[i] - points_[i - 1]);
    profile_.assign(n, cfg.max_speed);
    // curvature from the heading change over +-curvature_window (robust to grid stair-cases)
    std::size_t a = 0, b = 0;
    for (std::size_t i = 0; i < n; ++i) {
      while (a + 1 < i && arc_[i] - arc_[a + 1] >= cfg.curvature_window) ++a;
      if (b < i) b = i;
      while (b + 1 < n && arc_[b] - arc_[i] < cfg.curvature_window) ++b;
      if (i == 0 || i + 1 == n) continue;
      const Vec u = points_[i] - points_[a], w = points_[b] - points_[i];
      if (norm(u) < 1e-6 || norm(w) < 1e-6) continue;
      const double turn = std::fabs(std::atan2(u.x * w.y - u.y * w.x, dot(u, w)));
      const double kappa = turn / std::max(0.5 * (arc_[b] - arc_[a]), 1e-3);
      if (kappa > 1e-3)
        profile_[i] = std::min(profile_[i], std::max(std::sqrt(cfg.lateral_accel / kappa), cfg.min_corner_speed));
    }
    profile_[n - 1] = 0.0;
    for (std::size_t i = n - 1; i-- > 0;)
      profile_[i] = std::min(profile_[i], std::sqrt(profile_[i + 1] * profile_[i + 1] +
                                                    2.0 * cfg.brake_decel * (arc_[i + 1] - arc_[i])));
  }

  // Keep the current plan if the new one (first 8 m) stays within plan_keep_distance of it.
  bool keepCurrent(const std::vector<Vec>& fresh) const {
    if (cfg.plan_keep_distance <= 0 || norm(fresh.back() - points_.back()) > 0.15) return false;
    double fresh_len = 0.0, worst = 0.0;
    const double from = std::max(progress_ - 0.5, 0.0);
    for (std::size_t k = 0; k < fresh.size() && fresh_len <= 8.0; ++k) {
      if (k > 0) fresh_len += norm(fresh[k] - fresh[k - 1]);
      double best = std::numeric_limits<double>::infinity();
      for (std::size_t i = 1; i < points_.size(); ++i) {
        if (arc_[i] < from) continue;
        const Vec a = points_[i - 1], d = points_[i] - a;
        const double t = std::clamp(dot(fresh[k] - a, d) / dot(d, d), 0.0, 1.0);
        best = std::min(best, norm(fresh[k] - (a + d * t)));
      }
      worst = std::max(worst, best);
      if (worst >= cfg.plan_keep_distance) return false;
    }
    return true;
  }

  // Linear MPC.  Per axis, step h:  v+ = a v + (1-a) w,  p+ = p + b v + (h-b) w,
  // a = exp(-lag_gain h), b = (1-a)/lag_gain; w is the steady-state velocity aimed for.
  // Reference: a point that starts at the projection and moves along the path with
  // the speed profile.  Cost: q_cross (n.e)^2 + q_along (t.e)^2 + q_vel |v - vr|^2
  // + r_delta |dw|^2 + r_ref |w - vr|^2.  Unconstrained QP (2N variables), solved
  // with Cholesky; only w_0 is applied (receding horizon), the speed cap afterwards.
  bool solveMpc(double s_star, Vec p0, Vec v0, Vec& w0) {
    const int n = cfg.mpc_horizon;
    const double h = cfg.mpc_step;
    const double a = std::exp(-cfg.lag_gain * h);
    const double b = (1.0 - a) / cfg.lag_gain;
    const double c = h - b;
    const std::size_t nn = static_cast<std::size_t>(n);
    std::vector<double> gv(nn * nn, 0.0), gp(nn * nn, 0.0);
    std::vector<Vec> fv(nn), fp(nn), r(nn), vr(nn), tan(nn);
    Vec vf = v0, pf = p0;
    for (std::size_t k = 0; k < nn; ++k) {
      for (std::size_t j = 0; j < nn; ++j) {
        const double in = j == k ? 1.0 : 0.0;
        const double gv_prev = k ? gv[(k - 1) * nn + j] : 0.0;
        const double gp_prev = k ? gp[(k - 1) * nn + j] : 0.0;
        gv[k * nn + j] = a * gv_prev + (1.0 - a) * in;
        gp[k * nn + j] = gp_prev + b * gv_prev + c * in;
      }
      pf = pf + vf * b;
      vf = vf * a;
      fv[k] = vf;
      fp[k] = pf;
    }
    double s = s_star;
    for (std::size_t k = 0; k < nn; ++k) {
      s = std::min(length(), s + h * std::min(profileSpeed(s), cfg.max_speed));
      r[k] = sample(s);
      tan[k] = tangentAt(s);
      vr[k] = tan[k] * std::min(profileSpeed(s), cfg.max_speed);
    }
    const std::size_t m = 2 * nn;
    std::vector<double> H(m * m, 0.0), g(m, 0.0);
    for (std::size_t k = 0; k < nn; ++k) {
      const double tx = tan[k].x, ty = tan[k].y, nx = -ty, ny = tx;
      const double q[2][2] = {
          {cfg.mpc_q_along * tx * tx + cfg.mpc_q_cross * nx * nx, cfg.mpc_q_along * tx * ty + cfg.mpc_q_cross * nx * ny},
          {cfg.mpc_q_along * tx * ty + cfg.mpc_q_cross * nx * ny, cfg.mpc_q_along * ty * ty + cfg.mpc_q_cross * ny * ny}};
      const double e[2] = {fp[k].x - r[k].x, fp[k].y - r[k].y};
      const double ev[2] = {fv[k].x - vr[k].x, fv[k].y - vr[k].y};
      for (std::size_t ax = 0; ax < 2; ++ax) {
        for (std::size_t j = 0; j <= k; ++j) {
          const double gpa = gp[k * nn + j], gva = gv[k * nn + j];
          const std::size_t i = ax * nn + j;
          g[i] += gpa * (q[ax][0] * e[0] + q[ax][1] * e[1]) + cfg.mpc_q_vel * gva * ev[ax];
          for (std::size_t ax2 = 0; ax2 < 2; ++ax2) {
            for (std::size_t j2 = 0; j2 <= k; ++j2) {
              double add = gpa * q[ax][ax2] * gp[k * nn + j2];
              if (ax == ax2) add += cfg.mpc_q_vel * gva * gv[k * nn + j2];
              H[i * m + ax2 * nn + j2] += add;
            }
          }
        }
      }
    }
    const double wp[2] = {last_w_.x, last_w_.y};
    for (std::size_t ax = 0; ax < 2; ++ax) {
      for (std::size_t j = 0; j < nn; ++j) {
        const std::size_t i = ax * nn + j;
        H[i * m + i] += cfg.mpc_r_delta + cfg.mpc_r_ref + 1e-9;
        g[i] -= cfg.mpc_r_ref * (ax == 0 ? vr[j].x : vr[j].y);
        if (j == 0) {
          g[i] -= cfg.mpc_r_delta * wp[ax];
        } else {
          H[(i - 1) * m + i - 1] += cfg.mpc_r_delta;
          H[i * m + i - 1] -= cfg.mpc_r_delta;
          H[(i - 1) * m + i] -= cfg.mpc_r_delta;
        }
      }
    }
    // Cholesky  H = L L^T,  solve H z = -g
    std::vector<double> L(m * m, 0.0), y(m), z(m);
    for (std::size_t i = 0; i < m; ++i) {
      for (std::size_t j = 0; j <= i; ++j) {
        double sum = H[i * m + j];
        for (std::size_t k = 0; k < j; ++k) sum -= L[i * m + k] * L[j * m + k];
        if (i == j) {
          if (sum <= 1e-12) return false;
          L[i * m + i] = std::sqrt(sum);
        } else {
          L[i * m + j] = sum / L[j * m + j];
        }
      }
    }
    for (std::size_t i = 0; i < m; ++i) {
      double sum = -g[i];
      for (std::size_t k = 0; k < i; ++k) sum -= L[i * m + k] * y[k];
      y[i] = sum / L[i * m + i];
    }
    for (std::size_t i = m; i-- > 0;) {
      double sum = y[i];
      for (std::size_t k = i + 1; k < m; ++k) sum -= L[k * m + i] * z[k];
      z[i] = sum / L[i * m + i];
    }
    w0 = {z[0], z[nn]};
    if (!finite(w0)) return false;
    prediction_.assign(1, p0);
    for (std::size_t k = 0; k < nn; ++k) {
      Vec q = fp[k];
      for (std::size_t j = 0; j <= k; ++j) q = q + Vec{z[j], z[nn + j]} * gp[k * nn + j];
      prediction_.push_back(q);
    }
    reference_ = r;
    return true;
  }

  std::vector<Vec> points_;
  std::vector<double> arc_, profile_;
  std::deque<Sample> history_;
  std::vector<Vec> prediction_, reference_;
  Vec last_cmd_{}, last_w_{}, vel_integral_{};
  double last_time_{0.0}, progress_{0.0}, last_cross_{0.0};
  bool have_time_{false}, force_replace_{false};
};

// The simulator's base_link yaw is the gimbal yaw, which keeps turning.  The yaw
// comes from TF (30 Hz, so up to 33 ms old) and the command is applied at the next
// simulator step: rotate with yaw + rate * (age of the sample + latency).
class YawPredictor {
 public:
  double latency = 0.017;  // s, mean wait until the next 30 Hz simulator step
  // now: current time; stamp: time of the TF sample (same clock); yaw: its yaw.
  void update(double now, double stamp, double yaw) {
    if (have_ && stamp > stamp_ + 1e-6) {
      const double rate = std::clamp(std::remainder(yaw - yaw_, 2.0 * kPi) / (stamp - stamp_), -4.0, 4.0);
      rate_ = stamp - stamp_ < 0.2 ? rate_ + 0.3 * (rate - rate_) : 0.0;
    }
    if (!have_ || stamp > stamp_ + 1e-6) {
      yaw_ = yaw;
      stamp_ = stamp;
      have_ = true;
    }
    age_ = std::clamp(now - stamp_, 0.0, 0.2);
  }
  double predicted() const { return yaw_ + rate_ * (age_ + latency); }
  double rate() const { return rate_; }

 private:
  bool have_{false};
  double yaw_{0.0}, stamp_{0.0}, rate_{0.0}, age_{0.0};
};

}  // namespace sp_tracking
