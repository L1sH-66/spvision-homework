# 导航第四讲作业报告

## 任务一：恢复话题通信

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 run qos_debugger qos_debugger_sub
# 在另外的已 source 终端启动发布者
ros2 run qos_debugger qos_debugger_pub
ros2 topic list -t
ros2 topic info /sensor_data -v
ros2 interface show nav_hw_interfaces/msg/SensorData
```

list 查看实际话题名和类型，info -v 查看两个端点及 reliability、durability 等 QoS，interface show 查看消息结构。以代码为准，话题叫 `/sensor_data`，并非题目目录示意中的 `/SensorData`。

原题发布者 best_effort，订阅者 reliable。订阅端要求的可靠性高于发布端能提供的可靠性，因此不兼容。将发布者 reliability 默认值改为 reliable，任务一只修改发布者源码即可恢复通信。QoS 不要求逐项相同：例如 reliable 发布者可以匹配 best_effort 订阅者；反方向不行。修改后需要重新编译并 source。

## 任务二：处理丢帧并正确累计

```bash
ros2 param list /sensor_subscriber
ros2 param get /sensor_publisher rate
ros2 param get /sensor_subscriber callback_delay_ms
ros2 param get /sensor_subscriber depth
ros2 topic hz /sensor_data
# 对运行中的订阅者可用以下命令取消人为延迟
ros2 param set /sensor_subscriber callback_delay_ms 0
```

发布者默认每 10 ms 发布一次，原订阅回调每次 sleep 30 ms，处理能力不足约 33 Hz；KeepLast(10) 的队列无法无限积压。Reliable 负责传输可靠性，不能保证有限应用队列中的所有样本都被回调消费。单纯增大 depth 只能延缓问题。

提交版将订阅者 callback_delay_ms 默认值改为 0，去除人为瓶颈，保留参数方便复现。序号发生正向缺口时额外执行 lost_count_ += lost，修复原代码只告警、不累计的错误。报告丢包率为 lost / (received + lost)。这里的“丢包”指观察到的序号缺口，不能直接归因为网络丢包。第一个收到的 seq 作为基准，不将发现前消息算丢失；回退时重设基准，累计历史统计保留。统计假设单发布者、通常顺序到达；重启、乱序、多发布者和 uint32 序号回绕都不适合据此计算严格网络丢包率。

可另开发布者时保持默认，订阅者以如下命令复现慢消费（不要同时启动两个订阅者实例）：

```bash
ros2 run qos_debugger qos_debugger_sub --ros-args -p callback_delay_ms:=30
```

## 任务三：每秒打印接收帧率

```bash
ros2 run qos_debugger qos_debugger_sub
ros2 run qos_debugger qos_debugger_pub
# 两条 run 分别在两个终端执行
ros2 topic hz /sensor_data
```

在原有一秒 report 定时器内，用 steady_clock 测量实际间隔，接收帧率 = (本次累计收到 - 上次累计收到) / 实际间隔秒数，再保存计数和时间。steady_clock 不受系统时间校正影响，定时器调度延迟时也不会把间隔强行当成一秒。没有新消息时打印 0 Hz。topic hz 是额外订阅者自己的测量，可作为对照，不能代替本节点接收帧率。

完成内容：默认 reliable 对 reliable；默认取消 sleep；补齐 lost_count_；每秒报告实际接收帧率。未改变原消息接口和节点名。使用 AI 协助实现和整理。
