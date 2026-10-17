# 双 Jetson ROS 2 有线通信方案

## 当前拓扑

- 辅助 Jetson：`10.0.0.50/24`，接口 `enP8p1s0`；
- 机器人主 Jetson：`10.0.0.51/24`，接口 `enP8p1s0`；
- 两端 ROS 2 Humble，使用 `ROS_DOMAIN_ID=100`；
- 主 Jetson 同时保留 Wi-Fi `192.168.3.32` 作为 SSH/维护链路。

这两个有线地址属于同一子网，可以承载 ROS 2 DDS 通信。只有相同 Domain ID
还不足以证明通信已成功；两端还必须保证：

```bash
export ROS_DOMAIN_ID=100
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

防火墙必须允许 DDS 使用的 UDP 组播和单播。当前 `fastdds_shm.xml` 同时声明 SHM
和 UDPv4：同一主机的 Fast DDS 参与者可使用 SHM，跨主机通信必须使用 UDPv4。
但是该文件尚未限定网卡，因此 DDS 也可能公布 Wi-Fi 地址。先完成双向实测；后续
再根据实测生成仅允许 `enP8p1s0` 的配置，避免在 VSLAM 容器通信未验证前改变
稳定系统。

## 双向连通验收

先验证 IP：

```bash
# 在 10.0.0.51
ping -I enP8p1s0 -c 4 10.0.0.50

# 在 10.0.0.50
ping -I enP8p1s0 -c 4 10.0.0.51
```

主 Jetson `10.0.0.51` 发布：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=100
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 daemon stop
ros2 topic pub -r 2 /luckrobot/network_probe \
  std_msgs/msg/String "{data: main_10.0.0.51}"
```

辅助 Jetson `10.0.0.50` 接收：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=100
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 daemon stop
timeout -s INT 8s ros2 topic echo /luckrobot/network_probe --once
```

随后交换发布端和接收端再做一次。两个方向都成功，才算 topic 通信通过。之后再
测试服务与动作，不应直接用相机图像作为第一项网络测试。

## 节点部署边界

机器人主 Jetson 保留所有实时和安全节点：传感器驱动、轮速/IMU EKF、
slam_toolbox、Nav2、Collision Monitor、自定义规划器/控制器、VSLAM 及其健康门控。

辅助 Jetson 只负责高层命令：

- 发送 `NavigateToPose` 目标；
- 请求开始/停止自动探索；
- 读取低频地图、状态和诊断；
- 后续运行语义理解、任务规划和对话。

不要让辅助 Jetson 直接发布 `/cmd_vel*`，也不要把 Collision Monitor 或底盘控制
迁移过去。DDS 只会向匹配的订阅者发送用户数据，因此没有订阅的高带宽相机话题
不会持续占用有线带宽；但发现流量仍然存在。若辅助端订阅原始图像、点云或完整
代价地图，则会产生明显网络与序列化负载。

当前 `luckrobot_system.launch.py` 会启动建图/定位与 Nav2，但不会启动
`auto_explore.launch.py`，符合“主机待命、辅助 Jetson 下发任务”的边界。

