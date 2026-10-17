# 自动探索诊断安全门修复 V10.7.1

## 目的

修复 `tools/run_auto_diag.sh` 在 Jetson CPU/DDS 繁忙时，把正常工作的
Collision Monitor 安全速度链误判为失效的问题。

本补丁只修改诊断脚本，不修改底盘驱动、Nav2 参数、自定义规划器、
自定义控制器、行为树、代价地图、EKF 或 V10.7 探索算法。

## 修复内容

- 把单次零速度发布改为持续数秒的 10 Hz 多帧零速度探针，给 DDS 端点
  足够的发现和匹配时间。
- 若动态探针因系统负载未收到样本，检查唯一完整的安全链拓扑：
  `/cmd_vel -> collision_monitor -> /cmd_vel_safe -> chassis_driver_node`。
- 只有拓扑中 `/cmd_vel_safe` 恰好存在一个发布者和一个订阅者，且节点名
  分别为 `collision_monitor` 和 `chassis_driver_node` 时才允许继续。
- 任何明确的 `inactive`、`unconfigured`、节点缺失或重复节点仍会拒绝启动
  自动探索。

## 部署后运行

统一导航系统保持运行，无需重启终端 1。新终端执行：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
bash ~/ros2_ws/tools/run_auto_diag.sh 300
```

诊断脚本会自行启动 `auto_explore.launch.py`，不要再启动第二个探索节点，
也不要在 RViz 中手动发送导航目标。
