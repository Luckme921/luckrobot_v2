# 地图任务管理接口与版本流程（2026-10-27）

## 范围

新增可选 `managed_mapping.launch.py`，管理已有的全局分支探索、SLAM Toolbox 和 Nav2 生命周期。不修改运动控制、规划器或速度参数；原启动命令默认行为不变。当前定位后端仍为 **SLAM Toolbox localization**，不是 AMCL，也没有启用 VSLAM。

## 自动流程

1. 启动前检查同域内是否已有底盘、导航、SLAM 或探索节点，冲突时拒绝重复启动。
2. 新建地图时先启动传感器和 SLAM，Nav2 暂不激活。`auto_explore:=true` 时地图就绪后自动开始探索；为首次验证建议先设为 false，再调用 resume。
3. 真正收到探索完成后：停止目标并确认静止 → 保存位姿图、栅格图和清单 → 校验所有文件 → 更新完整地图指针。
4. 停止旧 SLAM，等待节点退出，再以刚保存的位置启动定位模式。不会同时运行两个 map→odom 发布者。
5. 定位健康稳定后激活 Nav2，发布 `NAV_READY`；后续业务可开始发送导航目标。
6. 受阻和人工暂停分开。受阻默认等待 30 秒，最多自动恢复 3 次；保留当前分支记忆。耗尽后保存未完成检查点，状态 `INCOMPLETE`，不冒充完成、不覆盖旧完整地图。

## 对外接口

所有控制服务均为 `std_srvs/srv/Trigger`，返回 success 只表示已接受；以状态话题确认执行结果。无效状态下的请求会被拒绝。

| 接口 | 类型 | 用途 |
|---|---|---|
| `/mapping/status` | std_msgs/String，JSON | 当前状态、原因、会话 ID、重试次数和地图清单；每秒发布，保留最后值 |
| `/mapping/map_ready` | std_msgs/String，JSON | 已校验完整地图的版本、文件路径、校验和；保留最后值 |
| `/mapping/navigation_ready` | std_msgs/Bool | 仅 NAV_READY 为 true；是定位导航业务放行信号，不是整个探索栈的运行状态 |
| `/mapping/pause` | Trigger | 暂停并停车，保留探索进程和分支记忆 |
| `/mapping/resume` | Trigger | 从暂停/未完成/取消恢复；新地图静止准备完成后也可开始探索 |
| `/mapping/cancel` | Trigger | 停止任务，建图模式保存未完成检查点 |
| `/mapping/save_checkpoint` | Trigger | 保存未完成检查点并暂停 |
| `/mapping/load_latest` | Trigger | 请求加载最新完整地图，缺少可信当前位姿时进入 WAITING_INITIAL_POSE |
| `/mapping/update_map` | Trigger | 从定位就绪状态加载完整位姿图进入更新探索；成功后生成新版本 |
| `/mapping/initial_pose` | geometry_msgs/PoseWithCovarianceStamped | 上层定位/已知停靠点提供当前 map 位姿；需当前时间戳、合法四元数 |

主要状态：STARTING、MAPPING、EXPLORING、BLOCKED、EXPLORATION_COMPLETE、STOPPING、SAVING、MAP_READY、LOCALIZING、NAV_READY、WAITING_INITIAL_POSE、PAUSED、CANCELLED、CHECKPOINT_SAVED、INCOMPLETE、ERROR。

暂停/取消可以由任何上层程序调用，无需人工操作终端。但显式暂停/取消不会被超时自动撤销，否则会违背操作意图。需要无人值守时，上层程序调用 resume；临时受阻由管理器自动重试。进程故障、定位不收敛和保存失败不自动重启整机。

## 地图存储

默认 `~/ros2_ws/map_store/versions/<唯一版本ID>/` 包含：

- `map.posegraph`、`map.data`：SLAM 官方序列化接口的位姿图。
- `map.pgm`、`map.yaml`：收到的 OccupancyGrid 快照，标准 trinary 格式。
- `manifest.json`：版本 ID、保存时间、会话、保存位姿、完整标志和 SHA256。

`current.json` 仅指向校验成功的完整地图；`checkpoint.json` 指向部分地图。采用原子替换指针，保存不全不修改 current，旧版本不自动删除。语义地图未来应引用此 map_id；本次没有迁移语义数据库。

## 重启定位

`boot_mode:=auto`：没有完整地图则新建；有完整地图则准备加载。机器人重启后可能被搬动，因此**不把保存时的位姿当作当前位姿**。

已知机器人停在固定位置时，可由启动系统提供 `boot_pose_json:='[x,y,yaw]'`。否则状态保持 WAITING_INITIAL_POSE，等待上层定位模块向 `/mapping/initial_pose` 提供可信位姿。任意位置开机的全局自动重定位尚未实现，不能宣称已具备。定位健康检查也不能替代真实位置的准确性验证。

## 首次实机验证命令

先结束旧系统 launch。终端一：

```bash
source ~/ros2_ws/tools/robot_env.sh
ros2 launch my_robot_mapping managed_mapping.launch.py boot_mode:=mapping auto_explore:=false
```

终端二检查状态，等 MAPPING：

```bash
source ~/ros2_ws/tools/robot_env.sh
ros2 topic echo /mapping/status
```

Ctrl+C 只退出 echo 后开始录包：

```bash
bash ~/ros2_ws/tools/run_managed_mapping_diag.sh 0
```

终端三，在车旁具备急停条件时开始探索：

```bash
source ~/ros2_ws/tools/robot_env.sh
ros2 service call /mapping/resume std_srvs/srv/Trigger '{}'
```

需要暂停：

```bash
ros2 service call /mapping/pause std_srvs/srv/Trigger '{}'
```

本模式不要同时运行旧的 run_auto_diag.sh，它会另启探索器；run_custom_nav_diag.sh 也不适用，因为它拒绝存在探索器。新录包脚本只记录，不启动/停止机器人任务。

## 验证方法与限制

`test/test_map_session.py`：无硬件的事务、状态与取消回归。

`test/integration_map_session.py`：强制独立 ROS_DOMAIN_ID=193，用合成静止房间扫描、里程计和 TF，调用真实 SLAM Toolbox 进行序列化和加载。设置 MAP_SESSION_REAL_NAV=1 使用真实 Nav2 生命周期；不启动底盘、不发送运动目标。测试用的完成事件由测试工具注入，因此不验证房间探索覆盖率。

```bash
source ~/ros2_ws/tools/robot_env.sh
python3 ~/ros2_ws/src/my_robot_mapping/test/test_map_session.py
ROS_DOMAIN_ID=193 ROS_LOCALHOST_ONLY=1 MAP_SESSION_REAL_NAV=1 \
  python3 ~/ros2_ws/src/my_robot_mapping/test/integration_map_session.py
```

部署前备份：`~/ros2_ws/backups/map_session_20261027/predeploy.tar.gz`。新管理器是可选入口，原手动导航/探索入口仍保留。

实现时核对了 [SLAM Toolbox Humble 定位源码](https://github.com/SteveMacenski/slam_toolbox/blob/humble/src/slam_toolbox_localization.cpp) 与 [扫描处理、TF 发布源码](https://github.com/SteveMacenski/slam_toolbox/blob/humble/src/slam_toolbox_common.cpp)。


## 本轮验证结果

2026-10-27：Jetson 上 23 项回归通过；真实 SLAM Toolbox + 真实 Nav2 的隔离集成测试通过。覆盖新建、完整保存、切换定位、暂停恢复、重建管理器及模拟传感器后等待初始位姿、加载旧图、进入更新建图、取消后保存检查点并保留旧完整地图。测试没有驱动真实车轮，也没有验证实地探索的完成判定。

重启测试最初仅在同一 Python 进程中重建管理器时，新订阅未收到仍存在的模拟 TF 发布者的数据；外部 tf2_echo 正常。按整机重启情景重建模拟传感器后通过。尚不能宣称覆盖所有“只重启管理器、传感器进程不重启”的通信故障；遇到 TF 不全时仍拒绝进入 NAV_READY。

隔离测试日志：`diagnostics/map_session_20261027/integration_pass.log`。实机首次验证仍待用户在旁执行上面的命令。
