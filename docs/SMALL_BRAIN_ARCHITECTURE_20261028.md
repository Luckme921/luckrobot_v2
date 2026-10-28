# 小脑当前功能与执行逻辑

日期2026-10-28。当前定位使用SLAM Toolbox localization，不启动AMCL。

## 启动与进程管理

Jetson上电后systemd用户服务luckrobot-supervisor常驻；Linger=yes使其无需SSH登录运行。监督服务不自动启动运动栈。大脑通过ROS2 Domain100请求/robot/system/start，监督服务启动唯一managed_mapping.launch.py，固定idle模式。RUNNING仅表示管理器可用，不代表定位就绪。

stop先调用mapping/pause，等待静止确认再SIGINT关闭所属launch进程组；restart先stop成功再重新启动到idle，旧目标不自动恢复。失败报告FAULT，不强行叠加启动。软件服务不是硬件急停，不支持给断电Jetson远程上电。启动、重复启动、停止、重启链路已实测；真正断电重启自启与大脑双机联调待验收。

## 模式与地图

manual_mapping启动/继续建图，启用导航供人工RViz发送目标。auto_mapping启动全局分支探索。已有mapping会话切换manual/auto保留地图；从定位进入建图可能新建会话。

探索维护全局分支与剩余未知区域，优先深入当前分支；选可达观察点，以Nav2执行，支持滚动交接。暂时受阻不直接算完成；可达范围无有效探索候选且满足确认条件时报告完成。狭窄与小残余未知可以保留，不要求所有灰色消失。完成判定仍受实际传感器及地图质量影响。

完成链：EXPLORATION_COMPLETE → STOPPING → SAVING → MAP_READY → LOCALIZING → NAV_READY。保存前停止任务并确认里程计静止；保存栅格和SLAM位姿图，校验后更新map_store/current.json。失败/取消检查点不覆盖最新完整地图。手动finish_mapping是用户确认覆盖完整，不是自动证明。

load_latest加载最新完整图，需要可信当前位姿。重启或搬动后不能默认停机位置，可能WAITING_INITIAL_POSE。目前旧地图等待预览和自动全局重定位仍未完善。update_map已有内部服务但未完整验收，不建议大脑第一阶段自动使用。

## 定位、规划与执行

轮编码器与IMU经EKF提供连续odom→base_footprint；SLAM Toolbox提供map→odom。建图和定位由同一管理器切换，避免多个定位器竞争TF。

Nav2调用项目自定义全局规划器与局部控制器。规划在代价地图上选择可行路径并进行曲线优化；控制器跟踪路径，结合误差、前视、弯道/目标制动约束及碰撞可行性计算线角速度。速度经平滑与碰撞监测链路交给底盘驱动。具体参数以部署配置为准，不把最高速度理解为每段都应达到。

大脑决定任务与目标，小脑决定安全可行动作。暂停会取消现有导航目标；resume不保证恢复原手动目标。多目标队列应由大脑根据Action结果逐项推进，不连续发送覆盖目标。

## 障碍与残留

全局代价图合并底图、雷达、相机后膨胀；局部代价图用于附近实时控制。相机高度层保护雷达看不到的高处障碍，不能用低处雷达空闲一律清空相机记忆。

新增ObservedStaticLayer：连续有效雷达观测覆盖整个栅格且回波位于其后，才撤销导航底图旧占用；缺测、遮挡、过期扫描不清除，新命中恢复。定位明显修正/地图尺寸变化重置证据。隔离测试已通过，实机残留改善尚需验证。不修改原SLAM定位底图，所以原/map仍可能显示旧黑点；不能宣称原始地图、探索覆盖、保存地图残留已全部解决。

## 状态与大脑

主接口ROS2：/robot/system/status描述运行栈，/mapping/status描述业务状态。都用新鲜周期消息判断在线。mapping的updated_unix是状态转换时间，不能当心跳时间；直接ROS没有TCP的stale字段。

ROS Trigger返回success只表示受理。大脑一次推进一个操作，busy不无限重试。导航必须NAV_READY且状态新鲜。TCP网关保留兼容，但不与ROS大脑同时写入命令。语义检测/语义地图相关代码不代表在此managed启动链中已自动开启；当前监督部署重点为导航与建图。

## 当前边界

未提供任意线程/PID重启接口；未配置硬件远程开机；未完成服务端目标控制权仲裁；未保证断网立即停车；未完成自动全局重定位。正常业务用ROS stop/restart，不直接重启监督systemd服务。故障无法确认静止时需要现场处理。

详细接口见BRAIN_START_AND_CONTROL_DEPLOYED_20261028.md及BRAIN_ROS2_CONTROL_HANDOFF_20261028.md。
