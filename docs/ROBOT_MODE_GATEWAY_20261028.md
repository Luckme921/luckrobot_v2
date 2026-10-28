# 有线模式接口（2026-10-28）

已部署代码，尚未启动外部监听/设置开机自启。主系统常驻，模式管理器串行切换SLAM/Nav2/探索子进程。TCP网关仅绑定10.0.0.51:8765，每行一个JSON，最大4096字节，最多4连接，空闲30秒断线。轻量、不传图像。TCP明文，只用于受控直连网段；需要穿越不可信网络时用TLS/SSH隧道。

报文：`{"id":"唯一请求号","command":"auto_mapping","token":"设备令牌"}`，行尾必须换行。
同进程最近256条id去重；相同id不同命令拒绝。重启不保证去重，断线/超时先查status，不能盲目重发。ok=true仅表示管理器受理，最终看status.state；客户端应每秒查一次status。stale=true不可用于宣告完成。断线不自动取消自主任务；急停独立于TCP。

命令：manual_mapping手动建图/RViz发点；auto_mapping自动探索；navigation加载最新完整图；finish_mapping由客户端明确确认当前手动地图可保存并切定位；pause暂停；resume恢复（mapping模式恢复自动探索，不用于恢复手动模式）；cancel取消；status只读。
已有mapping会话由manual/auto模式切换时保留，非mapping进入建图才新建。接口暂不提供强制丢弃现有地图命令。finish_mapping仅MAPPING/PAUSED/INCOMPLETE可用，它是客户端确认，不能冒充算法自动完成。

## 启动
旧launch先Ctrl+C退出。终端1：
```bash
source ~/ros2_ws/tools/robot_env.sh
ros2 launch my_robot_mapping managed_mapping.launch.py boot_mode:=idle auto_explore:=false
```
终端2首次创建令牌（不要重复覆盖已有令牌）：
```bash
mkdir -p ~/.config/luckrobot
chmod 700 ~/.config/luckrobot
if [ ! -s ~/.config/luckrobot/mode_token ]; then
  (umask 077; openssl rand -hex 32 > ~/.config/luckrobot/mode_token)
fi
source ~/ros2_ws/tools/robot_env.sh
python3 ~/ros2_ws/tools/robot_mode_gateway.py --bind 10.0.0.51
```
终端3只读检查：
```bash
python3 ~/ros2_ws/tools/robot_mode_client.py status
```
手动建图：
```bash
python3 ~/ros2_ws/tools/robot_mode_client.py manual_mapping
```
在带X11转发的MobaXterm终端打开RViz：
```bash
source ~/ros2_ws/tools/robot_env.sh
rviz2 -d ~/ros2_ws/install/my_robot_bringup/share/my_robot_bringup/rviz_config/rviz2_config1.rviz
```
自动建图（会开始自主移动，必须在车旁具备急停条件）：
```bash
python3 ~/ros2_ws/tools/robot_mode_client.py auto_mapping
```
录包在发送自动建图命令之前启动：
```bash
source ~/ros2_ws/tools/robot_env.sh
bash ~/ros2_ws/tools/run_managed_mapping_diag.sh 0
```
暂停：`python3 ~/ros2_ws/tools/robot_mode_client.py pause`。
导航：`python3 ~/ros2_ws/tools/robot_mode_client.py navigation`。
默认地图目录~/ros2_ws/map_store，current.json指向最新完整版本；部分检查点不覆盖current。

导航启动若没有可信位姿会WAITING_INITIAL_POSE，不能直接发送导航目标。/initialpose与/mapping/initial_pose可提供旧地图坐标中的当前位姿，但当前实现尚未补齐等待定位时旧地图预览，因此重启后不能假定RViz已显示正确旧图；不确定则暂不导航。正常建图完成切定位已有可信同一地图位姿，无需人工补定位。

初始7项协议测试、27项管理/版本测试通过；未与真实外部控制器联调、未测网络时延、未自动启动运动。TCP客户端脚本可复制到控制器，令牌通过受信通道配置，勿写入Git。
