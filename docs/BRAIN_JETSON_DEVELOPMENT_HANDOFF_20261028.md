# LuckRobot 大脑 Jetson 开发交接与接口规范

版本：2026-10-28 · 对应当前第一版 TCP 模式网关（schema 1）

本文可单独交给另一台 Jetson 上的 AI 开发者。请以本文“当前已实现”为兼容契约；“后续扩展”不是已可调用接口。开始开发前先只读检查大脑设备环境，不要自动开始机器人运动。

## 1. 开发任务与边界

请在另一台 Jetson（以下简称“大脑”）开发轻量模式控制客户端，连接底盘 Jetson，实现：

1. 配置有线连接、令牌、连接超时。
2. 查询并显示机器人状态、地图版本、受阻/失败原因。
3. 按用户明确指令执行手动建图、自动建图、加载地图导航、暂停、恢复、取消。
4. 正确区分请求受理、任务完成、地图保存成功、定位导航就绪。
5. 断线重连、状态过期处理、请求去重与审计日志。
6. 先提供 CLI 和可复用 Python API；如已有大脑 UI，再接入按钮与状态展示。无需为了这项功能新建复杂网页或安装 ROS。

第一阶段不负责电机控制、规划、SLAM、图像流或模型推理。不要在大脑端另起导航栈，不通过 SSH 杀进程实现日常模式切换，不通过 `/cmd_vel` 绕过底盘导航。

大脑开机默认仅连接和查询状态，不自动发送 auto_mapping、resume 或导航目标。底盘已在运行时不得因客户端重启而重建地图。

## 2. 当前机器人架构

底盘：Jetson Orin Nano Super 8 GB，ROS 2 Humble，工作空间 `/home/nvidia/ros2_ws`，ROS_DOMAIN_ID=100。

- 编码器＋IMU经过 EKF，输出连续里程计与 odom → base_footprint。
- SLAM Toolbox 建图；保存后使用 SLAM Toolbox localization 模式，提供 map → odom。
- 当前不是 AMCL 定位。不要启动 AMCL 与 SLAM Toolbox 争抢同一 TF。
- Nav2 使用本项目自定义规划器与控制器。
- 雷达和深度相机提供动态障碍观测。
- 模式管理器控制探索、地图保存、定位切换与导航就绪状态。
- TCP 网关将大脑命令转为管理器服务，不直接发运动目标。

已观察到一次完整实机状态链：探索完成 → STOPPING → SAVING → MAP_READY → LOCALIZING → NAV_READY。该结果不等于所有动态障碍问题或外部大脑联调已验收。

物理背景供未来扩展参考：整机约9.5 kg，DDSM115轮电机，差速底盘。第一阶段不得据此自行修改速度、加速度或碰撞范围。

## 3. 已实现与未实现

| 项目 | 当前状态 |
|---|---|
| TCP模式网关、CLI客户端 | 已部署第一版 |
| 协议单元测试 | 7项通过（此前记录） |
| 管理器/地图版本测试 | 27项通过（此前记录） |
| 实机探索完成、保存、定位切换 | 已观察到 NAV_READY |
| 外部大脑真实网络联调 | 待完成 |
| 网关常驻/开机自启 | 尚未配置，需现场检查监听 |
| TCP坐标目标接口 | 尚未实现 |
| TCP初始位姿接口 | 尚未实现 |
| 重启后无人工全局重定位 | 尚未实现 |
| 等待初始位姿时旧地图预览 | 尚未补齐，不要假定RViz显示正确旧图 |
| TCP地图下载、地图列表、切换指定地图 | 尚未实现 |
| 日常地图更新的TCP命令 | 尚未暴露，不能发送虚构命令 |
| 动态障碍残留 | 有修复在验证中，不属于网关验收成功的证明 |

## 4. 网络与资源设计

底盘有线地址 `10.0.0.51/24`，监听TCP端口8765。大脑可配置未占用的 `10.0.0.52/24`。必须先检查已有设备地址，不能覆盖正在使用的地址。

两端网线直连或同一受控交换机连接。有线接口不设置默认网关和DNS，保留Wi-Fi原有默认路由。底盘Wi-Fi曾使用192.168.3.32，仅用于维护，不作为该接口目标地址。不要把大脑的ROS多播、摄像头原始图像引入此接口。

协议只传小型JSON指令与状态；Python标准库socket/json足够，第一阶段无需Redis、数据库、消息队列或ROS依赖。没有实测时延/SLA，不能宣称毫秒级保证。

大脑只读检查命令（Linux）：

```bash
ip -br address
ip route
ip route get 10.0.0.51
ping -c 4 10.0.0.51
```

如使用NetworkManager，先执行 `nmcli connection show`，识别实际网口与连接名后再配置。不要照抄假设的eth0，也不要改动当前SSH依赖的接口。

## 5. 协议：TCP JSON Lines

- 服务地址：10.0.0.51:8765。
- UTF-8编码，每个请求和响应都是一行JSON，以字节LF（\n）结束。
- 支持一个连接多次请求，但不是服务端主动推送。
- 客户端按行累积解析；一次recv可能得到半条或多条报文。
- 单条请求最多4096字节，建议包含换行也不超过4096字节。
- 最多4个并发连接；空闲读取30秒会断开。
- 推荐大脑仅使用一个连接，串行请求；每秒查询一次status。
- 建议连接超时3秒、响应超时8秒，后台重连退避1、2、4、8秒并封顶10秒，增加少量随机抖动。
- 单连接读写由一个工作线程/事件循环管理；不要阻塞UI，不要让多个线程同时读同一socket。
- 当前网关命令处理串行，普通模式请求可能占用数秒；pause不能当实时急停。

### 5.1 请求

```json
{"id":"b85c4348-6f80-4c37-8994-600dfd3c1907","command":"status","token":"REPLACE_WITH_DEVICE_TOKEN"}
```

`id`是1～80字符的字符串，推荐UUID4；每个新逻辑请求生成新ID。`command`取下表规定值。`token`必须与底盘文件完全匹配，读取时去除首尾换行。

### 5.2 当前命令集合

| command | 功能 | 重要语义 |
|---|---|---|
| status | 查询状态 | 只读，不触发机器人运动 |
| manual_mapping | 手动建图 | 主人用RViz发点；不是自动探索 |
| auto_mapping | 自动探索建图 | 会自主运动，完成后自动保存并切定位 |
| finish_mapping | 确认手动建图完成 | 客户端明确认可覆盖范围，不是算法替用户判断完成 |
| navigation | 加载最新完整地图 | 可能进入WAITING_INITIAL_POSE；不发送导航目标 |
| pause | 暂停当前流程 | 等待PAUSED确认，不能代替物理急停 |
| resume | 恢复 | mapping模式会恢复自动探索，不用于恢复手动建图 |
| cancel | 取消 | 建图时可能保存不完整检查点 |

已有mapping会话在manual/auto之间切换会保留；从其他模式进入建图可能开始新会话。当前没有“丢弃现有地图并强制重新建图”的TCP接口。

manual_mapping、auto_mapping、navigation通常允许从IDLE/MAPPING/PAUSED/NAV_READY/INCOMPLETE/CANCELLED/WAITING_INITIAL_POSE请求；服务端仍会检查busy和实际状态。在EXPLORING时切模式应先pause并等到PAUSED，不要连续挤入命令。

finish_mapping仅在mode=mapping且state为MAPPING/PAUSED/INCOMPLETE时接受。不要自动用它把BLOCKED或INCOMPLETE冒充探索成功。

已是NAV_READY时不要每秒重复发送navigation。现实现navigation对应load_latest，重新调用可能再次要求初始位姿。需要恢复暂停的定位会话时优先按实际状态使用resume，而不是盲目重新加载。

### 5.3 响应示例（示意字段值）

模式请求受理：

```json
{
  "id": "b85c4348-6f80-4c37-8994-600dfd3c1907",
  "ok": true,
  "message": "accepted; final result on /mapping/status",
  "status": {
    "schema": 1,
    "session_id": "SESSION_ID",
    "state": "IDLE",
    "reason": "awaiting_mode_command",
    "mode": "",
    "retries": 0,
    "map": null,
    "updated_unix": 1793158000.0,
    "stale": false
  }
}
```

回复中的status可能仍是切换前状态。ok=true只代表受理，不代表命令执行结束。

status查询响应：

```json
{
  "id": "ANOTHER_UUID",
  "ok": true,
  "status": {
    "schema": 1,
    "session_id": "SESSION_ID",
    "state": "NAV_READY",
    "reason": "localization_health_stable",
    "mode": "localization",
    "retries": 0,
    "map": {
      "schema": 1,
      "map_id": "MAP_VERSION_ID",
      "prefix": "/home/nvidia/ros2_ws/map_store/versions/MAP_VERSION_ID/map",
      "complete": true
    },
    "updated_unix": 1793158142.0,
    "stale": false
  }
}
```

完整map记录还可包含session_id、saved_unix、pose_at_save、reason、hashes；客户端允许附加字段，不应依赖字段顺序。路径是底盘本机路径，不是大脑本机路径。

错误可能为：

```json
{"ok":false,"error":"unauthorized"}
```

```json
{"ok":false,"error":"invalid request"}
```

```json
{"id":"REQUEST_ID","ok":false,"message":"busy; inspect /mapping/status","status":{}}
```

部分错误没有id，解析器不能因此崩溃。具体message还可能是`manager status stale`、`service unavailable`、`command not valid in state ...`、`outcome_unknown; query status before another request`。

### 5.4 去重、超时与连接恢复

服务进程缓存最近256条非status命令ID。同ID同命令返回原响应；同ID不同命令拒绝。缓存不是永久存储，网关重启后不保留。重复响应中的status也是原快照，必须另发新status查询。

- 状态查询用新ID，失败可安全重试。
- 业务请求发出后超时，结果不确定；重连后先查询状态，不立即重新建图。
- 同一次请求在确定需要传输层重试时保留ID；明确被busy等拒绝、稍后重新提交的新尝试使用新ID，否则会一直拿到旧拒绝结果。
- 不要因为状态停留几秒就重发命令。
- 多个客户端可能同时操作；第一阶段约定只有一个大脑作为模式写入者，其余客户端只查状态。当前没有控制权租约或命令完成ID关联机制。
- TCP断线不会自动取消自主任务。不要把网络断开等同于停车，也不要在重连时擅自恢复运动。

## 6. 状态机与完成条件

| state | 含义 | 大脑处理 |
|---|---|---|
| STARTING | 管理器初始化 | 等待 |
| IDLE | 等待模式指令 | 可显示模式选择 |
| MAPPING | 建图模式 | reason为manual_mapping_ready时可提示RViz发点 |
| EXPLORING | 自动探索 | 显示探索中，可暂停 |
| BLOCKED | 受阻，可能自动重试 | 显示reason和retries，不当完成 |
| EXPLORATION_COMPLETE | 可达范围探索完成 | 等待保存，不立刻宣布导航就绪 |
| STOPPING | 停止探索/导航以切换 | 等待，不插入运动命令 |
| SAVING | 保存地图 | 等待 |
| MAP_READY | 地图已保存校验 | 仍须等待定位 |
| LOCALIZING | 加载定位或恢复定位健康 | 等待 |
| NAV_READY | 定位和导航就绪 | fresh时可显示就绪 |
| WAITING_INITIAL_POSE | 缺少可信初始位姿 | 提示定位需要处理 |
| PAUSED | 已暂停 | 保留会话，等待用户决定 |
| INCOMPLETE | 未完成，可能已保存检查点 | 不替换“成功完成”状态 |
| CANCELLED | 已取消 | 不自动恢复 |
| ERROR | 流程错误 | 显示reason，保留日志 |
| UNAVAILABLE | 网关暂未收到状态 | 视为不可用 |

遇到未知state显示原值并保守等待，不崩溃、不允许自动运动。

导航就绪判断：连接健康、最近响应新鲜、status.stale为false、state严格等于NAV_READY。需要已有地图的任务还应检查map存在且complete=true。

重要：updated_unix目前记录的是状态转换时间，不是每秒心跳时间。NAV_READY长期不变时它可以很旧。用本地收到响应的单调时钟时间和stale字段判断通信新鲜度，不能仅凭updated_unix判超时。网关stale超过约3秒未收到管理器消息时为true。

1秒轮询可能错过短暂MAP_READY等中间状态，这是正常的；最终NAV_READY带完整map记录可作为结果，不要求客户端亲眼收到每个中间状态。

## 7. 地图生命周期

默认目录：`/home/nvidia/ros2_ws/map_store`。

- `current.json`指向最新完整且通过保存校验的地图版本。
- `versions/<map_id>/`保存map.yaml、map.pgm、map.posegraph、map.data等文件。
- `session.json`为管理器状态持久记录，不代替实时健康查询。
- 不完整检查点不会覆盖最新完整地图。
- 地图版本ID用于记录与关联，不要把文件路径或保存时位姿当作实时定位。

重启后机器人可能被搬动或转向，不能默认沿用停机位置。当前WAITING_INITIAL_POSE流程由底盘端接受ROS初始位姿，TCP尚未提供。旧地图预览也未补齐，因此大脑只能明确提示限制，不能伪造“自动定位成功”。同一建图会话完成后自动切定位有可信同地图位姿，已实机走通过。

定位模式不自动等价于持久更新底图。实时障碍层更新、SLAM底图维护、语义地图更新是不同工作，当前协议不包含这些地图内容传输接口。

## 8. 底盘端启动步骤（交由现场操作者执行）

先确认没有另一套机器人launch；不要由大脑自动执行pkill。正在运行的健康会话不应为了联调重复启动。

终端1，底盘Jetson：

```bash
source ~/ros2_ws/tools/robot_env.sh
ros2 launch my_robot_mapping managed_mapping.launch.py \
  boot_mode:=idle auto_explore:=false
```

终端2，首次创建令牌且不覆盖已有令牌：

```bash
mkdir -p ~/.config/luckrobot
chmod 700 ~/.config/luckrobot
if [ ! -s ~/.config/luckrobot/mode_token ]; then
  (umask 077; openssl rand -hex 32 > ~/.config/luckrobot/mode_token)
fi
source ~/ros2_ws/tools/robot_env.sh
python3 ~/ros2_ws/tools/robot_mode_gateway.py --bind 10.0.0.51
```

确认监听（底盘端）：

```bash
ss -ltn | grep ':8765'
```

令牌通过可信渠道复制到大脑，建议也保存在`~/.config/luckrobot/mode_token`并chmod600；不要输出到聊天、日志或Git。TCP当前是明文，仅用于受控直连网段，不能直接映射到公网。

底盘本机只读检查：

```bash
python3 ~/ros2_ws/tools/robot_mode_client.py status
```

不要因为尚未监听就自行判断底盘ROS服务故障，网关与管理器是两个独立进程。

## 9. 大脑端建议实现

建议Python3标准库实现，目录示例：

```text
robot_brain_bridge/
  client.py          # TCP连接、分帧、超时、请求ID
  state.py           # 状态分类与就绪判定
  cli.py             # 命令行入口
  config.example.json
  tests/             # fake TCP server、状态与异常测试
  README.md
```

示例配置（由开发者实现读取，不是底盘现有配置文件）：

```json
{
  "host": "10.0.0.51",
  "port": 8765,
  "token_file": "~/.config/luckrobot/mode_token",
  "connect_timeout_seconds": 3,
  "response_timeout_seconds": 8,
  "status_poll_seconds": 1,
  "auto_start_motion": false
}
```

最小单次查询示例（不是完整重连客户端）：

```python
import json
import socket
import uuid
from pathlib import Path

token = Path.home().joinpath('.config/luckrobot/mode_token').read_text().strip()
request = {'id': str(uuid.uuid4()), 'command': 'status', 'token': token}
with socket.create_connection(('10.0.0.51', 8765), timeout=3) as sock:
    sock.settimeout(8)
    sock.sendall((json.dumps(request) + '\n').encode('utf-8'))
    stream = sock.makefile('rb')
    line = stream.readline(262145)
    if not line or len(line) > 262144 or not line.endswith(b'\n'):
        raise RuntimeError('connection closed or invalid response framing')
    response = json.loads(line.decode('utf-8'))
    # Only print this response; never print request/token.
    print(json.dumps(response, ensure_ascii=False, indent=2))
```

生产实现需要持久连接、请求队列、断线后丢弃旧半包、JSON格式检查、本地状态新鲜度和日志脱敏。建议日志记录request_id、command、耗时、是否受理、state、reason、session_id、map_id；禁止记录token。

业务调用与状态轮询共用串行通信队列。模式命令在队列中优先于下一次轮询；不要长时间阻塞GUI。当前服务端没有异步事件推送，不要等待未实现的推送事件。

## 10. 联调与验收顺序

### A. 不让车移动的联调

1. 确认网段、路由和端口。
2. 正确令牌status成功；错误令牌拒绝且不反复重试。
3. 连续status查询至少5分钟，记录成功率、往返延迟p50/p95/最大值，不编造性能结论。
4. fake server测试半包、粘包、无换行、非法JSON、超长响应、断线与超时。
5. 对state未知、map=null、status缺失、stale=true、错误响应无id均能正常处理。
6. 网络断开时UI立即标识连接异常，不把最后一次NAV_READY长期当成当前就绪。
7. 客户端重启只查询状态，不重新启动建图。

请求去重与未知结果用模拟服务测试；不要为了测试重复ID，向真实机器人反复发送auto_mapping。

### B. 有人看护、可急停时测试

1. manual_mapping受理，等待MAPPING且reason说明手动模式就绪。
2. 由主人通过RViz发送一个安全短目标；大脑不发坐标。
3. pause后等PAUSED；要继续手动建图应请求manual_mapping，不使用mapping模式下的resume。
4. 用户明确允许后auto_mapping，观察EXPLORING。
5. 完成后自动保存、定位，最终NAV_READY且map.complete=true。
6. 查询和展示地图版本；不要自动再次navigation。
7. 下一轮再测试暂停恢复、暂时受阻、取消及断线期间行为。

重启后加载地图作为单独用例：允许WAITING_INITIAL_POSE作为正确结果；尚未补齐的自动重定位不能作为本阶段通过项。

### C. 验收交付物

- 大脑端源代码、配置样例、安装及运行命令。
- CLI和API使用示例。
- 自动测试结果、真实status联调统计。
- 支持的指令/状态清单、已知限制。
- 联调日志（脱敏）和失败恢复说明。
- 如提供systemd，只自动启动客户端监视，不自动启动运动；先手动联调通过再启用。

## 11. RViz与录包（底盘侧）

带X11转发的MobaXterm终端打开RViz：

```bash
source ~/ros2_ws/tools/robot_env.sh
rviz2 -d ~/ros2_ws/install/my_robot_bringup/share/my_robot_bringup/rviz_config/rviz2_config1.rviz
```

已知历史上X11/OpenGL存在兼容问题，不能假设该问题已消失。大脑接口开发不应依赖远程RViz画面传输。

开始实际模式/运动测试之前录包：

```bash
source ~/ros2_ws/tools/robot_env.sh
bash ~/ros2_ws/tools/run_managed_mapping_diag.sh 0
```

结束时按一次Ctrl+C，只结束录包，不停止探索或导航。状态查看：

```bash
source ~/ros2_ws/tools/robot_env.sh
python3 ~/ros2_ws/tools/watch_mapping_status.py
```

Ctrl+C退出状态显示也不会停止机器人。需要暂停应调用pause并确认PAUSED。

## 12. 后续扩展（本阶段不能冒充已实现）

将来需要大脑发送目标时，在底盘新增受控目标接口，再由底盘统一调用Nav2 Action。应明确：

- map_id和frame_id=map，位置单位米、角度弧度。
- goal_id、受理/执行/成功/失败/取消状态。
- 定位就绪与状态新鲜度校验。
- 大脑与RViz的目标控制权、抢占规则。
- 断线后任务继续还是取消的明确策略。
- 目标超时、结果持久性及重试幂等性。

坐标报文格式尚未定稿，不要直接向现网发送navigate_to_pose或x/y/yaw字段并认为会生效。

后续还包括初始位姿、旧地图预览、地图版本列表、地图内容读取、受控地图更新和服务开机自启。任何扩展需同步底盘协议与测试，不自行扩大当前command白名单。

## 13. 提供给AI开发者的执行要求

请先阅读本文件并检查本机已有项目结构，复用现有工程。先实现并测试无运动的status通信，再接业务接口。缺少令牌、网口名称等必要信息时明确指出需要哪一项；不要要求用户安装不必要的大型依赖。

请区分“已编码”“模拟测试通过”“真实控制器联调通过”“实车运动验收通过”。最终报告列出实际完成内容、测试证据、未解决项和运行命令，不能把TCP成功连接描述成整机导航完全可靠。

## 14. 底盘端参考源码位置

以下均为底盘Jetson路径，可通过授权SSH只读查看：

- `/home/nvidia/ros2_ws/tools/robot_mode_gateway.py`
- `/home/nvidia/ros2_ws/tools/robot_mode_client.py`
- `/home/nvidia/ros2_ws/src/my_robot_mapping/scripts/map_session_manager.py`
- `/home/nvidia/ros2_ws/src/my_robot_mapping/scripts/map_versions.py`
- `/home/nvidia/ros2_ws/src/my_robot_mapping/launch/managed_mapping.launch.py`

不要把本文件所在Windows路径当成大脑Linux路径。只需把本Markdown文件传给另一台Jetson的AI即可作为开发输入；令牌应另行安全配置。
