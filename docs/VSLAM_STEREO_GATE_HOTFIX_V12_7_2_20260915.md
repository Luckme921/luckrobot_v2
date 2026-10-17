# LuckRobot VSLAM 双目门控启动修复 V12.7.2

## 结论

V12.7.1 的交替相机参数已经被 Gemini 336L 固件 1.4.60 正确接受，原始左右 IR 和
深度数据均在发布。`/vslam/stereo/...` 为零样本的直接原因是门控节点启动失败：

```text
AttributeError: can't set attribute 'subscriptions'
```

Humble 的 `rclpy.Node.subscriptions` 是只读属性。V12.7.2 仅把订阅对象引用保存到
节点自己的 `_subscription_refs` 字段，不改变过滤算法、相机配置、深度避障、Nav2、
生产 EKF、TF 或容器 cuVSLAM 参数。

交替模式不是永久关闭物理激光：序列 0 保持激光开启并供深度避障使用，序列 1 关闭
激光并供双目视觉里程计使用。这样避免随相机移动的主动红外纹理干扰视觉跟踪，同时
保留主动深度。是否确实改善本车 cuVSLAM，仍由后续动态 A/B 数据决定。

## 部署

先关闭终端 1 的统一系统 Launch。容器可以暂时保持停止状态，然后在宿主机执行：

```bash
cd ~/ros2_ws

sha256sum luckrobot_vslam_stereo_gate_hotfix_v12_7_2_20260915.zip

unzip -o \
  luckrobot_vslam_stereo_gate_hotfix_v12_7_2_20260915.zip \
  -d ~/ros2_ws

sha256sum -c \
  docs/VSLAM_STEREO_GATE_HOTFIX_V12_7_2_20260915_MANIFEST.sha256

chmod +x src/my_robot_bringup/scripts/vslam_stereo_monotonic_gate.py

source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select my_robot_bringup \
  --allow-overriding my_robot_bringup
source ~/ros2_ws/install/setup.bash
```

重新启动终端 1，仍使用 V12.7.1 的统一启动命令。先不要启动 cuVSLAM，另一个宿主机
终端执行：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 node list | grep vslam_stereo_monotonic_gate

python3 ~/ros2_ws/tools/check_vslam_stereo_sync.py \
  --duration 20 \
  --expected-rate 15
```

必须看到门控节点、左右约 15 Hz、两边 `nonmonotonic=0`、`RESULT=PASS`。如果通过，
再启动已经构建完成的容器 cuVSLAM，并运行30秒健康检查。动态运动仍需等健康检查通过。

