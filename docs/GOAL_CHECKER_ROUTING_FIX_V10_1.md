# V10.1 Humble Goal Checker 路由修复

## 1. 现象不是正常恢复

RViz 手动发送 `NavigateToPose` 后，控制器在尚未产生正常路径跟踪速度前就
连续报告：

```text
FollowPath called with goal_checker name  ... which does not exist.
Available goal checkers are: general_goal_checker explore_goal_checker
```

随后出现的清图、`Spin`、`Wait` 和 `BackUp` 是默认行为树针对连续
`FollowPath` 失败执行的恢复流程，并不是局部规划器判断车辆应该倒退。

## 2. 根因

ROS 2 Humble 官方默认导航树的 `FollowPath` 没有填写
`goal_checker_id`。控制器只有一个 goal checker 时会自动使用唯一插件；
V9 为自动探索增加第二个 `explore_goal_checker` 后，空名称不再具有唯一
解释，因此手动导航会在控制器入口直接失败。

自动探索没有同样的问题，因为 `explore_replan_no_recovery.xml` 已显式使用
`goal_checker_id="explore_goal_checker"`。

## 3. 修复

- 保留两个 checker 及其原有容差；
- 手动 `NavigateToPose` 和 `NavigateThroughPoses` 的默认树显式使用
  `general_goal_checker`；
- 自动探索专用树继续显式使用 `explore_goal_checker`；
- 手动树的规划、清图、Spin、Wait、BackUp 结构仍与 Humble 官方默认树
  一致，不改变底盘速度和控制器参数；
- `navigation.launch.py` 在启动时把默认树参数改写为安装目录中的绝对路径，
  不依赖固定用户名或工作空间路径。

## 4. 验收

重启整个导航栈后，先读取两个默认树参数并用 RViz 发送一个开阔区目标。
必须看到 `Received a goal, begin computing control effort` 后正常运动，且
整个过程不得再出现 `goal_checker name  ... does not exist`。没有真实路径
或碰撞问题时，也不应触发 Spin/Wait/BackUp。
