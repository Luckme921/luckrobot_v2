import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    explore_behavior_tree = os.path.join(
        get_package_share_directory('my_robot_mapping'),
        'behavior_trees',
        'explore_replan_no_recovery.xml')

    return LaunchDescription([
        # 启动 ROS 2 官方前沿探索节点 (Frontier Exploration)
        Node(
            package='explore_lite',
            executable='explore',
            name='explore_node',
            output='screen',
            parameters=[{
                'robot_base_frame': 'base_footprint',
                'costmap_topic': '/map',
                'costmap_updates_topic': '/map_updates',
                
                'visualize': True,
                
                # 每 0.5 秒检查一次；目标执行中仍保持锁定。配合专用
                # 路过点检查器，到点后的下一段通常在 0.05~0.55 秒接续。
                'planner_frequency': 2.0,
                
                # 路径预检会先过滤静态不可达目标；平移或正常转向都算
                # 进展，避免 V5 把起步阶段的 93 度转向误判成卡死。
                'progress_timeout': 15.0,

                # 每累计移动 5 厘米或转动约 8.6 度刷新一次进展计时。
                'progress_distance': 0.05,
                'progress_angle': 0.15,

                # 不可达目标周围 40 厘米作为同一片失败区域，避免反复尝试。
                'blacklist_radius': 0.40,

                # 前沿本身位于未知区。先在已知自由区内搜索一个带车体
                # 余量的落脚点，再调用 compute_path_to_pose 验证可达性。
                # 车体半径 0.175 m，加 75 mm 目标点安全余量；5 cm 栅格下
                # 等于 5 格。不要设成 inflation_radius，膨胀层仍由 Nav2
                # 作为软代价处理，否则会错误放弃实际可通行的窄通道。
                'goal_clearance': 0.25,
                # V9 实测前沿 (-1.49,-1.47) 在 0.80m 搜索圈内没有合格
                # 白区点，但 0.825m 处存在代价19的安全点，因此扩大一格
                # 余量到1.0m；目标代价阈值不变，V10.6再把车体余量
                # 从0.20m提高到0.25m以容纳实测跟踪偏差。
                'staging_search_radius': 1.00,
                'staging_retry_cooldown': 5.0,
                'path_check_timeout': 6.0,

                # /map 中的白色只代表 SLAM 认为可通行；仍需用 Nav2
                # 全局代价地图排除实时障碍物及其高代价膨胀区。实测异常
                # 目标的代价是 26~67，因此只接受 <=20 的落脚点。
                'navigation_costmap_topic': '/global_costmap/costmap',
                'max_staging_cost': 20,
                # 52/67 是膨胀层的软代价，不是致命/内切碰撞栅格。V9
                # 5分钟内因此误拒22次正确规划。允许路径穿过 <=80 的
                # 软代价区，终点仍须 <=20，RPP碰撞检查和安全监控不变。
                'max_path_cost': 80,
                # 40 是舒适区阈值，不是碰撞阈值。机器人若因动态障碍或
                # 跟踪误差停在 41~98 的软膨胀区，V10 会永久等待；V10.3
                # 改为只允许一个短距离、终点代价<=20 的规划脱困步。
                # 99/100 仍是内切/致命硬停止，绝不由探索器强行移动。
                'max_robot_cost': 40,
                'hard_stop_cost': 99,
                # V10.5中心包从代价74发出0.43m脱困步后落入代价99；
                # 缩至0.30m。30cm内没有低代价白点就换分支，不冒险前进。
                'soft_escape_goal_step': 0.30,
                'staging_cost_weight': 0.50,

                # 已经在车旁的落脚点直接视为覆盖，不发送一个会瞬间成功
                # 的重复目标；到达后只留一个短地图刷新窗口。
                'min_goal_distance': 0.25,
                'goal_settle_time': 0.05,

                # V10.6 每次只沿 Nav2 已验证的已知白色路径前进不超过
                # 1.20 m。两次 V10.4 录包中正常目标交接仅 0.2~0.6 秒，
                # 主要顿挫来自每 0.85m 都进入一次 0.35m 末端减速；在
                # 终点<=20、路径<=80、车体余量及安全监控不变的条件下，
                # 增长约41%可减少停车次数，也能跨过0.85m内全是软膨胀
                # 代价、稍远处才重新出现低代价白区的短通道。
                'max_goal_step': 1.20,
                'branch_weight': 0.75,
                'branch_match_radius': 1.00,
                'branch_direction_weight': 0.80,

                # 正常分支只能在当前路径方向前方 70 度锥形内延续；只有
                # DFS 切换到记忆旁支时才允许转向后方。40 度以上的路径
                # 分离记录为不同子树，父分叉仅作为逻辑记忆，不再强制
                # 机器人逐点回放旧路径。
                'branch_forward_angle': 1.22,
                'min_forward_progress': 0.10,
                'junction_angle': 0.70,
                'branch_choice_radius': 0.85,
                'branch_anchor_tolerance': 0.30,
                # 宏观分叉记忆：沿同一分支至少前进1.20m才允许再建立
                # 一层分叉；同一0.90m物理区域最多保留3个出口。恢复旧
                # 分支时，当前白区路径必须经过旧锚点0.75m范围并以不
                # 超过约60度的误差沿旧出口离开，避免把同一UNKNOWN盆地
                # 两侧的边界错误绑定为旧分支。
                'junction_memory_min_travel': 1.20,
                'junction_cluster_radius': 0.90,
                'resume_anchor_match_radius': 0.75,
                'resume_heading_tolerance': 1.05,
                'completed_branch_radius': 0.85,
                # 1.5秒覆盖3次规划周期；分支结束时不再额外静止2.5秒。
                'branch_end_timeout': 1.5,
                'max_branch_stack_size': 12,
                'max_siblings_per_junction': 3,

                # UNKNOWN-basin 是宏观分支判断。以每条前沿为种子，在
                # 1.5m 有界范围内沿灰色未知栅格洪泛，再只保留从可达
                # 白区观察位真正可见的部分；大房间/宽通道优先。
                # 即时目标至少0.50m²、持久DFS分支至少0.75m²，过滤已
                # 探索房间残留的灰色碎角和黑墙外的虚假大盆地。
                # 已完成区域至少新增 0.75m² 未知收益才允许重新打开，避免
                # 人员走动和边界抖动造成来回探索。
                'frontier_size_weight': 0.35,
                'unknown_gain_radius': 1.50,
                # 只统计从可达白色落脚点、穿不过黑色障碍且位于前方
                # 约120度视锥内的灰色未知栅格。墙外连通灰区和墙后灰区
                # 不再把已经完成的小分叉重新抬成高收益目标。
                'unknown_visibility_half_angle': 1.05,
                'unknown_gain_weight': 0.80,
                'unknown_gain_cap': 4.00,
                'min_unknown_gain': 0.50,
                'branch_memory_min_unknown_gain': 0.75,
                'completed_branch_reopen_gain_delta': 0.75,
                'unknown_basin_match_radius': 1.50,
                'unknown_basin_weight': 0.35,
                'frontier_refresh_delay': 1.50,
                'retry_cooldown': 20.0,
                # V10.4一角实测在同一末端左右目标间每17~18秒切换：
                # 无进展后把漂移中的整片0.85m边界冷却60秒，而不是只
                # 冷却一个40cm质心20秒。动态障碍清除后仍会允许复试。
                'stalled_retry_cooldown': 60.0,
                'stalled_region_radius': 0.85,
                # 活跃分支和记忆栈清空后连续 15 秒没有有效候选才完成。
                # 不再被一次空地图误停，也不再在已完成房间额外空等45秒。
                'no_candidate_timeout': 15.0,
                # 若跟踪偏差或SLAM更新使机器人中心持续落到99硬代价，
                # 绝不强行脱困；15秒仍未清除就安全暂停并明确报错，避免
                # 节点继续运行数分钟却不再发目标。
                'hard_cost_wait_timeout': 15.0,

                # 扫地机器人式连续性：同等距离优先选择当前朝向附近的
                # 前沿，减少横跨房间和左右来回掉头。
                # V5 首个 0.35 m 目标需要约 93 度转向，与前方候选几乎
                # 同分；略提高转向代价，优先沿当前朝向启动探索。
                'heading_weight': 0.45,

                # 探索层自己负责换区和黑名单。规划或跟踪失败后立即
                # 返回 explore_lite，不执行默认行为树的多轮清图、原地
                # 旋转、等待和后退；手动 RViz 导航仍使用默认行为树。
                'behavior_tree': explore_behavior_tree,

                # 先按距离稳定输入顺序；最终 V10 评分还会加入路径方向、
                # 父分叉记忆、前沿长度和未知盆地收益，不再是最近点贪心。
                'near_first': True,
                
                # 调大距离权重，优先探索眼前的屋子
                'potential_scale': 5.0,      
                'orientation_scale': 0.0,    
                'gain_scale': 1.0,           
                'transform_tolerance': 0.3,  
                
                # frontier 是白/灰边界的长度而不是未知区面积。0.30 m
                # （地图分辨率 0.05 m 时约 6 格）可滤掉零散噪点，同时不会
                # 错过机器人能够通过的普通门口；安全落脚点与路径预检负责
                # 继续排除椅子缝、墙后等不可达区域。
                'min_frontier_size': 0.30,
            }]
        )
    ])
