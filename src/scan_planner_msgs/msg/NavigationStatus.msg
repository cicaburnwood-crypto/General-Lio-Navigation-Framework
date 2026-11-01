unique_identifier_msgs/UUID task_id
bool output_blocked
bool cleanup_complete
std_msgs/Header header

# FSM states
uint8 FSM_INIT=0 
uint8 FSM_WAIT_TARGET=1
uint8 FSM_GEN_NEW_TRAJ=2
uint8 FSM_REPLAN_TRAJ=3
uint8 FSM_EXEC_TRAJ=4
uint8 FSM_EMERGENCY_STOP=5
 # 上面的是消息中定义的常量，2个作用：1. 说明每个uint状态代表的含义，类似注释。2. 方便代码中使用status.fsm_state = scan_planner_msgs::msg::NavigationStatus::FSM_EXEC_TRAJ; 否则只能status.fsm_state = 4;

uint8 fsm_state   # 实际传输的消息字段
string fsm_state_name   # 实际传输的消息字段

# Navigation modes
uint8 NAV_MANUAL_TARGET=1   # local planner 拿一个simple nav goal作为一次性的测试Goal
uint8 NAV_PRESET_TARGET=2   # local planner用手动打点的一串waypoints作为全局路径依据
uint8 NAV_REFERENCE_PATH=3  # local planner用全局输出的/pct_path作为全局路径依据

uint8 navigation_mode  # 实际传输的消息字段

# Navigation task lifecycle. This is independent from the internal FSM state.
uint8 NAVIGATION_IDLE=0
uint8 NAVIGATION_ACTIVE=1
uint8 NAVIGATION_SUCCEEDED=2
uint8 NAVIGATION_FAILED=3
uint8 NAVIGATION_CANCELED=4  # Managed StopTask after SCAN cleanup; Action result added in PR3b.

uint8 navigation_state
string navigation_state_name

# Navigation input
bool has_odometry  # 节点是否曾经收到过odom，0表示从未收到 odometry，1表示收到过但不确定是否中断
builtin_interfaces/Time last_odometry_time  # 最近一次收到odom是什么时候，建议明确定义为SCAN 节点最后一次执行 odometry 回调的 ROS 时间

bool has_target  # 表示 SCAN 当前是否有一个有效的导航目标。

# Navigation progress
int32 waypoint_index  # 表示当前正在执行第几个 preset waypoint，建议定义为从 0 开始。PRESET_TARGET模式下有效，其他模式-1.
uint32 waypoint_count  # 表示当前 preset waypoint 序列中一共有多少个点。即为当前已加载序列的总点数。其他模式下为0.
float64 distance_to_waypoint  # PRESET_TARGET 模式下，当前位置到原始当前 waypoint 的三维距离；其他模式为 NaN

uint64 replan_attempt_count  # SCAN 节点启动后累计的局部重规划尝试次数，成功和失败都计数
uint32 consecutive_replan_failures  # 当前连续重规划（规划失败）多少次

# SCAN reference-path endpoint after existing body_height lift, before occupancy adjustment.
geometry_msgs/Point original_goal
# Target actually used by SCAN after occupancy adjustment; not the Action goal's Z convention.
geometry_msgs/Point effective_goal

bool goal_adjusted

# Navigation error code
uint16 FAILURE_NONE=0
uint16 FAILURE_INVALID_TARGET=1
uint16 FAILURE_NO_ODOMETRY=2
uint16 FAILURE_GLOBAL_PLAN_FAILED=3
uint16 FAILURE_TARGET_OCCUPIED=4
uint16 FAILURE_LOCAL_REPLAN_FAILED=5
uint16 FAILURE_TRAJECTORY_COLLISION=6
uint16 FAILURE_REPLAN_LIMIT_REACHED=7

uint16 last_failure_code
string last_failure_reason
builtin_interfaces/Time last_failure_time
