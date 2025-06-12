# ST图数据消息定义
Header header

# 时间范围
float64 t_min
float64 t_max

# 路径范围
float64 s_min
float64 s_max

# 规划起始点的ST坐标
float64 init_s
float64 init_ds
float64 init_dds

# ST障碍物边界点
# 每个障碍物有四个点：左上、右上、左下、右下
# 格式：[obs1_upper_left_s, obs1_upper_left_t, obs1_upper_right_s, obs1_upper_right_t, 
#        obs1_lower_left_s, obs1_lower_left_t, obs1_lower_right_s, obs1_lower_right_t,
#        obs2_upper_left_s, ...]
float64[] st_boundaries_s
float64[] st_boundaries_t

# 每个障碍物的ID
string[] obstacle_ids

# 规划轨迹的ST坐标
float64[] traj_t
float64[] traj_s
float64[] traj_ds
float64[] traj_dds 