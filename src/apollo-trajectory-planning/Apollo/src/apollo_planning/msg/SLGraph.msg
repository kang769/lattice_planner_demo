# SL图数据消息定义
Header header

# 参考线的S坐标
float64[] ref_s
# 参考线的L坐标
float64[] ref_l

# 规划起始点的SL坐标
float64 init_s
float64 init_l
float64 init_ds
float64 init_dl
float64 init_dds
float64 init_ddl

# 障碍物的SL边界
float64[] obs_s_lower  # 障碍物S方向下边界
float64[] obs_s_upper  # 障碍物S方向上边界
float64[] obs_l_lower  # 障碍物L方向下边界
float64[] obs_l_upper  # 障碍物L方向上边界

# 横向边界
float64[] s_samples    # 采样的S坐标点
float64[] l_lower      # L方向下边界
float64[] l_upper      # L方向上边界

# 规划轨迹的SL坐标
float64[] traj_s
float64[] traj_l
float64[] traj_ds
float64[] traj_dl
float64[] traj_dds
float64[] traj_ddl 