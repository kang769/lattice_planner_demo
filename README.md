# Lattice Planner Demo

这是一个基于https://github.com/yufan25/apollo-trajectory-planning.git 仓库的改进仓库，增加了SL图和ST图的插件实现

## 项目结构

- `src/apollo-trajectory-planning/`: Apollo轨迹规划相关代码
- `src/apollo_rviz_plugins/`: Apollo RViz可视化插件

## 安装与使用

### 前提条件

- ROS (Robot Operating System)
- C++11或更高版本
- CMake 3.0.2或更高版本

### 编译

```bash
# 克隆仓库
git clone https://github.com/kang769/lattice_planner_demo.git

# 进入工作空间
cd lattice_planner_demo

# 编译
catkin_make
```

### 运行

```bash
# 设置环境变量
source devel/setup.bash

# 运行演示
roslaunch apollo_trajectory_planning demo.launch
```

![ST_graph](/home/jjk/图片/ST_graph.png)

![SL_graph](/home/jjk/图片/SL_graph.png)

## 许可证

MIT License 