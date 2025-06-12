# Apollo轨迹规划

这个ROS包包含了从Apollo自动驾驶平台（v5.0）中解耦的轨迹规划部分，可以在ROS系统中运行。

已在Ubuntu 18.04和ROS Melodic环境下测试通过。

## 功能

在v1.0版本中实现了带有静态障碍物的格点规划（Lattice Planning）。

## 安装

确保您已经安装了ROS环境（推荐Melodic）。然后将此包克隆到您的工作空间中：

```bash
cd ~/catkin_ws/src
git clone https://github.com/yourusername/apollo-trajectory-planning.git
cd ..
catkin_make
source devel/setup.bash
```

## 使用方法

### 单独运行各个节点

按照以下顺序启动各个组件：

1. 发布道路参考线：
```bash
rosrun apollo_planning PNC_MapPub
```

2. 发布障碍物：
```bash
rosrun apollo_planning PNC_ObsPub
```

3. 运行格点规划：
```bash
rosrun apollo_planning lattice_planner
```

### 使用launch文件一键启动

为方便使用，我们提供了一个launch文件，可以按正确顺序同时启动所有组件：

```bash
roslaunch apollo_planning apollo_trajectory_planning.launch
```

这将按顺序启动以下节点：
1. 道路参考线发布节点 (PNC_MapPub)
2. 障碍物发布节点 (PNC_ObsPub)
3. 格点规划节点 (lattice_planner)
4. RViz可视化工具（使用rviz_PNC.rviz配置文件）

## 注意事项

- 格点规划算法目前仅支持静态障碍物
- 对于横向优化，除了默认求解器外，OSQP求解器也是一个选项（需要Eigen库支持）

## 可视化

使用RViz可以查看：
- 道路参考线 (Map_Pub话题)
- 障碍物 (Obs_Pub话题)
- 规划的轨迹 (Obj_Pub话题)

## 贡献

欢迎提交Pull Request或Issues来改进这个项目。 