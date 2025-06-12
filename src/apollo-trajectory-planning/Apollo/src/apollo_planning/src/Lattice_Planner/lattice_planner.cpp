#include<iostream>
#include <ctime>
#include <math.h>
#include "lattice_planner.h"
#include "include/path_matcher.h"
#include "include/cartesian_frenet_conversion.h"
#include "include/path_time_graph.h"
#include "include/prediction_querier.h"
#include "include/trajectory1d_generator.h"
#include "include/trajectory_evaluator.h"
#include "include/trajectory_combiner.h"
#include "include/collision_checker.h"
#include "include/constraint_checker.h"

using namespace std;
using namespace apollo;
using namespace apollo::common::math;

namespace apollo
{
namespace planning
{

LatticePlanner::LatticePlanner() {
    // 初始化ROS发布器
    sl_graph_pub_ = nh_.advertise<apollo_planning::SLGraph>("planning/sl_graph", 1);
    st_graph_pub_ = nh_.advertise<apollo_planning::STGraph>("planning/st_graph", 1);
}

void LatticePlanner::PlanOnReferenceLine(
        const TrajectoryPoint& planning_init_point, Frame* frame,
        ReferenceLineInfo* reference_line_info)
{
    static size_t num_planning_cycles = 0;
    static size_t num_planning_succeeded_cycles = 0;

    clock_t start_time = clock();
    clock_t current_time = start_time;

    ++num_planning_cycles;

    reference_line_info->set_is_on_reference_line();

    //-------------------------------------------------------------------
    //1. obtain a reference line and transform it to the PathPoint format.
    auto ptr_reference_line = std::make_shared<std::vector<PathPoint>>(ToDiscretizedReferenceLine(
        reference_line_info->reference_line().reference_points()));
    vector<PathPoint> ptr = *ptr_reference_line;

    //-----------------------------------------------------------------
    //2. compute the matched point of the init planning point on the reference
    // line.
    PathPoint matched_point = PathMatcher::MatchToPath(
      *ptr_reference_line, planning_init_point.path_point.x,
      planning_init_point.path_point.y);

    //-----------------------------------------------------------------
    // 3. according to the matched point, compute the init state in Frenet frame.
    std::array<double, 3> init_s;
    std::array<double, 3> init_d;
    ComputeInitFrenetState(matched_point, planning_init_point, &init_s, &init_d);

    current_time = clock();
    
    //-----------------------------------------------------------------
    // 4. parse the decision and get the planning target.

    //4.1将frame中障碍物的信息传递给ptr_prediction_querier
    auto ptr_prediction_querier = std::make_shared<PredictionQuerier>(
    frame->obstacles(), ptr_reference_line);

    //4.2 将障碍物信息，参考线，规划起始点，目标规划的路径以及时间长度传递给ptr_path_time_graph
    auto ptr_path_time_graph = std::make_shared<PathTimeGraph>(
        ptr_prediction_querier->GetObstacles(), *ptr_reference_line,
        reference_line_info, init_s[0],
        init_s[0] + FLAGS_speed_lon_decision_horizon, 0.0,
        FLAGS_trajectory_time_length, init_d);

    //4.3设置目标车速
    double speed_limit = 5; //限速
    reference_line_info->SetCruiseSpeed(speed_limit);

    //4.4将规划的目标传递给planning_target，包括目标车速，停车指示牌
    PlanningTarget planning_target = reference_line_info->planning_target();

    current_time = clock();

    //-----------------------------------------------------------------
    // 5. generate 1d trajectory bundle for longitudinal and lateral respectively.
    Trajectory1dGenerator trajectory1d_generator(
        init_s, init_d, ptr_path_time_graph, ptr_prediction_querier);
    std::vector<std::shared_ptr<Curve1d>> lon_trajectory1d_bundle;
    std::vector<std::shared_ptr<Curve1d>> lat_trajectory1d_bundle;
    trajectory1d_generator.GenerateTrajectoryBundles(
        planning_target, &lon_trajectory1d_bundle, &lat_trajectory1d_bundle);

    current_time = clock();

    //-----------------------------------------------------------------
    // 6. first, evaluate the feasibility of the 1d trajectories according to
    // dynamic constraints.
    //   second, evaluate the feasible longitudinal and lateral trajectory pairs
    //   and sort them according to the cost.
    TrajectoryEvaluator trajectory_evaluator(
        init_s, planning_target, lon_trajectory1d_bundle, lat_trajectory1d_bundle,
        ptr_path_time_graph, ptr_reference_line);
    
    auto fir = trajectory_evaluator.next_top_trajectory_pair(); 
    auto lon = fir.first;
    auto lat = fir.second;

    int i=0;

    // Get instance of collision checker and constraint checker
    CollisionChecker collision_checker(frame->obstacles(), init_s[0], init_d[0],
                                      *ptr_reference_line, reference_line_info,
                                      ptr_path_time_graph);

    // 7. always get the best pair of trajectories to combine; return the first
    // collision-free trajectory.
    size_t collision_failure_count = 0;
    size_t combined_constraint_failure_count = 0;

    size_t lon_vel_failure_count = 0;
    size_t lon_acc_failure_count = 0;
    size_t lon_jerk_failure_count = 0;
    size_t curvature_failure_count = 0;
    size_t lat_acc_failure_count = 0;
    size_t lat_jerk_failure_count = 0;

    size_t num_lattice_traj = 0;

    while (trajectory_evaluator.has_more_trajectory_pairs()) {
      double trajectory_pair_cost =
          trajectory_evaluator.top_trajectory_pair_cost();
      auto trajectory_pair = trajectory_evaluator.next_top_trajectory_pair();

      //!!! combine two 1d trajectories to one 2d trajectory
      auto combined_trajectory = TrajectoryCombiner::Combine(
          *ptr_reference_line, *trajectory_pair.first, *trajectory_pair.second,
          planning_init_point.relative_time);

      // check longitudinal and lateral acceleration
      // considering trajectory curvatures
      auto result = ConstraintChecker::ValidTrajectory(combined_trajectory);
      if (result != ConstraintChecker::Result::VALID) {
        ++combined_constraint_failure_count;

        switch (result) {
          case ConstraintChecker::Result::LON_VELOCITY_OUT_OF_BOUND:
            lon_vel_failure_count += 1;
            break;
          case ConstraintChecker::Result::LON_ACCELERATION_OUT_OF_BOUND:
            lon_acc_failure_count += 1;
            break;
          case ConstraintChecker::Result::LON_JERK_OUT_OF_BOUND:
            lon_jerk_failure_count += 1;
            break;
          case ConstraintChecker::Result::CURVATURE_OUT_OF_BOUND:
            curvature_failure_count += 1;
            break;
          case ConstraintChecker::Result::LAT_ACCELERATION_OUT_OF_BOUND:
            lat_acc_failure_count += 1;
            break;
          case ConstraintChecker::Result::LAT_JERK_OUT_OF_BOUND:
            lat_jerk_failure_count += 1;
            break;
          case ConstraintChecker::Result::VALID:
          default:
            // Intentional empty
            break;
        }
        continue;
      }

      // check collision with other obstacles
      if (collision_checker.InCollision(combined_trajectory)) {
        ++collision_failure_count;
        continue;
      }

      // put combine trajectory into debug data
      const auto& combined_trajectory_points = combined_trajectory;
      num_lattice_traj += 1;
      reference_line_info->SetTrajectory(combined_trajectory);
      reference_line_info->SetCost(reference_line_info->PriorityCost() +
                                  trajectory_pair_cost);
      reference_line_info->SetDrivable(true);

      // 发布SL和ST图数据
      PublishSLGraph(init_s, init_d, ptr_reference_line, ptr_path_time_graph, combined_trajectory);
      PublishSTGraph(init_s, ptr_path_time_graph, combined_trajectory);

      break;
    }

    if (num_lattice_traj > 0) {
        ++num_planning_succeeded_cycles;
        // std::cout<<"成功规划"<<num_planning_succeeded_cycles<<"次，总共尝试"<<num_planning_cycles<<"次"<<std::endl;
    }

    return;
}

// 发布SL图数据
void LatticePlanner::PublishSLGraph(const std::array<double, 3>& init_s, 
                                   const std::array<double, 3>& init_d,
                                   const std::shared_ptr<std::vector<PathPoint>>& ptr_reference_line,
                                   const std::shared_ptr<PathTimeGraph>& ptr_path_time_graph,
                                   const DiscretizedTrajectory& discretized_trajectory) {
    apollo_planning::SLGraph sl_graph_msg;
    sl_graph_msg.header.stamp = ros::Time::now();
    sl_graph_msg.header.frame_id = "my_frame";
    
    // 设置初始状态
    sl_graph_msg.init_s = init_s[0];
    sl_graph_msg.init_ds = init_s[1];
    sl_graph_msg.init_dds = init_s[2];
    sl_graph_msg.init_l = init_d[0];
    sl_graph_msg.init_dl = init_d[1];
    sl_graph_msg.init_ddl = init_d[2];
    
    // 设置参考线
    for (const auto& point : *ptr_reference_line) {
        sl_graph_msg.ref_s.push_back(point.s);
        sl_graph_msg.ref_l.push_back(0.0); // 参考线的L坐标为0
    }
    
    // 设置横向边界
    double s_start = init_s[0];
    double s_end = init_s[0] + FLAGS_speed_lon_decision_horizon;
    double s_resolution = 0.5;
    auto lateral_bounds = ptr_path_time_graph->GetLateralBounds(s_start, s_end, s_resolution);
    
    for (size_t i = 0; i < lateral_bounds.size(); ++i) {
        sl_graph_msg.s_samples.push_back(s_start + i * s_resolution);
        sl_graph_msg.l_lower.push_back(lateral_bounds[i].first);
        sl_graph_msg.l_upper.push_back(lateral_bounds[i].second);
    }
    
    // 设置规划轨迹
    for (const auto& trajectory_point : discretized_trajectory) {
        const auto& path_point = trajectory_point.path_point;
        sl_graph_msg.traj_s.push_back(path_point.s);
        
        // 计算frenet坐标系下的l值（这里简化处理，实际应该使用cartesian_to_frenet转换）
        double l = 0.0; // 简化处理，实际应该计算
        sl_graph_msg.traj_l.push_back(l);
        
        // 速度和加速度投影到frenet坐标系
        double ds = trajectory_point.v * cos(path_point.theta);
        double dl = trajectory_point.v * sin(path_point.theta);
        double dds = trajectory_point.a * cos(path_point.theta);
        double ddl = trajectory_point.a * sin(path_point.theta);
        
        sl_graph_msg.traj_ds.push_back(ds);
        sl_graph_msg.traj_dl.push_back(dl);
        sl_graph_msg.traj_dds.push_back(dds);
        sl_graph_msg.traj_ddl.push_back(ddl);
    }
    
    // 发布消息
    sl_graph_pub_.publish(sl_graph_msg);
}

// 发布ST图数据
void LatticePlanner::PublishSTGraph(const std::array<double, 3>& init_s,
                                   const std::shared_ptr<PathTimeGraph>& ptr_path_time_graph,
                                   const DiscretizedTrajectory& discretized_trajectory) {
    apollo_planning::STGraph st_graph_msg;
    st_graph_msg.header.stamp = ros::Time::now();
    st_graph_msg.header.frame_id = "my_frame";
    
    // 设置时间和路径范围
    st_graph_msg.t_min = 0.0;
    st_graph_msg.t_max = FLAGS_trajectory_time_length;
    st_graph_msg.s_min = init_s[0];
    st_graph_msg.s_max = init_s[0] + FLAGS_speed_lon_decision_horizon;
    
    // 设置初始状态
    st_graph_msg.init_s = init_s[0];
    st_graph_msg.init_ds = init_s[1];
    st_graph_msg.init_dds = init_s[2];
    
    // 设置ST障碍物边界
    const auto& st_obstacles = ptr_path_time_graph->GetPathTimeObstacles();
    for (const auto& st_obstacle : st_obstacles) {
        // 上边界点
        st_graph_msg.st_boundaries_s.push_back(st_obstacle.upper_left_point().s());
        st_graph_msg.st_boundaries_t.push_back(st_obstacle.upper_left_point().t());
        
        st_graph_msg.st_boundaries_s.push_back(st_obstacle.upper_right_point().s());
        st_graph_msg.st_boundaries_t.push_back(st_obstacle.upper_right_point().t());
        
        // 下边界点
        st_graph_msg.st_boundaries_s.push_back(st_obstacle.bottom_left_point().s());
        st_graph_msg.st_boundaries_t.push_back(st_obstacle.bottom_left_point().t());
        
        st_graph_msg.st_boundaries_s.push_back(st_obstacle.bottom_right_point().s());
        st_graph_msg.st_boundaries_t.push_back(st_obstacle.bottom_right_point().t());
        
        // 障碍物ID
        st_graph_msg.obstacle_ids.push_back(st_obstacle.id());
    }
    
    // 设置规划轨迹
    for (const auto& trajectory_point : discretized_trajectory) {
        st_graph_msg.traj_t.push_back(trajectory_point.relative_time);
        st_graph_msg.traj_s.push_back(trajectory_point.path_point.s);
        st_graph_msg.traj_ds.push_back(trajectory_point.v);
        st_graph_msg.traj_dds.push_back(trajectory_point.a);
    }
    
    // 发布消息
    st_graph_pub_.publish(st_graph_msg);
}

std::vector<PathPoint> LatticePlanner::ToDiscretizedReferenceLine(
    const std::vector<ReferencePoint>& ref_points) {
  double s = 0.0;
  std::vector<PathPoint> path_points;
  for (const auto& ref_point : ref_points) {
    PathPoint path_point;
    path_point.x = ref_point.x();
    path_point.y = ref_point.y();
    path_point.theta = ref_point.heading();
    path_point.kappa = ref_point.kappa();
    path_point.dkappa = ref_point.dkappa();

    if (!path_points.empty()) {
      double dx = path_point.x - path_points.back().x;
      double dy = path_point.y - path_points.back().y;
      s += sqrt(dx * dx + dy * dy);
    }
    path_point.s = s;
    path_points.push_back(std::move(path_point));
  }
  return path_points;
}

void LatticePlanner::ComputeInitFrenetState(const PathPoint& matched_point,
                            const TrajectoryPoint& cartesian_state,
                            std::array<double, 3>* ptr_s,
                            std::array<double, 3>* ptr_d) {
  CartesianFrenetConverter::cartesian_to_frenet(
      matched_point.s, matched_point.x, matched_point.y,
      matched_point.theta, matched_point.kappa, matched_point.dkappa,
      cartesian_state.path_point.x, cartesian_state.path_point.y,
      cartesian_state.v, cartesian_state.a,
      cartesian_state.path_point.theta, cartesian_state.path_point.kappa,
      ptr_s, ptr_d);
}

}   //namespace planning
}   //namespace apollo

