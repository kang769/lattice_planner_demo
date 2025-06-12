#ifndef LATTICE_PLANNER_H
#define LATTICE_PLANNER_H

#include <iostream>
#include <vector>
#include <array>
#include <memory>

#include <ros/ros.h>
#include "apollo_planning/SLGraph.h"
#include "apollo_planning/STGraph.h"

#include "include/reference_line_info.h"
#include "include/frame.h"
#include "include/pnc_point.pb.h"
#include "include/planning_gflags.h"
#include "include/discretized_trajectory.h"

namespace apollo
{
namespace planning
{

// 前向声明
class PathTimeGraph;

class LatticePlanner {
    public:
    LatticePlanner();
    virtual ~LatticePlanner() = default;

    void PlanOnReferenceLine(
      const TrajectoryPoint& planning_init_point, Frame* frame,
      ReferenceLineInfo* reference_line_info);
    
    
/*
    void PlanOnReferenceLine(
      const TrajectoryPoint& planning_init_point, Frame* frame,
      ReferenceLineInfo* reference_line_info) override;
      */

    private:
    std::vector<PathPoint> ToDiscretizedReferenceLine(
        const std::vector<ReferencePoint>& ref_points);

    void ComputeInitFrenetState(const PathPoint& matched_point,
                            const TrajectoryPoint& cartesian_state,
                            std::array<double, 3>* ptr_s,
                            std::array<double, 3>* ptr_d);
    
    // 发布SL图数据
    void PublishSLGraph(const std::array<double, 3>& init_s, 
                        const std::array<double, 3>& init_d,
                        const std::shared_ptr<std::vector<PathPoint>>& ptr_reference_line,
                        const std::shared_ptr<PathTimeGraph>& ptr_path_time_graph,
                        const DiscretizedTrajectory& discretized_trajectory);
    
    // 发布ST图数据
    void PublishSTGraph(const std::array<double, 3>& init_s,
                        const std::shared_ptr<PathTimeGraph>& ptr_path_time_graph,
                        const DiscretizedTrajectory& discretized_trajectory);
    
    // ROS发布器
    ros::Publisher sl_graph_pub_;
    ros::Publisher st_graph_pub_;
    ros::NodeHandle nh_;
};

}   //namespace planning
}   //namespace Apollo

#endif // LATTICE_PLANNER_H