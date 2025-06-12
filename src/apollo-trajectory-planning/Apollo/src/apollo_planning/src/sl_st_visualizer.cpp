#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "apollo_planning/SLGraph.h"
#include "apollo_planning/STGraph.h"

class SLSTVisualizer {
public:
    SLSTVisualizer() {
        // 初始化ROS节点
        ros::NodeHandle nh;
        
        // 订阅SL和ST图数据
        sl_graph_sub_ = nh.subscribe("planning/sl_graph", 1, &SLSTVisualizer::slGraphCallback, this);
        st_graph_sub_ = nh.subscribe("planning/st_graph", 1, &SLSTVisualizer::stGraphCallback, this);
        
        // 发布可视化标记
        sl_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("visualization/sl_graph", 1);
        st_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("visualization/st_graph", 1);
    }

    // SL图回调函数
    void slGraphCallback(const apollo_planning::SLGraph::ConstPtr& msg) {
        visualization_msgs::MarkerArray marker_array;
        
        // 1. 可视化参考线
        visualization_msgs::Marker ref_line;
        ref_line.header = msg->header;
        ref_line.ns = "reference_line";
        ref_line.id = 0;
        ref_line.type = visualization_msgs::Marker::LINE_STRIP;
        ref_line.action = visualization_msgs::Marker::ADD;
        ref_line.pose.orientation.w = 1.0;
        ref_line.scale.x = 0.1;  // 线宽
        ref_line.color.r = 1.0;
        ref_line.color.g = 1.0;
        ref_line.color.a = 1.0;
        
        for (size_t i = 0; i < msg->ref_s.size(); ++i) {
            geometry_msgs::Point p;
            p.x = msg->ref_s[i];
            p.y = msg->ref_l[i];
            p.z = 5.0;  // SL图在z=5平面上显示
            ref_line.points.push_back(p);
        }
        marker_array.markers.push_back(ref_line);
        
        // 2. 可视化初始点
        visualization_msgs::Marker init_point;
        init_point.header = msg->header;
        init_point.ns = "init_point";
        init_point.id = 1;
        init_point.type = visualization_msgs::Marker::SPHERE;
        init_point.action = visualization_msgs::Marker::ADD;
        init_point.pose.position.x = msg->init_s;
        init_point.pose.position.y = msg->init_l;
        init_point.pose.position.z = 5.0;
        init_point.pose.orientation.w = 1.0;
        init_point.scale.x = 0.3;
        init_point.scale.y = 0.3;
        init_point.scale.z = 0.3;
        init_point.color.g = 1.0;
        init_point.color.a = 1.0;
        marker_array.markers.push_back(init_point);
        
        // 3. 可视化横向边界
        visualization_msgs::Marker l_bounds;
        l_bounds.header = msg->header;
        l_bounds.ns = "lateral_bounds";
        l_bounds.id = 2;
        l_bounds.type = visualization_msgs::Marker::LINE_LIST;
        l_bounds.action = visualization_msgs::Marker::ADD;
        l_bounds.pose.orientation.w = 1.0;
        l_bounds.scale.x = 0.05;
        l_bounds.color.b = 1.0;
        l_bounds.color.a = 0.5;
        
        for (size_t i = 0; i < msg->s_samples.size(); ++i) {
            // 下边界点
            geometry_msgs::Point p_lower;
            p_lower.x = msg->s_samples[i];
            p_lower.y = msg->l_lower[i];
            p_lower.z = 5.0;
            
            // 上边界点
            geometry_msgs::Point p_upper;
            p_upper.x = msg->s_samples[i];
            p_upper.y = msg->l_upper[i];
            p_upper.z = 5.0;
            
            // 连接下边界点和上边界点
            l_bounds.points.push_back(p_lower);
            l_bounds.points.push_back(p_upper);
        }
        marker_array.markers.push_back(l_bounds);
        
        // 4. 可视化规划轨迹
        visualization_msgs::Marker traj;
        traj.header = msg->header;
        traj.ns = "trajectory";
        traj.id = 3;
        traj.type = visualization_msgs::Marker::LINE_STRIP;
        traj.action = visualization_msgs::Marker::ADD;
        traj.pose.orientation.w = 1.0;
        traj.scale.x = 0.1;
        traj.color.r = 1.0;
        traj.color.g = 0.0;
        traj.color.b = 0.0;
        traj.color.a = 1.0;
        
        for (size_t i = 0; i < msg->traj_s.size(); ++i) {
            geometry_msgs::Point p;
            p.x = msg->traj_s[i];
            p.y = msg->traj_l[i];
            p.z = 5.0;
            traj.points.push_back(p);
        }
        marker_array.markers.push_back(traj);
        
        // 发布标记数组
        sl_marker_pub_.publish(marker_array);
    }
    
    // ST图回调函数
    void stGraphCallback(const apollo_planning::STGraph::ConstPtr& msg) {
        visualization_msgs::MarkerArray marker_array;
        
        // 1. 可视化ST图坐标系
        visualization_msgs::Marker axes;
        axes.header = msg->header;
        axes.ns = "st_axes";
        axes.id = 0;
        axes.type = visualization_msgs::Marker::LINE_LIST;
        axes.action = visualization_msgs::Marker::ADD;
        axes.pose.orientation.w = 1.0;
        axes.scale.x = 0.05;
        axes.color.r = 0.5;
        axes.color.g = 0.5;
        axes.color.b = 0.5;
        axes.color.a = 0.5;
        
        // S轴
        geometry_msgs::Point s_start, s_end;
        s_start.x = msg->s_min;
        s_start.y = 0;
        s_start.z = 10.0;  // ST图在z=10平面上显示
        s_end.x = msg->s_max;
        s_end.y = 0;
        s_end.z = 10.0;
        axes.points.push_back(s_start);
        axes.points.push_back(s_end);
        
        // T轴
        geometry_msgs::Point t_start, t_end;
        t_start.x = msg->s_min;
        t_start.y = 0;
        t_start.z = 10.0;
        t_end.x = msg->s_min;
        t_end.y = msg->t_max;
        t_end.z = 10.0;
        axes.points.push_back(t_start);
        axes.points.push_back(t_end);
        
        marker_array.markers.push_back(axes);
        
        // 2. 可视化ST障碍物边界
        visualization_msgs::Marker st_boundaries;
        st_boundaries.header = msg->header;
        st_boundaries.ns = "st_boundaries";
        st_boundaries.id = 1;
        st_boundaries.type = visualization_msgs::Marker::LINE_LIST;
        st_boundaries.action = visualization_msgs::Marker::ADD;
        st_boundaries.pose.orientation.w = 1.0;
        st_boundaries.scale.x = 0.05;
        st_boundaries.color.r = 1.0;
        st_boundaries.color.a = 0.8;
        
        // 每个障碍物有四个点，形成一个多边形
        size_t num_obstacles = msg->st_boundaries_s.size() / 4;
        for (size_t i = 0; i < num_obstacles; ++i) {
            size_t base_idx = i * 4;
            
            // 上边界线
            geometry_msgs::Point p1, p2;
            p1.x = msg->st_boundaries_s[base_idx];     // 左上
            p1.y = msg->st_boundaries_t[base_idx];
            p1.z = 10.0;
            p2.x = msg->st_boundaries_s[base_idx + 1]; // 右上
            p2.y = msg->st_boundaries_t[base_idx + 1];
            p2.z = 10.0;
            st_boundaries.points.push_back(p1);
            st_boundaries.points.push_back(p2);
            
            // 右边界线
            geometry_msgs::Point p3;
            p3.x = msg->st_boundaries_s[base_idx + 3]; // 右下
            p3.y = msg->st_boundaries_t[base_idx + 3];
            p3.z = 10.0;
            st_boundaries.points.push_back(p2);
            st_boundaries.points.push_back(p3);
            
            // 下边界线
            geometry_msgs::Point p4;
            p4.x = msg->st_boundaries_s[base_idx + 2]; // 左下
            p4.y = msg->st_boundaries_t[base_idx + 2];
            p4.z = 10.0;
            st_boundaries.points.push_back(p3);
            st_boundaries.points.push_back(p4);
            
            // 左边界线
            st_boundaries.points.push_back(p4);
            st_boundaries.points.push_back(p1);
        }
        marker_array.markers.push_back(st_boundaries);
        
        // 3. 可视化规划轨迹
        visualization_msgs::Marker traj;
        traj.header = msg->header;
        traj.ns = "st_trajectory";
        traj.id = 2;
        traj.type = visualization_msgs::Marker::LINE_STRIP;
        traj.action = visualization_msgs::Marker::ADD;
        traj.pose.orientation.w = 1.0;
        traj.scale.x = 0.1;
        traj.color.r = 0.0;
        traj.color.g = 1.0;
        traj.color.b = 0.0;
        traj.color.a = 1.0;
        
        for (size_t i = 0; i < msg->traj_t.size(); ++i) {
            geometry_msgs::Point p;
            p.x = msg->traj_s[i];
            p.y = msg->traj_t[i];
            p.z = 10.0;
            traj.points.push_back(p);
        }
        marker_array.markers.push_back(traj);
        
        // 发布标记数组
        st_marker_pub_.publish(marker_array);
    }

private:
    ros::Subscriber sl_graph_sub_;
    ros::Subscriber st_graph_sub_;
    ros::Publisher sl_marker_pub_;
    ros::Publisher st_marker_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "sl_st_visualizer");
    
    SLSTVisualizer visualizer;
    
    ros::spin();
    
    return 0;
} 