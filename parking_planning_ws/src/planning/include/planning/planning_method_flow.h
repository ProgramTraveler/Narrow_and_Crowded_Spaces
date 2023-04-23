#ifndef PLANNING_PLANNING_METHOD_FLOW_H
#define PLANNING_PLANNING_METHOD_FLOW_H

#include "ros/ros.h"

#include "planning/planning_method.h"
#include "planning/costmap_subscriber.h"
#include "planning/init_pose_subscriber.h"
#include "planning/goal_pose_subscriber.h"
#include "planning/type.h"

#include "nav_msgs/OccupancyGrid.h"

class PlanningMethodFlow {

public:
    PlanningMethodFlow() = default; // 生成默认的构造函数

    explicit PlanningMethodFlow(ros::NodeHandle &nh); // 禁止隐式调用

    void Run();

private:
    void InitPoseData();
    void ReadData();

    bool HasStartPose();
    bool HasGoalPose();

    void PublishPath(const VectorVec3d &path);

    void PublishSearchedTree(const VectorVec4d &searched_tree);

    void PublishVechicle(const VectorVec3d &path, double width, double length, unsigned int vehicle_interval);

private:
    std::shared_ptr<InitPoseSubscriber2D> init_pose_sub_ptr_;
    std::shared_ptr<GoalPoseSubscriber2D> goal_pose_sub_ptr_;
    std::shared_ptr<PlanningMethod> kinodynamic_searcher_ptr_;
    std::shared_ptr<CostMapSubscriber> costmap_sub_ptr_;

    ros::Publisher path_pub_;
    ros::Publisher searched_tree_pub_;
    ros::Publisher vehicle_path_pub_;
 
    std::deque<geometry_msgs::PoseWithCovarianceStampedPtr> init_pose_deque_;
    std::deque<geometry_msgs::PoseStampedPtr> goal_pose_deque_;
    /* 
        需要在 OccupancyGrid.h 中配置
            typedef boost::shared_ptr< ::nav_msgs::OccupancyGrid > OccupancyGridPtr;
            typedef boost::shared_ptr< ::nav_msgs::OccupancyGrid const> OccupancyGridConstPtr;
    */
    std::deque<nav_msgs::OccupancyGridPtr> costmap_deque_;

    geometry_msgs::PoseWithCovarianceStampedPtr current_init_pose_ptr_;
    geometry_msgs::PoseStampedPtr current_goal_pose_ptr_;
    nav_msgs::OccupancyGridPtr current_costmap_ptr_;

    ros::Time timestamp_;

    bool has_map_{};
};

#endif