#include "planning/planning_method_flow.h"

#include "nav_msgs/Path.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_broadcaster.h"


PlanningMethodFlow::PlanningMethodFlow(ros::NodeHandle &nh) { // 初始化
    // 创建一个 InitPoseSubscriber2D 对象(是一个智能指针类型) 
    // 传入的三个参数分别为 ros::NodeHandle 对象 话题名称 队列大小
    init_pose_sub_ptr_ = std::make_shared<InitPoseSubscriber2D>(nh, "/initialpose", 1);

    goal_pose_sub_ptr_ = std::make_shared<GoalPoseSubscriber2D>(nh, "/move_base_simple/goal", 1);
}

void PlanningMethodFlow::Run() {
    ReadData();
    
    while (HasStartPose() && HasGoalPose()) { // 初始位姿和目标位姿队列不为空
        // 获取当前位姿信息
        InitPoseData();

        // 从四元数中提取偏航角
        double start_yaw = tf::getYaw(current_init_pose_ptr_ -> pose.pose.orientation);
        double goal_yaw = tf::getYaw(current_goal_pose_ptr_ -> pose.orientation);

        Vec3d start_state = Vec3d(
            current_init_pose_ptr_ -> pose.pose.position.x,
            current_init_pose_ptr_ -> pose.pose.position.y,
            start_yaw
        );

        Vec3d goal_state = Vec3d(
            current_goal_pose_ptr_ -> pose.position.x,
            current_goal_pose_ptr_ -> pose.position.y,
            goal_yaw
        );
    }
}

void PlanningMethodFlow::ReadData() {
    
    init_pose_sub_ptr_ -> ParseData(init_pose_deque_);
    goal_pose_sub_ptr_ -> ParseData(goal_pose_deque_);
}

void PlanningMethodFlow::InitPoseData() {
    current_init_pose_ptr_ = init_pose_deque_.front();
    init_pose_deque_.pop_front();

    current_goal_pose_ptr_ = goal_pose_deque_.front();
    goal_pose_deque_.pop_front();
}

bool PlanningMethodFlow::HasStartPose() {
    return !init_pose_deque_.empty();
}

bool PlanningMethodFlow::HasGoalPose() {
    return !goal_pose_deque_.empty();
}