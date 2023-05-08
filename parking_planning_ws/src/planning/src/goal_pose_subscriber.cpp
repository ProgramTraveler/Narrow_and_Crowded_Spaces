#include "planning/goal_pose_subscriber.h"
/*
    geometry_msgs::PoseStamped 表示一个带有参考坐标系和时间戳的姿态

        std_msgs/Header header
        Pose pose
*/
GoalPoseSubscriber2D::GoalPoseSubscriber2D(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size) {
    subscriber_ = nh.subscribe(topic_name, buff_size, &GoalPoseSubscriber2D::MessageCallBack, this);
}

void GoalPoseSubscriber2D::MessageCallBack(const geometry_msgs::PoseStampedPtr &goal_pose_ptr) {
    buff_mutex_.lock();
    goal_pose_.emplace_back(goal_pose_ptr);
    buff_mutex_.unlock();
}

void GoalPoseSubscriber2D::ParseData(std::deque<geometry_msgs::PoseStampedPtr> &pose_data_buff) {
    buff_mutex_.lock();

    if (!goal_pose_.empty()) {
        pose_data_buff.insert(pose_data_buff.end(), goal_pose_.begin(), goal_pose_.end());
        goal_pose_.clear();
    }

    buff_mutex_.unlock();
}