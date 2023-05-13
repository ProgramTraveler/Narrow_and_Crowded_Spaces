#ifndef PLANNING_GOAL_POSE_SUBSCRIBER_H
#define PLANNING_GOAL_POSE_SUBSCRIBER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h> // A Pose with reference coordinate frame and timestamp

#include <deque>
#include <mutex>

class GoalPoseSubscriber2D {
public:
    GoalPoseSubscriber2D(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size);

    void ParseData(std::deque<geometry_msgs::PoseStampedPtr> &pose_data_buff);

private:
    void MessageCallBack(const geometry_msgs::PoseStampedPtr &goal_pose_ptr); // 回调函数

private:
    ros::Subscriber subscriber_;
    std::deque<geometry_msgs::PoseStampedPtr> goal_poses_;

    std::mutex buff_mutex_;
};

#endif
