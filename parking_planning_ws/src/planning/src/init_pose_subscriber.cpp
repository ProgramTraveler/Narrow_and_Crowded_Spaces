#include "planning/init_pose_subscriber.h"

/*
    geometry_msgs::PoseWithCovarianceStamped 表示一个带有参考坐标系和时间戳的估计姿态

        std_msgs/Header header
        PoseWithCovariance pose
*/

InitPoseSubscriber2D::InitPoseSubscriber2D(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size) {
    subscriber_ = nh.subscribe(topic_name, buff_size, &InitPoseSubscriber2D::MessgeCallBack, this);
}

void InitPoseSubscriber2D::MessgeCallBack(const geometry_msgs::PoseWithCovarianceStampedPtr &init_pose_ptr) {
    buff_mutex_.lock();
    init_poses_.emplace_back(init_pose_ptr);
    buff_mutex_.unlock();
}

void InitPoseSubscriber2D::ParseData(std::deque<geometry_msgs::PoseWithCovarianceStampedPtr> &pose_data_buff) {
    buff_mutex_.lock();

    if (!init_poses_.empty()) {
        // 组合数据
        pose_data_buff.insert(pose_data_buff.end(), init_poses_.begin(), init_poses_.end());

        init_poses_.clear();
    }

    buff_mutex_.unlock();
}