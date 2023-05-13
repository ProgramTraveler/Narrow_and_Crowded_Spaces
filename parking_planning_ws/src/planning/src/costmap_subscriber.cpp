#include "planning/costmap_subscriber.h"
/*
    nav_msgs::OccupancyGrid 表示的是一个二维网格地图(栅格地图？) 其中每个单元格表示占用概率
        Header header 

        地图的元数据
        MapMetaData info

        地图数据 按行优先顺序 从(0, 0) 开始
        概率在[0, 100] 范围内 未知是 -1
        int8[] data
*/

CostMapSubscriber::CostMapSubscriber(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size) {
    subscriber_ = nh.subscribe(topic_name, buff_size, &CostMapSubscriber::MessageCallBack, this);
}

void CostMapSubscriber::MessageCallBack(const nav_msgs::OccupancyGridPtr &costmap_msg_ptr) {
    buff_mutex_.lock();
    deque_costmap_.emplace_back(costmap_msg_ptr);
    buff_mutex_.unlock();
}

void CostMapSubscriber::ParseData(std::deque<nav_msgs::OccupancyGridPtr> &deque_costmap_msg_ptr) {
    buff_mutex_.lock();
    if (!deque_costmap_.empty()) {
        deque_costmap_msg_ptr.insert(deque_costmap_msg_ptr.end(),
                                     deque_costmap_.begin(),
                                     deque_costmap_.end());

        deque_costmap_.clear();
    }
    buff_mutex_.unlock();
}