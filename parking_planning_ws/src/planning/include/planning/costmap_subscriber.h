#ifndef PLANNING_COSTMAP_SUBSCRIBER_H
#define PLANNING_COSTMAP_SUBSCRIBER_H

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

#include "deque"
#include "mutex"
#include "thread"
#include "string"

class CostMapSubscriber {
public:
    CostMapSubscriber(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size);

    void PareData(std::deque<nav_msgs::OccupancyGridConstPtr> &deque_cost_msg_ptr);

private:
    void MessageCallBack(const nav_msgs::OccupancyGridConstPtr &costmap_msg_ptr);

private:
    ros::Subscriber subscriber_;
    std::deque<nav_msgs::OccupancyGridConstPtr> deque_costmap_;

    std::mutex buff_mutex_;

};

#endif