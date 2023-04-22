#include "ros/ros.h"
// 使用第三方库
#include "3rd/backward.hpp"

namespace backward {
    // 发生常见错误时自动打印栈跟踪信息
    backward::SignalHandling sh;
}

int main (int argc, char *argv[]) {
    ros::init(argc, argv, "run_planning");
    
    ros::NodeHandle node_handle("~"); // 私有空间

    ros::Rate rate(10);

    while (ros::ok()) {


        ros::spinOnce();
        rate.sleep();
    }

    ros::shutdown(); // 关闭节点

    return 0;
}