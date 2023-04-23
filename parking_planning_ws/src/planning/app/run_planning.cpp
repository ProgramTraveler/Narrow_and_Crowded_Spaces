#include "ros/ros.h"
// 使用第三方库
#include "3rd/backward.hpp"
#include "planning/planning_method_flow.h"

namespace backward {
    // 发生常见错误时自动打印栈跟踪信息
    backward::SignalHandling sh;
}

int main (int argc, char *argv[]) {
    ros::init(argc, argv, "run_planning");
    
    ros::NodeHandle node_handle("~"); // 私有空间

    PlanningMethodFlow kinodynamic_flow(node_handle);

    ros::Rate rate(10);

    while (ros::ok()) {
        kinodynamic_flow.Run();

        ros::spinOnce();
        rate.sleep();
    }

    ros::shutdown(); // 关闭节点

    return 0;
}