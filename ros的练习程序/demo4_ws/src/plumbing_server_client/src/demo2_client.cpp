#include "ros/ros.h"
#include "plumbing_server_client/Addints.h"

/*
    客户端 提交两个整数 并处理响应的结果
        1. 包含头文件
        2. 初始化 ros 节点
        3. 创建节点句柄
        4. 创建一个客户端对象
        5. 提交请求并处理响应
*/

int main(int argc, char *argv[]) {
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "daBao");

    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<plumbing_server_client::Addints>("addInts");

    // 提交请求并处理响应
    plumbing_server_client::Addints ai;

    // 组织请求
    ai.request.num1 = 100;
    ai.request.num2 = 200;

    // 处理响应
    bool flag = client.call(ai);

    if (flag) ROS_INFO("响应成功, 结果 = %d", ai.response.sum);
    else ROS_INFO("响应失败");

    return 0;
}