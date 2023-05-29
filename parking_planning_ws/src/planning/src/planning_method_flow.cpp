#include "planning/planning_method_flow.h"

#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

double Mod2Pi(const double &x) {
    double v = fmod(x, 2 * M_PI);

    if (v < -M_PI) {
        v += 2.0 * M_PI;
    } else if (v > M_PI) {
        v -= 2.0 * M_PI;
    }

    return v;
}

PlanningMethodFlow::PlanningMethodFlow(ros::NodeHandle &nh) { // 初始化
    double steering_angle = nh.param("planner/steering_angle", 10); // 转向角
    int steering_angle_discrete_num = nh.param("planner/steering_angle_discrete_num", 1); // 转向角离散数

    double wheel_base = nh.param("planner/wheel_base", 1.0); // 轮距

    double segment_length = nh.param("planner/segment_length", 1.6); // 段长
    int segment_length_discrete_num = nh.param("planner/segment_length_discrete_num", 8); // 段长离散数

    double steering_penalty = nh.param("planner/steering_penalty", 1.05);
    double steering_change_penalty = nh.param("planner/steering_change_penalty", 1.5); // 转向变化惩罚

    double reversing_penalty = nh.param("planner/reversing_penalty", 2.0); // 倒车惩罚
    double shot_distance = nh.param("planner/shot_distance", 5.0);

    kinodynamic_searcher_ptr_ = std::make_shared<PlanningMethod>(
            steering_angle, steering_angle_discrete_num, segment_length, segment_length_discrete_num, wheel_base,
            steering_penalty, reversing_penalty, steering_change_penalty, shot_distance);

    // 创建一个 InitPoseSubscriber2D 对象(是一个智能指针类型) 
    // 传入的三个参数分别为 ros::NodeHandle 对象 话题名称 队列大小
    init_pose_sub_ptr_ = std::make_shared<InitPoseSubscriber2D>(nh, "/initialpose", 1);
    goal_pose_sub_ptr_ = std::make_shared<GoalPoseSubscriber2D>(nh, "/move_base_simple/goal", 1);

    costmap_sub_ptr_ = std::make_shared<CostMapSubscriber>(nh, "/map", 1);

    path_pub_ = nh.advertise<nav_msgs::Path>("searched_path", 1);
    searched_tree_pub_ = nh.advertise<visualization_msgs::Marker>("searched_tree", 1);
    vehicle_path_pub_ = nh.advertise<visualization_msgs::MarkerArray>("vehicle_path", 1);

    has_map_ = false;
}

void PlanningMethodFlow::Run() {
    ReadData();

    if (!has_map_) { // 读入地图信息
        if (costmap_deque_.empty()) {
            return;
        }

        current_costmap_ptr_ = costmap_deque_.front();
        costmap_deque_.pop_front();

        const double map_resolution = 0.2; // 生成栅格地图的大小

        kinodynamic_searcher_ptr_ -> Init(
                /*
                    info.origin.position.x
                        表示地图原点在实际世界中的位置 这是地图中单元格(0, 0)的实际世界姿态
                */
                current_costmap_ptr_ -> info.origin.position.x, // x_lower [m]
                1.0 * current_costmap_ptr_ -> info.width * current_costmap_ptr_ -> info.resolution, // x_upper [m]
                current_costmap_ptr_ -> info.origin.position.y, // y_lower [m]
                1.0 * current_costmap_ptr_ -> info.height * current_costmap_ptr_ -> info.resolution, // y_upper [m]
                current_costmap_ptr_ -> info.resolution, // state_grid_resolution [m / cell]
                map_resolution // map_grid_resolution
        );

        unsigned int map_w = std::floor(current_costmap_ptr_ -> info.width / map_resolution); 
        unsigned int map_h = std::floor(current_costmap_ptr_ -> info.height / map_resolution);
        
        for (unsigned int w = 0; w < map_w; ++ w) {
            for (unsigned int h = 0; h < map_h; ++ h) {
                // x -> [0, current_costmap_ptr -> width]
                auto x = static_cast<unsigned int> ((w + 0.5) * map_resolution
                                                    / current_costmap_ptr_ -> info.resolution);
                // y -> [0, current_costmap_ptr -> heitght]
                auto y = static_cast<unsigned int> ((h + 0.5) * map_resolution
                                                    / current_costmap_ptr_ -> info.resolution);

                /*
                    nav_msgs/OccupancyGrid 是 ROS(机器人操作系统)中的一种消息类型，用于表示占用栅格地图
                        它包含了一个 data 字段 用于存储地图中每个单元格的占用状态

                    data 字段是一个一维整数数组 其长度等于地图的宽度乘以高度
                        数组中的每个元素都表示地图中对应单元格的占用状态
                            值为 0 表示该单元格是空闲的 值为 100 表示该单元格被占用 值为 -1 表示该单元格的状态未知

                    数组中的元素按行优先顺序排列 即第一行的所有元素排在最前面 然后是第二行的所有元素 依此类推
                        因此 可以通过以下公式计算地图中给定坐标 (x,y) 对应的数组索引 index = y * width + x 其中 width 是地图的宽度
                */

                if (current_costmap_ptr_ -> data[y * current_costmap_ptr_ -> info.width + x]) {
                    kinodynamic_searcher_ptr_ -> SetObstacle(w, h);
                }
            }
        }
        has_map_ = true;
    }
    costmap_deque_.clear();

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

        if (kinodynamic_searcher_ptr_ -> Search(start_state, goal_state)) {
            auto path = kinodynamic_searcher_ptr_ -> GetPath();
            PublishPath(path);
            PublishVehiclePath(path, 4.7, 2.0, 5u);
            PublishSearchedTree(kinodynamic_searcher_ptr_ -> GetSearchedTree());

            nav_msgs::Path path_ros;
            geometry_msgs::PoseStamped pose_stamped;
            // 将 path 转换为 ROS 中的 nav_msgs::Path 类型
            for (const auto &pose : path) { // 遍历位姿 将其转换为 ROS 中的位姿表示
                pose_stamped.header.frame_id = "world";
                pose_stamped.pose.position.x = pose.x();
                pose_stamped.pose.position.y = pose.y();
                pose_stamped.pose.position.z = 0.0;

                pose_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pose.z()); // 将欧拉角转换为四元数

                path_ros.poses.emplace_back(pose_stamped);
            }

            path_ros.header.frame_id = "world";
            path_ros.header.stamp = ros::Time::now();
            static tf::TransformBroadcaster transform_broadcaster;
            for (const auto &pose : path_ros.poses) { // 遍历 path_ros.poses 将其转换为 tf::Transform 类型
                tf::Transform transform;
                transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, 0.0));

                tf::Quaternion q;
                q.setX(pose.pose.orientation.x);
                q.setY(pose.pose.orientation.y);
                q.setZ(pose.pose.orientation.z);
                q.setW(pose.pose.orientation.w);
                transform.setRotation(q);

                transform_broadcaster.sendTransform(tf::StampedTransform(transform,
                                                                         ros::Time::now(), "world",
                                                                         "ground_link") // 发布到 ROS 系统
                );

                ros::Duration(0.05).sleep();
            }
        }


        // debug
        // std::cout << "visited nodes: " << kinodynamic_searcher_ptr_ -> GetVisitedNodesNumber() << std::endl;
        kinodynamic_searcher_ptr_ -> Reset();
    }
}

void PlanningMethodFlow::ReadData() {
    init_pose_sub_ptr_ -> ParseData(init_pose_deque_); // 获取初始位置信息  ->  init_pose_deque_
    goal_pose_sub_ptr_ -> ParseData(goal_pose_deque_); // 获取目标位置信息  ->  goal_pose_deque_
    
    costmap_sub_ptr_ -> ParseData(costmap_deque_);
}

void PlanningMethodFlow::InitPoseData() {
    current_init_pose_ptr_ = init_pose_deque_.front();
    init_pose_deque_.pop_front();

    current_goal_pose_ptr_ = goal_pose_deque_.front();
    goal_pose_deque_.pop_front();
}

bool PlanningMethodFlow::HasGoalPose() {
    return !goal_pose_deque_.empty();
}

bool PlanningMethodFlow::HasStartPose() {
    return !init_pose_deque_.empty();
}

void PlanningMethodFlow::PublishPath(const VectorVec3d &path) {
    nav_msgs::Path nav_path;

    geometry_msgs::PoseStamped pose_stamped;

    for (const auto &pose: path) {
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = pose.x();
        pose_stamped.pose.position.y = pose.y();
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(pose.z());

        nav_path.poses.emplace_back(pose_stamped);
    }

    nav_path.header.frame_id = "world";
    nav_path.header.stamp = timestamp_;

    path_pub_.publish(nav_path);
}

void PlanningMethodFlow::PublishVehiclePath(const VectorVec3d &path, double width,
                                         double length, unsigned int vehicle_interval = 5u) {
    visualization_msgs::MarkerArray vehicle_array;

    for (unsigned int i = 0; i < path.size(); i += vehicle_interval) {
        visualization_msgs::Marker vehicle;

        if (i == 0) {
            vehicle.action = 3;
        }

        vehicle.header.frame_id = "world";
        vehicle.header.stamp = ros::Time::now();
        vehicle.type = visualization_msgs::Marker::CUBE;
        vehicle.id = static_cast<int>(i / vehicle_interval);

        vehicle.scale.x = width;
        vehicle.scale.y = length;
        vehicle.scale.z = 0.01;

        vehicle.color.a = 0.1;
        vehicle.color.r = 1.0;
        vehicle.color.b = 0.0;
        vehicle.color.g = 0.0;

        vehicle.pose.position.x = path[i].x();
        vehicle.pose.position.y = path[i].y();
        vehicle.pose.position.z = 0.0;

        vehicle.pose.orientation = tf::createQuaternionMsgFromYaw(path[i].z());
        vehicle_array.markers.emplace_back(vehicle);
    }

    vehicle_path_pub_.publish(vehicle_array);
}

void PlanningMethodFlow::PublishSearchedTree(const VectorVec4d &searched_tree) {
    visualization_msgs::Marker tree_list;
    tree_list.header.frame_id = "world";
    tree_list.header.stamp = ros::Time::now();
    tree_list.type = visualization_msgs::Marker::LINE_LIST;
    tree_list.action = visualization_msgs::Marker::ADD;
    tree_list.ns = "searched_tree";
    tree_list.scale.x = 0.02;

    tree_list.color.a = 1.0;
    tree_list.color.r = 0;
    tree_list.color.g = 0;
    tree_list.color.b = 0;

    tree_list.pose.orientation.w = 1.0;
    tree_list.pose.orientation.x = 0.0;
    tree_list.pose.orientation.y = 0.0;
    tree_list.pose.orientation.z = 0.0;

    geometry_msgs::Point point;
    for (const auto &i: searched_tree) {
        point.x = i.x();
        point.y = i.y();
        point.z = 0.0;
        tree_list.points.emplace_back(point);

        point.x = i.z();
        point.y = i.w();
        point.z = 0.0;
        tree_list.points.emplace_back(point);
    }

    searched_tree_pub_.publish(tree_list);
}