
#include "planning/planning_method.h"
#include "ros/ros.h"
#include "planning/timer.h"

#include <iostream>

PlanningMethod::PlanningMethod(double steering_angle, int steering_angle_discrete_num, double segment_length,
                         int segment_length_discrete_num, double wheel_base, double steering_penalty,
                         double reversing_penalty, double steering_change_penalty, double shot_distance,
                         int grid_size_phi) {
    wheel_base_ = wheel_base;

    segment_length_ = segment_length;

    steering_radian_ = steering_angle * M_PI / 180.0; // 将角度变为弧度
    steering_discrete_num_ = steering_angle_discrete_num;
    steering_radian_step_size_ = steering_radian_ / steering_discrete_num_;

    move_step_size_ = segment_length / segment_length_discrete_num; // 移动的步长

    // static_cast 是 c++ 中的类型转换运算符
    // static_cast<int>(segment_length_discrete_num)  ->  将 segment_length_discrete_num 值转换为 int
    segment_length_discrete_num_ = static_cast<int>(segment_length_discrete_num);

    steering_penalty_ = steering_penalty;
    steering_change_penalty_ = steering_change_penalty;
    reversing_penalty_ = reversing_penalty;

    shot_distance_ = shot_distance; 

    CHECK_EQ(static_cast<float>(segment_length_discrete_num_ * move_step_size_), static_cast<float>(segment_length_))
        << "The segment length must be divisible by the step size. segment_length: "
        << segment_length_ << " | step_size: " << move_step_size_;

    rs_path_ptr_ = std::make_shared<RSPath>(wheel_base_ / std::tan(steering_radian_));
    tie_breaker_ = 1.0 + 1e-3;

    STATE_GRID_SIZE_PHI_ = grid_size_phi; // 72

    ANGULAR_RESOLUTION_ = 360.0 / STATE_GRID_SIZE_PHI_ * M_PI / 180.0; // 角度分辨率
}

PlanningMethod::~PlanningMethod() {
    ReleaseMemory();
}

void PlanningMethod::Init(double x_lower, double x_upper, double y_lower, double y_upper,
                       double state_grid_resolution, double map_grid_resolution) {
    SetVehicleShape(4.7, 2.0, 1.3);
    
    // [m]
    map_x_lower_ = x_lower; // 世界地图 x 轴的最小值
    map_x_upper_ = x_upper; // 世界地图 x 轴的最大值
    map_y_lower_ = y_lower; // 世界地图 y 轴的最小值
    map_y_upper_ = y_upper; // 世界地图 y 轴的最大值

    STATE_GRID_RESOLUTION_ = state_grid_resolution; // current_costmap_ptr_ -> info.resolution
    MAP_GRID_RESOLUTION_ = map_grid_resolution;
    
    STATE_GRID_SIZE_X_ = std::floor((map_x_upper_ - map_x_lower_) / STATE_GRID_RESOLUTION_);
    STATE_GRID_SIZE_Y_ = std::floor((map_y_upper_ - map_y_lower_) / STATE_GRID_RESOLUTION_);

    
    MAP_GRID_SIZE_X_ = std::floor((map_x_upper_ - map_x_lower_) / MAP_GRID_RESOLUTION_);
    MAP_GRID_SIZE_Y_ = std::floor((map_y_upper_ - map_y_lower_) / MAP_GRID_RESOLUTION_);

    if (map_data_) {
        delete[] map_data_;
        map_data_ = nullptr;
    }

    // uint8_t 无符号八位整数
    map_data_ = new uint8_t[MAP_GRID_SIZE_X_ * MAP_GRID_SIZE_Y_];

    if (state_node_map_) {
        for (int i = 0; i < STATE_GRID_SIZE_X_; ++ i) {

            if (state_node_map_[i] == nullptr)
                continue;

            for (int j = 0; j < STATE_GRID_SIZE_Y_; ++ j) {
                if (state_node_map_[i][j] == nullptr)
                    continue;

                for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++ k) {
                    if (state_node_map_[i][j][k] != nullptr) {
                        delete state_node_map_[i][j][k];
                        state_node_map_[i][j][k] = nullptr;
                    }
                }
                delete[] state_node_map_[i][j];
                state_node_map_[i][j] = nullptr;
            }
            delete[] state_node_map_[i];
            state_node_map_[i] = nullptr;
        }

        delete[] state_node_map_;
        state_node_map_ = nullptr;
    }

    // 节点在地图中的状态 x y phi 位置 角度
    state_node_map_ = new StateNode::Ptr **[STATE_GRID_SIZE_X_];
    for (int i = 0; i < STATE_GRID_SIZE_X_; ++ i) {
        state_node_map_[i] = new StateNode::Ptr *[STATE_GRID_SIZE_Y_];
        for (int j = 0; j < STATE_GRID_SIZE_Y_; ++ j) {
            state_node_map_[i][j] = new StateNode::Ptr[STATE_GRID_SIZE_PHI_];
            for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++ k) {
                state_node_map_[i][j][k] = nullptr;
            }
        }
    }
}

inline bool PlanningMethod::LineCheck(double x0, double y0, double x1, double y1) {
    /*
        使用 Bresenham 算法检查 点(x0, y0) 到 (x1, y1) 的直线路径上是否存在障碍物或是否超出边界
    */
    
    // 确定直线斜率
    // abs(y1 - y0) > std::abs(x1 - x0) 斜率 > 1
    bool steep = (std::abs(y1 - y0) > std::abs(x1 - x0));

    // 根据斜率的绝对值与 1 的关系来决定是否交换坐标
    if (steep) {
        std::swap(x0, y0);
        std::swap(y1, x1);
    }

    if (x0 > x1) {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }

    // 计算增量
    auto delta_x = x1 - x0;
    auto delta_y = std::abs(y1 - y0);
    auto delta_error = delta_y / delta_x;
    decltype(delta_x) error = 0; //
    decltype(delta_x) y_step; // 
    auto yk = y0;

    // 确定步长
    if (y0 < y1) {
        y_step = 1;
    } else {
        y_step = -1;
    }

    auto N = static_cast<unsigned int>(x1 - x0);

    // 遍历直线路径上的每个像素点
    for (unsigned int i = 0; i < N; ++ i) {
        // 检查像素点是否存在障碍物或者是否超出边界
        if (steep) {
            if (HasObstacle(Vec2i(yk, x0 + i * 1.0))
                || BeyondBoundary(Vec2d(yk * MAP_GRID_RESOLUTION_,
                                        (x0 + i) * MAP_GRID_RESOLUTION_))
                    ) {
                return false;
            }
        } else {
            if (HasObstacle(Vec2i(x0 + i * 1.0, yk))
                || BeyondBoundary(Vec2d((x0 + i) * MAP_GRID_RESOLUTION_,
                                        yk * MAP_GRID_RESOLUTION_))
                    ) {
                return false;
            }
        }

        error += delta_error;
        if (error >= 0.5) {
            yk += y_step;
            error = error - 1.0;
        }
    }

    return true;
}

bool PlanningMethod::CheckCollision(const double &x, const double &y, const double &theta) {
    Timer timer;
    Mat2d R;
    R << std::cos(theta), -std::sin(theta),
            std::sin(theta), std::cos(theta);

    MatXd transformed_vehicle_shape;
    transformed_vehicle_shape.resize(8, 1);

    for (unsigned int i = 0; i < 4u; ++ i) {
        transformed_vehicle_shape.block<2, 1>(i * 2, 0)
                = R * vehicle_shape_.block<2, 1>(i * 2, 0) + Vec2d(x, y);
    }

    Vec2i transformed_pt_index_0 = Coordinate2MapGridIndex(
            transformed_vehicle_shape.block<2, 1>(0, 0)
    );

    Vec2i transformed_pt_index_1 = Coordinate2MapGridIndex(
            transformed_vehicle_shape.block<2, 1>(2, 0)
    );

    Vec2i transformed_pt_index_2 = Coordinate2MapGridIndex(
            transformed_vehicle_shape.block<2, 1>(4, 0)
    );

    Vec2i transformed_pt_index_3 = Coordinate2MapGridIndex(
            transformed_vehicle_shape.block<2, 1>(6, 0)
    );

    double y1, y0, x1, x0;
    // pt1 -> pt0
    x0 = static_cast<double>(transformed_pt_index_0.x());
    y0 = static_cast<double>(transformed_pt_index_0.y());
    x1 = static_cast<double>(transformed_pt_index_1.x());
    y1 = static_cast<double>(transformed_pt_index_1.y());

    if (!LineCheck(x1, y1, x0, y0)) {
        return false;
    }

    // pt2 -> pt1
    x0 = static_cast<double>(transformed_pt_index_1.x());
    y0 = static_cast<double>(transformed_pt_index_1.y());
    x1 = static_cast<double>(transformed_pt_index_2.x());
    y1 = static_cast<double>(transformed_pt_index_2.y());

    if (!LineCheck(x1, y1, x0, y0)) {
        return false;
    }

    // pt3 -> pt2
    x0 = static_cast<double>(transformed_pt_index_2.x());
    y0 = static_cast<double>(transformed_pt_index_2.y());
    x1 = static_cast<double>(transformed_pt_index_3.x());
    y1 = static_cast<double>(transformed_pt_index_3.y());

    if (!LineCheck(x1, y1, x0, y0)) {
        return false;
    }

    // pt0 -> pt3
    x0 = static_cast<double>(transformed_pt_index_0.x());
    y0 = static_cast<double>(transformed_pt_index_0.y());
    x1 = static_cast<double>(transformed_pt_index_3.x());
    y1 = static_cast<double>(transformed_pt_index_3.y());

    if (!LineCheck(x0, y0, x1, y1)) {
        return false;
    }

    check_collision_use_time += timer.End();
    num_check_collision ++;

    return true;
}

bool PlanningMethod::HasObstacle(const int grid_index_x, const int grid_index_y) const {
    return (grid_index_x >= 0 && grid_index_x < MAP_GRID_SIZE_X_
            && grid_index_y >= 0 && grid_index_y < MAP_GRID_SIZE_Y_
            && (map_data_[grid_index_y * MAP_GRID_SIZE_X_ + grid_index_x] == 1));
}

bool PlanningMethod::HasObstacle(const Vec2i &grid_index) const {
    int grid_index_x = grid_index[0];
    int grid_index_y = grid_index[1];

    return (grid_index_x >= 0 && grid_index_x < MAP_GRID_SIZE_X_
            && grid_index_y >= 0 && grid_index_y < MAP_GRID_SIZE_Y_
            && (map_data_[grid_index_y * MAP_GRID_SIZE_X_ + grid_index_x] == 1));
}

void PlanningMethod::SetObstacle(unsigned int x, unsigned int y) {
    // 不在地图范围内
    if (x < 0u || x > static_cast<unsigned int>(MAP_GRID_SIZE_X_)
        || y < 0u || y > static_cast<unsigned int>(MAP_GRID_SIZE_Y_)) {
        return;
    }

    map_data_[x + y * MAP_GRID_SIZE_X_] = 1;
}

void PlanningMethod::SetObstacle(const double pt_x, const double pt_y) {
    if (pt_x < map_x_lower_ || pt_x > map_x_upper_ ||
        pt_y < map_y_lower_ || pt_y > map_y_upper_) {
        return;
    }

    int grid_index_x = static_cast<int>((pt_x - map_x_lower_) / MAP_GRID_RESOLUTION_);
    int grid_index_y = static_cast<int>((pt_y - map_y_lower_) / MAP_GRID_RESOLUTION_);

    map_data_[grid_index_x + grid_index_y * MAP_GRID_SIZE_X_] = 1;
}

void PlanningMethod::SetVehicleShape(double length, double width, double rear_axle_dist) {
    /*!
    * Set vehicle shape
    * Consider the shape of the vehicle as a rectangle.
    * @param length vehicle length (a to c)
    * @param width vehicle width (a to d)
    * @param rear_axle_dist Length from rear axle to rear (a to b)
    *
    *         b
    *  a  ---------------- c
    *    |    |          |    Front
    *    |    |          |
    *  d  ----------------
    */

    vehicle_shape_.resize(8);

    // block 方法 访问矩阵子块
    // block<2, 1>(0, 0) -> 从 (0, 0) 开始到 (1, 0) 的子块
    vehicle_shape_.block<2, 1>(0, 0) = Vec2d(-rear_axle_dist, width / 2);
    vehicle_shape_.block<2, 1>(2, 0) = Vec2d(length - rear_axle_dist, width / 2);
    vehicle_shape_.block<2, 1>(4, 0) = Vec2d(length - rear_axle_dist, -width / 2);
    vehicle_shape_.block<2, 1>(6, 0) = Vec2d(-rear_axle_dist, -width / 2);

    const double step_size = move_step_size_;
    const auto N_length = static_cast<unsigned int>(length / step_size);
    const auto N_width = static_cast<unsigned int> (width / step_size);

    // 2u -> 值为 2 的无符号整数
    vehicle_shape_discrete_.resize(2, (N_length + N_width) * 2u);

    const Vec2d edge_0_normalized = (vehicle_shape_.block<2, 1>(2, 0)
                                     - vehicle_shape_.block<2, 1>(0, 0)).normalized(); // 计算单位向量

    for (unsigned int i = 0; i < N_length; ++ i) { // 
        vehicle_shape_discrete_.block<2, 1>(0, i + N_length)
                = vehicle_shape_.block<2, 1>(4, 0) - edge_0_normalized * i * step_size;
        vehicle_shape_discrete_.block<2, 1>(0, i)
                = vehicle_shape_.block<2, 1>(0, 0) + edge_0_normalized * i * step_size;
    }

    const Vec2d edge_1_normalized = (vehicle_shape_.block<2, 1>(4, 0)
                                     - vehicle_shape_.block<2, 1>(2, 0)).normalized();

    for (unsigned int i = 0; i < N_width; ++ i) { // 
        vehicle_shape_discrete_.block<2, 1>(0, (2 * N_length) + i)
                = vehicle_shape_.block<2, 1>(2, 0) + edge_1_normalized * i * step_size;
        vehicle_shape_discrete_.block<2, 1>(0, (2 * N_length) + i + N_width)
                = vehicle_shape_.block<2, 1>(6, 0) - edge_1_normalized * i * step_size;
    }
}

__attribute__((unused)) Vec2d PlanningMethod::CoordinateRounding(const Vec2d &pt) const {
    return MapGridIndex2Coordinate(Coordinate2MapGridIndex(pt));
}

Vec2d PlanningMethod::MapGridIndex2Coordinate(const Vec2i &grid_index) const {
    Vec2d pt;
    pt.x() = ((double) grid_index[0] + 0.5) * MAP_GRID_RESOLUTION_ + map_x_lower_;
    pt.y() = ((double) grid_index[1] + 0.5) * MAP_GRID_RESOLUTION_ + map_y_lower_;

    return pt;
}

Vec3i PlanningMethod::State2Index(const Vec3d &state) const {
    Vec3i index;
    // 保证坐标是合理的 [cell]
    index[0] = std::min(std::max(int((state[0] - map_x_lower_) / STATE_GRID_RESOLUTION_), 0), STATE_GRID_SIZE_X_ - 1);
    index[1] = std::min(std::max(int((state[1] - map_y_lower_) / STATE_GRID_RESOLUTION_), 0), STATE_GRID_SIZE_Y_ - 1);
    index[2] = std::min(std::max(int((state[2] - (-M_PI)) / ANGULAR_RESOLUTION_), 0), STATE_GRID_SIZE_PHI_ - 1);

    return index;
}

Vec2i PlanningMethod::Coordinate2MapGridIndex(const Vec2d &pt) const {
    Vec2i grid_index;

    grid_index[0] = int((pt[0] - map_x_lower_) / MAP_GRID_RESOLUTION_);
    grid_index[1] = int((pt[1] - map_y_lower_) / MAP_GRID_RESOLUTION_);

    return grid_index;
}

void PlanningMethod::GetNeighborNodes(const StateNode::Ptr &curr_node_ptr,
                                   std::vector<StateNode::Ptr> &neighbor_nodes) { // 扩展节点
    neighbor_nodes.clear();

    for (int i = -steering_discrete_num_; i <= steering_discrete_num_; ++ i) { // -1 0 1 左转 不转 右转
        VectorVec3d intermediate_state;
        bool has_obstacle = false;

        double x = curr_node_ptr -> state_.x();
        double y = curr_node_ptr -> state_.y();
        double theta = curr_node_ptr -> state_.z();

        const double phi = i * steering_radian_step_size_;

        // forward
        for (int j = 1; j <= segment_length_discrete_num_; j++ ) {
            DynamicModel(move_step_size_, phi, x, y, theta);
            intermediate_state.emplace_back(Vec3d(x, y, theta));

            if (!CheckCollision(x, y, theta)) {
                has_obstacle = true;
                break;
            }
        }

        Vec3i grid_index = State2Index(intermediate_state.back());

        if (!BeyondBoundary(intermediate_state.back().head(2)) && !has_obstacle) { // 对最后一个点判定
            auto neighbor_forward_node_ptr = new StateNode(grid_index);

            neighbor_forward_node_ptr -> intermediate_states_ = intermediate_state;
            neighbor_forward_node_ptr -> state_ = intermediate_state.back();
            neighbor_forward_node_ptr -> steering_grade_ = i;
            neighbor_forward_node_ptr -> direction_ = StateNode::FORWARD;

            neighbor_nodes.push_back(neighbor_forward_node_ptr); // 这个点是合理的(没有碰到边界 也没有碰撞障碍物)
        }

        // backward
        has_obstacle = false;
        intermediate_state.clear();

        x = curr_node_ptr -> state_.x();
        y = curr_node_ptr -> state_.y();
        theta = curr_node_ptr -> state_.z();

        for (int j = 1; j <= segment_length_discrete_num_; j++ ) {
            DynamicModel(-move_step_size_, phi, x, y, theta);
            intermediate_state.emplace_back(Vec3d(x, y, theta));

            if (!CheckCollision(x, y, theta)) {
                has_obstacle = true;
                break;
            }
        }

        if (!BeyondBoundary(intermediate_state.back().head(2)) && !has_obstacle) {
            grid_index = State2Index(intermediate_state.back());

            auto neighbor_backward_node_ptr = new StateNode(grid_index);
            neighbor_backward_node_ptr -> intermediate_states_ = intermediate_state;
            neighbor_backward_node_ptr -> state_ = intermediate_state.back();
            neighbor_backward_node_ptr -> steering_grade_ = i;
            neighbor_backward_node_ptr -> direction_ = StateNode::BACKWARD;

            neighbor_nodes.push_back(neighbor_backward_node_ptr);
        }
    }
}

void PlanningMethod::DynamicModel(const double &step_size, const double &phi,
                               double &x, double &y, double &theta) const { // 更新车辆位置和方向
    x = x + step_size * std::cos(theta);
    y = y + step_size * std::sin(theta);

    theta = Mod2Pi(theta + step_size / wheel_base_ * std::tan(phi));
}

double PlanningMethod::Mod2Pi(const double &x) { // 将给定的角度转换为等效的角度 使其位于区间 [-π, π] 内
    double v = fmod(x, 2 * M_PI);
  
    if (v < -M_PI) {
        v += 2.0 * M_PI;
    } else if (v > M_PI) {
        v -= 2.0 * M_PI;
    }

    return v;
}

bool PlanningMethod::BeyondBoundary(const Vec2d &pt) const { // 边界判定
    return pt.x() < map_x_lower_ || pt.x() > map_x_upper_ || pt.y() < map_y_lower_ || pt.y() > map_y_upper_;
}

double PlanningMethod::ComputeH(const StateNode::Ptr &current_node_ptr,
                             const StateNode::Ptr &terminal_node_ptr) { // 当前到目标的启发值
    double h;
    // L2 欧几里德范数
    // h = (current_node_ptr -> state_.head(2) - terminal_node_ptr -> state_.head(2)).norm();

    // L1 曼哈顿范数
    h = (current_node_ptr -> state_.head(2) - terminal_node_ptr -> state_.head(2)).lpNorm<1>();

    if (h < 3.0 * shot_distance_) {
        h = rs_path_ptr_ -> Distance(current_node_ptr -> state_.x(), current_node_ptr -> state_.y(),
                                   current_node_ptr -> state_.z(),
                                   terminal_node_ptr -> state_.x(), terminal_node_ptr -> state_.y(),
                                   terminal_node_ptr -> state_.z());
    }

    return h;
}

double PlanningMethod::ComputeG(const StateNode::Ptr &current_node_ptr,
                             const StateNode::Ptr &neighbor_node_ptr) const { // 当前的 g(x)
    double g;

    if (neighbor_node_ptr -> direction_ == StateNode::FORWARD) {

        if (neighbor_node_ptr -> steering_grade_ != current_node_ptr -> steering_grade_) { // 发生转向
            // 尽量走直线
            if (neighbor_node_ptr -> steering_grade_ == 0) { // 从转弯回正
                g = segment_length_ * steering_change_penalty_;
            } else { // 打方向盘
                g = segment_length_ * steering_change_penalty_ * steering_penalty_;
            }

        } else { // 方向没有变化

            if (neighbor_node_ptr -> steering_grade_ == 0) {
                g = segment_length_;
            } else {
                g = segment_length_ * steering_penalty_;
            }
        }

    } else { // 倒车

        if (neighbor_node_ptr -> steering_grade_ != current_node_ptr -> steering_grade_) {
            if (neighbor_node_ptr -> steering_grade_ == 0) {
                g = segment_length_ * steering_change_penalty_ * reversing_penalty_;
            } else {
                g = segment_length_ * steering_change_penalty_ * steering_penalty_ * reversing_penalty_;
            }

        } else {

            if (neighbor_node_ptr -> steering_grade_ == 0) {
                g = segment_length_ * reversing_penalty_;
            } else {
                g = segment_length_ * steering_penalty_ * reversing_penalty_;
            }
        }
    }

    return g;
}

bool PlanningMethod::Search(const Vec3d &start_state, const Vec3d &goal_state) {
    Timer search_used_time;

    double neighbor_time = 0.0, compute_h_time = 0.0, compute_g_time = 0.0;

    const Vec3i start_grid_index = State2Index(start_state);
    const Vec3i goal_grid_index = State2Index(goal_state);

    auto goal_node_ptr = new StateNode(goal_grid_index);
    goal_node_ptr -> state_ = goal_state;
    goal_node_ptr -> direction_ = StateNode::NO; 
    goal_node_ptr -> steering_grade_ = 0;

    auto start_node_ptr = new StateNode(start_grid_index);
    start_node_ptr -> state_ = start_state;
    start_node_ptr -> steering_grade_ = 0;
    start_node_ptr -> direction_ = StateNode::NO;
    start_node_ptr -> node_status_ = StateNode::IN_OPENSET;
    start_node_ptr -> intermediate_states_.emplace_back(start_state);
    start_node_ptr -> g_cost_ = 0.0;
    start_node_ptr -> f_cost_ = ComputeH(start_node_ptr, goal_node_ptr);

    state_node_map_[start_grid_index.x()][start_grid_index.y()][start_grid_index.z()] = start_node_ptr;
    state_node_map_[goal_grid_index.x()][goal_grid_index.y()][goal_grid_index.z()] = goal_node_ptr;

    openset_.clear();
    openset_.insert(std::make_pair(0, start_node_ptr)); // (f_cost_, StateNode)

    std::vector<StateNode::Ptr> neighbor_nodes_ptr;
    StateNode::Ptr current_node_ptr;
    StateNode::Ptr neighbor_node_ptr;

    int count = 0;

    while (!openset_.empty()) {
        current_node_ptr = openset_.begin() -> second;
        current_node_ptr -> node_status_ = StateNode::IN_CLOSESET;
        openset_.erase(openset_.begin());

        // 距离足够近
        if ((current_node_ptr -> state_.head(2) - goal_node_ptr -> state_.head(2)).norm() <= shot_distance_) {
            double rs_length = 0.0;

            if (AnalyticExpansions(current_node_ptr, goal_node_ptr, rs_length)) {
                terminal_node_ptr_ = goal_node_ptr;

                // 回溯
                StateNode::Ptr grid_node_ptr = terminal_node_ptr_ -> parent_node_;

                while (grid_node_ptr != nullptr) {
                    grid_node_ptr = grid_node_ptr -> parent_node_;
                    path_length_ = path_length_ + segment_length_;
                }
                path_length_ = path_length_ - segment_length_ + rs_length;

                std::cout << "ComputeH use time(ms): " << compute_h_time << std::endl;
                std::cout << "check collision use time(ms): " << check_collision_use_time << std::endl;
                std::cout << "GetNeighborNodes use time(ms): " << neighbor_time << std::endl;
                std::cout << "average time of check collision(ms): "
                          << check_collision_use_time / num_check_collision
                          << std::endl;
                ROS_INFO("\033[1;32m - ->  Time in Hybrid A star is %f ms, path length: %f  \033[0m\n",
                         search_used_time.End(), path_length_);

                check_collision_use_time = 0.0;
                num_check_collision = 0.0;

                return true;
            }
        }

        Timer timer_get_neighbor;
        GetNeighborNodes(current_node_ptr, neighbor_nodes_ptr);
        neighbor_time = neighbor_time + timer_get_neighbor.End();

        for (unsigned int i = 0; i < neighbor_nodes_ptr.size(); ++ i) { // 计算 cost
            neighbor_node_ptr = neighbor_nodes_ptr[i];

            Timer timer_compute_g;
            const double neighbor_edge_cost = ComputeG(current_node_ptr, neighbor_node_ptr);
            compute_g_time = compute_g_time + timer_get_neighbor.End();

            Timer timer_compute_h;
            const double current_h = ComputeH(current_node_ptr, goal_node_ptr) * tie_breaker_;
            compute_h_time = compute_h_time + timer_compute_h.End();

            const Vec3i &index = neighbor_node_ptr -> grid_index_; // 索引

            if (state_node_map_[index.x()][index.y()][index.z()] == nullptr) {
                neighbor_node_ptr -> g_cost_ = current_node_ptr -> g_cost_ + neighbor_edge_cost; // 起点到当前的距离
                neighbor_node_ptr -> parent_node_ = current_node_ptr;
                neighbor_node_ptr -> node_status_ = StateNode::IN_OPENSET;
                neighbor_node_ptr -> f_cost_ = neighbor_node_ptr -> g_cost_ + current_h; // f(x) = g(x) + h(x)
                openset_.insert(std::make_pair(neighbor_node_ptr -> f_cost_, neighbor_node_ptr));
                
                state_node_map_[index.x()][index.y()][index.z()] = neighbor_node_ptr;
                continue;
            
            } else if (state_node_map_[index.x()][index.y()][index.z()] -> node_status_ == StateNode::IN_OPENSET) {
                double g_cost_temp = current_node_ptr -> g_cost_ + neighbor_edge_cost; // 当前的 g(x)

                if (state_node_map_[index.x()][index.y()][index.z()] -> g_cost_ > g_cost_temp) {
                    neighbor_node_ptr -> g_cost_ = g_cost_temp;
                    neighbor_node_ptr -> f_cost_ = g_cost_temp + current_h;
                    neighbor_node_ptr -> parent_node_ = current_node_ptr;
                    neighbor_node_ptr -> node_status_ = StateNode::IN_OPENSET;

                    /// TODO: This will cause a memory leak
                    //delete state_node_map_[index.x()][index.y()][index.z()];
                    state_node_map_[index.x()][index.y()][index.z()] = neighbor_node_ptr;
               
                } else {
                    delete neighbor_node_ptr;
                }
                continue;
            
            } else if (state_node_map_[index.x()][index.y()][index.z()] -> node_status_ == StateNode::IN_CLOSESET) {
                delete neighbor_node_ptr; // 从 close 中删除
                continue;
            }
        }

        count ++;
        if (count > 50000) {
            ROS_WARN("Exceeded the number of iterations, the search failed");
            return false;
        }
    }

    return false;
}

VectorVec4d PlanningMethod::GetSearchedTree() {
    VectorVec4d tree;
    Vec4d point_pair;

    visited_node_number_ = 0;
    for (int i = 0; i < STATE_GRID_SIZE_X_; ++ i) {
        for (int j = 0; j < STATE_GRID_SIZE_Y_; ++ j) {
            for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++ k) {
                if (state_node_map_[i][j][k] == nullptr || state_node_map_[i][j][k] -> parent_node_ == nullptr) {
                    continue;
                }

                const unsigned int number_states = state_node_map_[i][j][k] -> intermediate_states_.size() - 1;
                for (unsigned int l = 0; l < number_states; ++ l) {
                    point_pair.head(2) = state_node_map_[i][j][k] -> intermediate_states_[l].head(2);
                    point_pair.tail(2) = state_node_map_[i][j][k] -> intermediate_states_[l + 1].head(2);

                    tree.emplace_back(point_pair);
                }

                point_pair.head(2) = state_node_map_[i][j][k] -> intermediate_states_[0].head(2);
                point_pair.tail(2) = state_node_map_[i][j][k] -> parent_node_ -> state_.head(2);
                tree.emplace_back(point_pair);
                visited_node_number_ ++;
            }
        }
    }

    return tree;
}

void PlanningMethod::ReleaseMemory() {
    if (map_data_ != nullptr) {
        delete[] map_data_;
        map_data_ = nullptr;
    }

    if (state_node_map_) {
        for (int i = 0; i < STATE_GRID_SIZE_X_; ++ i) {
            if (state_node_map_[i] == nullptr)
                continue;

            for (int j = 0; j < STATE_GRID_SIZE_Y_; ++ j) {
                if (state_node_map_[i][j] == nullptr)
                    continue;

                for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++ k) {
                    if (state_node_map_[i][j][k] != nullptr) {
                        delete state_node_map_[i][j][k];
                        state_node_map_[i][j][k] = nullptr;
                    }
                }

                delete[] state_node_map_[i][j];
                state_node_map_[i][j] = nullptr;
            }

            delete[] state_node_map_[i];
            state_node_map_[i] = nullptr;
        }

        delete[] state_node_map_;
        state_node_map_ = nullptr;
    }

    terminal_node_ptr_ = nullptr;
}

__attribute__((unused)) double PlanningMethod::GetPathLength() const {
    return path_length_;
}

VectorVec3d PlanningMethod::GetPath() const {
    VectorVec3d path;

    std::vector<StateNode::Ptr> temp_nodes;

    StateNode::Ptr state_grid_node_ptr = terminal_node_ptr_;
    
    while (state_grid_node_ptr != nullptr) {
        temp_nodes.emplace_back(state_grid_node_ptr);
        state_grid_node_ptr = state_grid_node_ptr -> parent_node_;
    }

    std::reverse(temp_nodes.begin(), temp_nodes.end());
    for (const auto &node: temp_nodes) {
        path.insert(path.end(), node -> intermediate_states_.begin(),
                    node -> intermediate_states_.end());
    }

    return path;
}

void PlanningMethod::Reset() {
    if (state_node_map_) {
        for (int i = 0; i < STATE_GRID_SIZE_X_; ++ i) {
            if (state_node_map_[i] == nullptr)
                continue;

            for (int j = 0; j < STATE_GRID_SIZE_Y_; ++ j) {
                if (state_node_map_[i][j] == nullptr)
                    continue;

                for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++ k) {
                    if (state_node_map_[i][j][k] != nullptr) {
                        delete state_node_map_[i][j][k];
                        state_node_map_[i][j][k] = nullptr;
                    }
                }
            }
        }
    }

    path_length_ = 0.0;
    terminal_node_ptr_ = nullptr;
}

bool PlanningMethod::AnalyticExpansions(const StateNode::Ptr &current_node_ptr,
                                     const StateNode::Ptr &goal_node_ptr, double &length) {
    VectorVec3d rs_path_poses = rs_path_ptr_ -> GetRSPath(current_node_ptr -> state_,
                                                        goal_node_ptr -> state_,
                                                        move_step_size_, length);

    for (const auto &pose : rs_path_poses)
        if (BeyondBoundary(pose.head(2)) || !CheckCollision(pose.x(), pose.y(), pose.z())) {
            return false;
        };

    goal_node_ptr -> intermediate_states_ = rs_path_poses;
    goal_node_ptr -> parent_node_ = current_node_ptr;

    auto begin = goal_node_ptr -> intermediate_states_.begin();
    goal_node_ptr -> intermediate_states_.erase(begin);

    return true;
}
