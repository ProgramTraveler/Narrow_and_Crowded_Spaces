#include "planning/planning_method.h"

PlanningMethod::PlanningMethod(double steering_angle, int steering_angle_discrete_num, double segment_length, 
                                int segment_length_discrete_num, double wheel_base, double steering_penalty,
                                double reversing_penalty, double steering_change_penalty, double shot_distance,
                                int grid_size_phi) {
    
    wheel_base_ = wheel_base;
    segment_length_ = segment_length;
    steering_radian_ = steering_angle * M_PI / 180.0; // 将角度变为弧度
    steering_discrete_num_ = steering_angle_discrete_num;
    steering_radian_step_size_ = steering_radian_ / steering_discrete_num_; //
    move_step_size_ = segment_length / segment_length_discrete_num; //
    segment_length_discrete_num_ = static_cast<int>(segment_length_discrete_num); //
    steering_penalty_ = steering_penalty;
    steering_change_penalty_ = steering_change_penalty;
    reversing_penalty_ = reversing_penalty;
    shot_distance_ = shot_distance;

    
}

PlanningMethod::~PlanningMethod() {
    ReleaseMemory();
}

void PlanningMethod::Init(double x_lower, double x_upper, double y_lower, double y_upper, 
                    double state_grid_resolution, double map_grid_resolution) {
    SetVehicleShape(4.7, 2.0, 1.3);

    map_x_lower_ = x_lower;
    map_x_upper_ = x_upper;
    map_y_lower_ = y_lower;
    map_y_upper_ = y_upper;

    STATE_GRID_RESOLUTION_ = state_grid_resolution;
    MAP_GRID_RESOLUTION_ = map_grid_resolution;

    STATE_GRID_SIZE_X_ = std::floor((map_x_upper_ - map_x_lower_) / STATE_GRID_RESOLUTION_);
    STATE_GRID_SIZE_Y_ = std::floor((map_y_upper_ - map_y_lower_) / STATE_GRID_RESOLUTION_);

    MAP_GRID_SIZE_X_ = std::floor((map_x_upper_ - map_x_lower_) / MAP_GRID_RESOLUTION_);
    MAP_GRID_SIZE_Y_ = std::floor((map_y_upper_ - map_y_lower_) / MAP_GRID_RESOLUTION_);

    if (map_data_) {
        delete[] map_data_;

        map_data_ = nullptr;
    }

    map_data_ = new uint8_t[MAP_GRID_SIZE_X_ * MAP_GRID_SIZE_Y_];

    if (state_node_map_) {
        for (int i = 0; i < STATE_GRID_SIZE_X_; i ++) {
            if (state_node_map_[i] == nullptr) continue;

            for (int j = 0; j < STATE_GRID_SIZE_Y_; j ++) {
                if (state_node_map_[i][j] == nullptr) continue;

                for (int k = 0; k < STATE_GRID_SIZE_PHI_; k ++) {
                    if (state_node_map_[i][j][k] != nullptr) {
                        delete[] state_node_map_[i][j][k];

                        state_node_map_[i][j][k] = nullptr;
                    }
                }
                delete[] state_node_map_[i][j];
                state_node_map_[i][j] = nullptr;
            }

            delete[] state_node_map_[i];
            state_node_map_[i] = nullptr;
        }

        state_node_map_ = new StateNode::Ptr **[STATE_GRID_SIZE_X_];
        for (int i = 0; i < STATE_GRID_SIZE_X_; i ++) {
            state_node_map_[i] = new StateNode::Ptr *[STATE_GRID_SIZE_Y_];

            for (int j = 0; j < STATE_GRID_SIZE_Y_; j ++) {
                state_node_map_[i][j] = new StateNode::Ptr [STATE_GRID_SIZE_PHI_];

                for (int k = 0; k < STATE_GRID_SIZE_PHI_; k ++) {
                    state_node_map_[i][j][k] = nullptr;
                }
            }
        }
    }
}

// inline bool PlanningMethod:: LineCheck(double x0, double y0, double x1, double y1) {

// }

// bool PlanningMethod::CheckCollision(const double &x, const double &y, const double &theta) {

// }

bool PlanningMethod::HasObstacle(const int grid_index_x, const int grid_index_y) const {
    return (grid_index_x >= 0 && grid_index_x < MAP_GRID_SIZE_X_
            && grid_index_y >= 0 && grid_index_y < MAP_GRID_SIZE_Y_
            && (map_data_[grid_index_y * MAP_GRID_SIZE_X_ + grid_index_x] == 1));
}

// bool PlanningMethod::HasObstacle(const Vec2i &grid_index) const {

// }

void PlanningMethod::SetObstacle(unsigned int x, unsigned int y) {

}

void PlanningMethod::SetObstacle(const double pt_x, const double pt_y) {

}


void PlanningMethod::SetVehicleShape(double length, double width, double rear_axle_dist) {
    vehicle_shape_.resize(8);
    vehicle_shape_.block<2, 1>(0, 0) = Vec2d(-rear_axle_dist, width / 2);
    vehicle_shape_.block<2, 1>(2, 0) = Vec2d(length - rear_axle_dist, width / 2);
    vehicle_shape_.block<2, 1>(4, 0) = Vec2d(length - rear_axle_dist, -width / 2);
    vehicle_shape_.block<2, 1>(6, 0) = Vec2d(-rear_axle_dist, -width / 2);

    const double step_size = move_step_size_;
    const auto N_length = static_cast<unsigned int>(length / step_size);
    const auto N_width = static_cast<unsigned int>(width / step_size);

    vehicle_shape_discrete_.resize(2, (N_length + N_width) * 2u);

    const Vec2d edge_0_normalized = (vehicle_shape_.block<2, 1>(2, 0) 
                                    - vehicle_shape_.block<2, 1>(0, 0)).normalized();

    for (unsigned int i = 0; i < N_length; i ++) {
        vehicle_shape_discrete_.block<2, 1>(0, i + N_length) = vehicle_shape_.block<2, 1>(4, 0) 
                                                                - edge_0_normalized * i * step_size;

        vehicle_shape_discrete_.block<2, 1>(0, i) = vehicle_shape_.block<2, 1>(0, 0) + edge_0_normalized * i * step_size;
    }

    const Vec2d edge_1_normalized = (vehicle_shape_.block<2, 1>(4, 0)
                                    - vehicle_shape_.block<2, 1>(2, 0)).normalized();

    for (unsigned int i = 0; i < N_width; i ++) {
        vehicle_shape_discrete_.block<2, 1>(0, (2 * N_length) + i) = vehicle_shape_.block<2, 1>(2, 0) 
                                                                    + edge_1_normalized * i * step_size;

        vehicle_shape_discrete_.block<2, 1>(0, (2 * N_length) + i + N_width) = vehicle_shape_.block<2, 1>(6, 0) 
                                                                                - edge_1_normalized * i * step_size;
    }
}

// __attribute__((unused)) Vec2d PlanningMethod::CoordinateRounding(const Vec2d &pt) const {

// }

// Vec2d PlanningMethod::MapGridIndex2Coordinate(const Vec2i &grid_index) const {

// }

// Vec3i PlanningMethod::State2Index(const Vec3d &state) const {

// }

// Vec2i PlanningMethod::Coordinate2MapGridIndex(const Vec2d &pt) const {

// }

void PlanningMethod::GetNeighborNodes(const StateNode::Ptr &curr_node_ptr,
                                        std::vector<StateNode::Ptr> &neighbor_node) {

}

void PlanningMethod::DynamicModel(const double &step_size, const double &phi, double &x, double &y, double &theta) const {

}

// double PlanningMethod::Mod2Pi(const double &x) {

// }

// bool PlanningMethod::BeyondBoundary(const Vec2d &pt) const {

// }

// double PlanningMethod::ComputeH(const StateNode::Ptr &current_node_ptr, const StateNode::Ptr &terminal_node_ptr) {

// }

// double PlanningMethod::ComputeG(const StateNode::Ptr &current_node_ptr, const StateNode::Ptr &terminal_node_ptr) const {

// }

bool PlanningMethod::Search(const Vec3d &start_state, const Vec3d &goal_state) {

}

// VectorVec4d PlanningMethod::GetSearchedTree() {

// }

void PlanningMethod::ReleaseMemory() {

}

// __attribute__((unused)) double PlanningMethod::GetPathLength() const {

// }

// VectorVec3d PlanningMethod::GetPath() const {

// }

void PlanningMethod::Reset() {

}

// bool PlanningMethod::AnalyticExpansions(const StateNode::Ptr &current_node_ptr, const StateNode::Ptr &goal_node_ptr, double &length) {

// }