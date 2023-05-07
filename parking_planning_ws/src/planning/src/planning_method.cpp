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

}