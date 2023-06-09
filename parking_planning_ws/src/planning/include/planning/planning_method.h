#ifndef PLANNING_PLANNING_METHOD_H
#define PLANNING_PLANNING_METHOD_H

#include "rs_path.h"
#include "state_node.h"
#include <glog/logging.h>

#include <map>
#include <memory>

class PlanningMethod {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 宏 用于在类中重载 new 运算符 以便为 Eigen 库中的定长向量和矩阵分配正确对齐的内存

    PlanningMethod() = delete;

    PlanningMethod(double steering_angle, int steering_angle_discrete_num, double segment_length,
                int segment_length_discrete_num, double wheel_base, double steering_penalty,
                double reversing_penalty, double steering_change_penalty, double shot_distance,
                int grid_size_phi = 72);

    ~PlanningMethod();

    void Init(double x_lower, double x_upper, double y_lower, double y_upper,
              double state_grid_resolution, double map_grid_resolution = 0.1);

    bool Search(const Vec3d &start_state, const Vec3d &goal_state);

    VectorVec4d GetSearchedTree();

    VectorVec3d GetPath() const;

    /*
        __attribute__((unused)) 用于指示编译器不要对未使用的函数、变量或类型产生警告
    */
    __attribute__((unused)) int GetVisitedNodesNumber() const { return visited_node_number_; }

    __attribute__((unused)) double GetPathLength() const;

    __attribute__((unused)) Vec2d CoordinateRounding(const Vec2d &pt) const;

    Vec2i Coordinate2MapGridIndex(const Vec2d &pt) const;

    void SetObstacle(double pt_x, double pt_y);

    void SetObstacle(unsigned int x, unsigned int y);

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
    void SetVehicleShape(double length, double width, double rear_axle_dist);

    void Reset();

private:
    // inline -> 内联函数
    inline bool HasObstacle(int grid_index_x, int grid_index_y) const;

    inline bool HasObstacle(const Vec2i &grid_index) const;

    bool CheckCollision(const double &x, const double &y, const double &theta);

    inline bool LineCheck(double x0, double y0, double x1, double y1);

    bool AnalyticExpansions(const StateNode::Ptr &current_node_ptr,
                            const StateNode::Ptr &goal_node_ptr, double &length);

    inline double ComputeG(const StateNode::Ptr &current_node_ptr, const StateNode::Ptr &neighbor_node_ptr) const;

    inline double ComputeH(const StateNode::Ptr &current_node_ptr, const StateNode::Ptr &terminal_node_ptr);

    inline Vec3i State2Index(const Vec3d &state) const;

    inline Vec2d MapGridIndex2Coordinate(const Vec2i &grid_index) const;

    void GetNeighborNodes(const StateNode::Ptr &curr_node_ptr, std::vector<StateNode::Ptr> &neighbor_nodes);

    /*!
     * Simplified car model. Center of the rear axle
     * refer to: http://planning.cs.uiuc.edu/node658.html
     * @param step_size Length of discrete steps
     * @param phi Car steering angle
     * @param x Car position (world frame)
     * @param y Car position (world frame)
     * @param theta Car yaw (world frame)
     */
    inline void DynamicModel(const double &step_size, const double &phi, double &x, double &y, double &theta) const;

    static inline double Mod2Pi(const double &x);

    bool BeyondBoundary(const Vec2d &pt) const;

    void ReleaseMemory();

private:
    uint8_t *map_data_ = nullptr;
    double STATE_GRID_RESOLUTION_{}, MAP_GRID_RESOLUTION_{}; // 世界地图和栅格地图分辨率
    double ANGULAR_RESOLUTION_{};
    int STATE_GRID_SIZE_X_{}, STATE_GRID_SIZE_Y_{}, STATE_GRID_SIZE_PHI_{}; // 世界地图 x y phi

    int MAP_GRID_SIZE_X_{}, MAP_GRID_SIZE_Y_{}; // 栅格地图 x y

    double map_x_lower_{}, map_x_upper_{}, map_y_lower_{}, map_y_upper_{}; // 世界地图的 x y 轴的最小和最大值

    StateNode::Ptr terminal_node_ptr_ = nullptr;
    StateNode::Ptr ***state_node_map_ = nullptr;

    std::multimap<double, StateNode::Ptr> openset_;

    double wheel_base_; // 前后轴之间的距离

    double segment_length_;
    int segment_length_discrete_num_;

    double move_step_size_;

    double steering_radian_step_size_;
    double steering_radian_; // 弧度
    int steering_discrete_num_;

    double tie_breaker_; // 处理子结点和父节点在同一栅格的情况

    double shot_distance_; // 命中距离 当前点与目标点的距离小于该变量时视为命中终点


    double steering_penalty_; // 
    double reversing_penalty_; // 倒车惩罚
    double steering_change_penalty_;

    double path_length_ = 0.0;

    std::shared_ptr<RSPath> rs_path_ptr_;

    VecXd vehicle_shape_;
    MatXd vehicle_shape_discrete_;

    // debug
    double check_collision_use_time = 0.0;
    int num_check_collision = 0;
    int visited_node_number_ = 0;
};

#endif
