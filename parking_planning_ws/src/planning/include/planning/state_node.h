#ifndef PLANNING_STATE_NODE_H
#define PLANNING_STATE_NODE_H

#include "Eigen/Dense"
#include "planning/type.h"

struct StateNode {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // enum -> 用于定义枚举类型

    enum NODE_STATUS { // 定义节点状态
        NOT_VISITED = 0, IN_OPENSET = 1, IN_CLOSESET = 2 
    };

    enum DIRECTION { // 运动方向
        FORWARD = 0, BACKWARD = 1, NO = 3
    };

    StateNode() = delete;

    explicit StateNode(const Vec3i &grid_index) {
        node_status_ = NOT_VISITED;        parent_node_ = nullptr;
    }

    void Reset() {
        node_status_ = NOT_VISITED;
        parent_node_ = nullptr;
    }

    NODE_STATUS node_status_;

    DIRECTION direction_{};

    Vec3d state_;
    Vec3i grid_index_;

    double g_cost_{}, f_cost_{};
    int steering_grade_{};

    StateNode *parent_node_;
    typedef StateNode *Ptr;

    VectorVec3d intermediate_states_;
};

#endif