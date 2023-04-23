#ifndef PLANNING_TYPE_H
#define PALNNING_TYPE_H

// 一些模板

#include "vector"
#include "Eigen/Core"

template<int dim>
using TypeVectorVecd = typename std::vector<Eigen::Matrix<double, dim, 1>,
        Eigen::aligned_allocator<Eigen::Matrix<double, dim, 1>>>;

typedef TypeVectorVecd<4> VectorVec4d;
typedef TypeVectorVecd<3> VectorVec3d; // ?
typedef TypeVectorVecd<2> VectorVec2d;

typedef typename Eigen::Vector2d Vec2d;
typedef typename Eigen::Vector3d Vec3d; // ?
typedef typename Eigen::Vector4d Vec4d;

typedef typename Eigen::Vector2i Vec2i;
typedef typename Eigen::Vector3i Vec3i;

typedef typename Eigen::Matrix2d Mat2d;
typedef typename Eigen::Matrix3d Mat3d;

typedef typename Eigen::MatrixXd MatXd;
typedef typename Eigen::VectorXd VecXd;

#endif