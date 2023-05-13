#ifndef PLANNING_TYPE_H
#define PLANNING_TYPE_H

#include <vector>
#include <Eigen/Core>

// 一些模板

template<int dim>

// 定义一个新的类型别名
using TypeVectorVecd = typename std::vector<Eigen::Matrix<double, dim, 1>, // 元素类型
        Eigen::aligned_allocator<Eigen::Matrix<double, dim, 1>>>;

typedef TypeVectorVecd<4> VectorVec4d;
typedef TypeVectorVecd<3> VectorVec3d;
typedef TypeVectorVecd<2> VectorVec2d;

typedef typename Eigen::Vector2d Vec2d; // 大小为 2 的列向量 元素类型为 double
typedef typename Eigen::Vector3d Vec3d; // 大小为 3 的列向量 元素类型为 double
typedef typename Eigen::Vector4d Vec4d; // 大小为 4 的列向量 元素类型为 double

typedef typename Eigen::Vector2i Vec2i; // 大小为 2 的列向量 元素类型为 int
typedef typename Eigen::Vector3i Vec3i; // 大小为 3 的列向量 元素类型为 int

typedef typename Eigen::Matrix2d Mat2d; // 2x2 的矩阵 元素类型为 double
typedef typename Eigen::Matrix3d Mat3d; // 3x3 的矩阵 元素类型为 double

typedef typename Eigen::MatrixXd MatXd; // 动态大小的矩阵 元素类型为 double
typedef typename Eigen::VectorXd VecXd; // 动态大小的列向量 元素类型为 double

#endif