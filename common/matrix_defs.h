
#pragma once

#include <Eigen/Core>

typedef Eigen::Matrix<double, 1, 1> Vec1;
typedef Eigen::Matrix<double, 2, 1> Vec2;
typedef Eigen::Matrix<double, 3, 1> Vec3;
typedef Eigen::Matrix<double, 4, 1> Vec4;
typedef Eigen::Matrix<double, 5, 1> Vec5;
typedef Eigen::Matrix<double, 6, 1> Vec6;
typedef Eigen::Matrix<double, 7, 1> Vec7;
typedef Eigen::Matrix<double, 8, 1> Vec8;
typedef Eigen::Matrix<double, 9, 1> Vec9;
typedef Eigen::Matrix<double, 10, 1> Vec10;

typedef Eigen::Matrix<double, 1, 1> Mat1;
typedef Eigen::Matrix<double, 2, 2> Mat2;
typedef Eigen::Matrix<double, 3, 3> Mat3;
typedef Eigen::Matrix<double, 4, 4> Mat4;
typedef Eigen::Matrix<double, 5, 5> Mat5;
typedef Eigen::Matrix<double, 6, 6> Mat6;
typedef Eigen::Matrix<double, 7, 7> Mat7;
typedef Eigen::Matrix<double, 8, 8> Mat8;
typedef Eigen::Matrix<double, 9, 9> Mat9;
typedef Eigen::Matrix<double, 10, 10> Mat10;

static const Eigen::Vector3d e_x(1.0, 0, 0);
static const Eigen::Vector3d e_y(0, 1.0, 0);
static const Eigen::Vector3d e_z(0, 0, 1.0);

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> skew(const Eigen::MatrixBase<Derived>& v)
{
    static_assert(Derived::RowsAtCompileTime == 3 || Derived::RowsAtCompileTime == -1);
    Eigen::Matrix<typename Derived::Scalar, 3, 3> mat;
    typename Derived::Scalar zr(0.0);
    // clang-format off
    mat << zr, -v(2), v(1),
           v(2), zr, -v(0),
          -v(1), v(0), zr;
    // clang-format on
    return mat;
}

/// TODO: Use variadic templates to handle arbitrary number of arguments
Eigen::MatrixXd hstack(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b)
{
    assert(a.rows() == b.rows() && a.rows() != 0);
    Eigen::MatrixXd out(a.rows(), a.cols() + b.cols());
    out << a, b;
    return out;
}

Eigen::MatrixXd vstack(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b)
{
    assert(a.cols() == b.cols() && a.cols() != 0);
    Eigen::MatrixXd out(a.rows() + b.rows(), a.cols());
    out << a, b;
    return out;
}
