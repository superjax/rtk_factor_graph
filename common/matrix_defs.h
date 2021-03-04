
#pragma once

#include <Eigen/Core>
#include <vector>

namespace mc {

using Vec1 = Eigen::Matrix<double, 1, 1>;
using Vec2 = Eigen::Matrix<double, 2, 1>;
using Vec3 = Eigen::Matrix<double, 3, 1>;
using Vec4 = Eigen::Matrix<double, 4, 1>;
using Vec5 = Eigen::Matrix<double, 5, 1>;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Vec7 = Eigen::Matrix<double, 7, 1>;
using Vec8 = Eigen::Matrix<double, 8, 1>;
using Vec9 = Eigen::Matrix<double, 9, 1>;
using Vec10 = Eigen::Matrix<double, 10, 1>;
using Vec11 = Eigen::Matrix<double, 11, 1>;
using Vec12 = Eigen::Matrix<double, 12, 1>;
using Vec30 = Eigen::Matrix<double, 30, 1>;

using Vec1f = Eigen::Matrix<float, 1, 1>;
using Vec2f = Eigen::Matrix<float, 2, 1>;
using Vec3f = Eigen::Matrix<float, 3, 1>;
using Vec4f = Eigen::Matrix<float, 4, 1>;
using Vec5f = Eigen::Matrix<float, 5, 1>;
using Vec6f = Eigen::Matrix<float, 6, 1>;
using Vec7f = Eigen::Matrix<float, 7, 1>;
using Vec8f = Eigen::Matrix<float, 8, 1>;
using Vec9f = Eigen::Matrix<float, 9, 1>;
using Vec10f = Eigen::Matrix<float, 10, 1>;
using Vec11f = Eigen::Matrix<float, 11, 1>;
using Vec12f = Eigen::Matrix<float, 12, 1>;
using Vec30f = Eigen::Matrix<float, 30, 1>;

using Mat1 = Eigen::Matrix<double, 1, 1>;
using Mat2 = Eigen::Matrix<double, 2, 2>;
using Mat3 = Eigen::Matrix<double, 3, 3>;
using Mat4 = Eigen::Matrix<double, 4, 4>;
using Mat5 = Eigen::Matrix<double, 5, 5>;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using Mat7 = Eigen::Matrix<double, 7, 7>;
using Mat8 = Eigen::Matrix<double, 8, 8>;
using Mat9 = Eigen::Matrix<double, 9, 9>;
using Mat10 = Eigen::Matrix<double, 10, 10>;
using Mat11 = Eigen::Matrix<double, 11, 11>;
using Mat12 = Eigen::Matrix<double, 12, 12>;
using Mat30 = Eigen::Matrix<double, 30, 30>;

using Mat36 = Eigen::Matrix<double, 3, 6>;
using Mat38 = Eigen::Matrix<double, 3, 8>;
using Mat93 = Eigen::Matrix<double, 9, 3>;
using Mat96 = Eigen::Matrix<double, 9, 6>;
using Mat98 = Eigen::Matrix<double, 9, 8>;

using MatRM3 = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;
using MatRM32 = Eigen::Matrix<double, 3, 2, Eigen::RowMajor>;
using MatRM36 = Eigen::Matrix<double, 3, 6, Eigen::RowMajor>;
using MatRM38 = Eigen::Matrix<double, 3, 8, Eigen::RowMajor>;
using MatRM68 = Eigen::Matrix<double, 6, 8, Eigen::RowMajor>;
using MatRM93 = Eigen::Matrix<double, 9, 3, Eigen::RowMajor>;
using MatRM96 = Eigen::Matrix<double, 9, 6, Eigen::RowMajor>;
using MatRM98 = Eigen::Matrix<double, 9, 8, Eigen::RowMajor>;

using Mat1f = Eigen::Matrix<float, 1, 1>;
using Mat2f = Eigen::Matrix<float, 2, 2>;
using Mat3f = Eigen::Matrix<float, 3, 3>;
using Mat4f = Eigen::Matrix<float, 4, 4>;
using Mat5f = Eigen::Matrix<float, 5, 5>;
using Mat6f = Eigen::Matrix<float, 6, 6>;
using Mat7f = Eigen::Matrix<float, 7, 7>;
using Mat8f = Eigen::Matrix<float, 8, 8>;
using Mat9f = Eigen::Matrix<float, 9, 9>;
using Mat10f = Eigen::Matrix<float, 10, 10>;

using DiagMat1 = Eigen::DiagonalMatrix<double, 1>;
using DiagMat2 = Eigen::DiagonalMatrix<double, 2>;
using DiagMat3 = Eigen::DiagonalMatrix<double, 3>;
using DiagMat4 = Eigen::DiagonalMatrix<double, 4>;
using DiagMat5 = Eigen::DiagonalMatrix<double, 5>;
using DiagMat6 = Eigen::DiagonalMatrix<double, 6>;
using DiagMat7 = Eigen::DiagonalMatrix<double, 7>;
using DiagMat8 = Eigen::DiagonalMatrix<double, 8>;
using DiagMat9 = Eigen::DiagonalMatrix<double, 9>;
using DiagMat10 = Eigen::DiagonalMatrix<double, 10>;
using DiagMat30 = Eigen::DiagonalMatrix<double, 30>;

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
inline Eigen::MatrixXd hstack(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b)
{
    assert(a.rows() == b.rows() && a.rows() != 0);
    Eigen::MatrixXd out(a.rows(), a.cols() + b.cols());
    out << a, b;
    return out;
}

inline Eigen::MatrixXd vstack(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b)
{
    assert(a.cols() == b.cols() && a.cols() != 0);
    Eigen::MatrixXd out(a.rows() + b.rows(), a.cols());
    out << a, b;
    return out;
}

template <typename Derived>
bool isNan(const Eigen::MatrixBase<Derived>& m)
{
    return (m.array() != m.array()).any();
}

template <typename Derived>
bool isFinite(const Eigen::MatrixBase<Derived>& m)
{
    return (m.array() == m.array()).all();
}

template <typename Derived>
bool isFinite(const Eigen::DiagonalBase<Derived>& m)
{
    return (m.diagonal().array() == m.diagonal().array()).all();
}

const Vec3 GRAVITY = 9.80665 * Vec3::UnitZ();

namespace detail {
using argsort_pair = std::pair<int, double>;

inline bool argsort_comp(const argsort_pair& left, const argsort_pair& right)
{
    return left.second < right.second;
}
}  // namespace detail

template <typename Derived>
Eigen::VectorXd argsort(const Eigen::MatrixBase<Derived>& x)
{
    Eigen::VectorXd indices(x.size());
    std::vector<detail::argsort_pair> data(x.size());
    for (int i = 0; i < x.size(); i++)
    {
        data[i].first = i;
        data[i].second = x(i);
    }
    std::sort(data.begin(), data.end(), detail::argsort_comp);
    for (size_t i = 0; i < data.size(); i++)
    {
        indices(data[i].first) = i;
    }
    return indices;
}

template <typename Derived>
Derived modf(const Derived& x, Derived* intpart)
{
    Eigen::Matrix<int, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime> int_cast =
        x.template cast<int>();
    (*intpart) = int_cast.template cast<double>();
    return x - (*intpart);
}

template <typename Derived>
Derived modf(const Derived& x)
{
    Eigen::Matrix<int, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime> int_cast =
        x.template cast<int>();
    Eigen::Matrix<double, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime> floored =
        int_cast.template cast<double>();

    return x - floored;
}

namespace detail {
template <typename T>
std::true_type is_eigen_test(const Eigen::MatrixBase<T>*);
std::false_type is_eigen_test(...);

template <typename T>
struct is_eigen : public decltype(is_eigen_test(std::declval<T*>()))
{
};

template <typename T>
struct is_lie_group;

template <typename T>
struct is_lie_group
{
    static constexpr std::false_type value = std::false_type();
};

}  // namespace detail
}  // namespace mc
