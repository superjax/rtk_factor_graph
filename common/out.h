#pragma once

#include <Eigen/Core>
#include <type_traits>

#include "common/check.h"

namespace mc {

namespace detail {
// These functions are never defined.
template <typename T>
std::true_type is_eigen_test(const Eigen::MatrixBase<T>*);
std::false_type is_eigen_test(...);

template <typename T>
struct is_eigen : public decltype(is_eigen_test(std::declval<T*>()))
{
};

}  // namespace detail

// This is a simple wrapper for a reference to a type.  It is intended to flag an argument of a
// function as an output.  It acts a lot like a pointer to the object
template <typename T>
class Out
{
    static_assert(!std::is_const<T>::value, "Cannot make an Out variable out of constant");

 public:
    explicit Out(T& val) : value_(val) {}

    T& operator*() { return value_; }
    T* operator->() { return &value_; }

 private:
    T& value_;
};

template <typename T, int Rows, int Cols>
class MatOut
{
    using BasePtr = Eigen::MatrixBase<T>*;
    static_assert(Rows == T::RowsAtCompileTime || T::RowsAtCompileTime == Eigen::Dynamic ||
                  Rows == Eigen::Dynamic);
    static_assert(Cols == T::ColsAtCompileTime || T::ColsAtCompileTime == Eigen::Dynamic ||
                  Cols == Eigen::Dynamic);

 public:
    explicit MatOut(Eigen::MatrixBase<T> const& val)
        : base_ptr_((const_cast<Eigen::MatrixBase<T>*>(&val)))
    {
        if constexpr (T::RowsAtCompileTime == Eigen::Dynamic && Rows != Eigen::Dynamic)
        {
            check(val.rows() == Rows);
        }
        if constexpr (T::ColsAtCompileTime == Eigen::Dynamic && Cols != Eigen::Dynamic)
        {
            check(val.cols() == Cols);
        }
    }

    Eigen::MatrixBase<T>& operator*() { return *base_ptr_; }
    Eigen::MatrixBase<T>* operator->() { return base_ptr_; }

 private:
    Eigen::MatrixBase<T>* base_ptr_;
};

template <typename T, std::enable_if_t<detail::is_eigen<T>::value, int> = 0>
MatOut<T, T::RowsAtCompileTime, T::ColsAtCompileTime> make_out(Eigen::MatrixBase<T> const& m)
{
    return MatOut<T, T::RowsAtCompileTime, T::ColsAtCompileTime>(m);
}

template <typename T, std::enable_if_t<!detail::is_eigen<T>::value, int> = 0>
Out<T> make_out(T& val)
{
    return Out<T>(val);
}

}  // namespace mc
