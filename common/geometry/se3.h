#pragma once

#include <iostream>

#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

#include "common/geometry/so3.h"
#include "common/matrix_defs.h"

template <typename T>
class SE3
{
    using Vec4 = Eigen::Matrix<T, 4, 1>;
    using Mat4 = Eigen::Matrix<T, 4, 4>;
    using Vec3 = Eigen::Matrix<T, 3, 1>;
    using Mat3 = Eigen::Matrix<T, 3, 3>;
    using Vec6 = Eigen::Matrix<T, 6, 1>;

 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SE3()
    {
        r_.matrix().setIdentity();
        t_.setZero();
    }

    SE3(const Mat4& arr) : r_(arr.template block<3, 3>(0, 0)), t_(arr.template block<3, 1>(0, 3)) {}
    SE3(const SO3<T>& rot, const Vec3& t) : r_(rot), t_(t) {}
    SE3(const Mat3& rot, const Vec3& t) : r_(rot), t_(t) {}

    static SE3 identity() { return SE3(SO3<T>::identity(), Vec3::Zero()); }

    SO3<T>& rotation() { return r_; }
    const SO3<T>& rotation() const { return r_; }
    SO3<T>& R() { return r_; }
    const SO3<T>& R() const { return r_; }
    Vec3& translation() { return t_; }
    const Vec3& translation() const { return t_; }
    Vec3& t() { return t_; }
    const Vec3& t() const { return t_; }

    SE3 operator*(const SE3& other) const { return SE3(r_ * other.r_, (r_ * other.t_) + t_); }
    SE3 inverse() const
    {
        const SO3<T> Rinv = r_.inverse();
        return SE3(Rinv, -(Rinv * t_));
    }

    SE3 rectified() const { return SE3(r_.rectified(), t_); }
    SE3& rectify()
    {
        r_.rectify();
        return *this;
    }

    Mat4 matrix() const
    {
        Mat4 mat;
        mat.template block<3, 3>(0, 0) = r_.matrix();
        mat.template block<3, 1>(0, 3) = t_;
        mat.template block<1, 3>(3, 0).setZero();
        mat(3, 3) = (T)1.0;
        return mat;
    }

    static SE3 exp(const Vec6& wu)
    {
        auto w = wu.template head<3>();
        auto v = wu.template tail<3>();
        const double th2 = w.squaredNorm();
        const double th = std::sqrt(th2);
        const double A = std::sin(th) / th;
        const double B = ((T)1. - std::cos(th)) / th2;
        const double C = ((T)1. - A) / th2;
        Mat3 R = Mat3::Identity();
        Mat3 V = Mat3::Identity();
        Mat3 sk_w = skew(w);
        if (th > 1e-8)
        {
            R += A * sk_w + B * sk_w * sk_w;
            V += B * sk_w + C * sk_w * sk_w;
        }
        return SE3(R, V * v);
    }

    Vec6 log() const
    {
        Vec6 wv;
        auto w = wv.template head<3>();
        w = r_.log();

        const double th2 = w.squaredNorm();
        const double th = std::sqrt(th2);
        const double A = std::sin(th) / th;
        const double B = ((T)1. - std::cos(th)) / th2;
        const double C = ((T)1. - A) / th2;

        double D;
        if (th < 1e-4)
        {
            D = (1. / 12.) + th2 * ((1. / 720.) + th2 * (1. / 30240.));
        }
        // else if (th > M_PI - 1e-4)
        // {
        //     D = (B - (0.5 * A)) / (B * th2);
        // }
        else
        {
            D = (B * 0.5 - C) / A;
        }

        Vec3 wxt = w.cross(t_);
        Vec3 w2xt = w.cross(wxt);

        wv.template tail<3>() = t_ - (0.5 * wxt) + (D * w2xt);

        return wv;
    }

    SE3 operator+(const Vec6& v) const { return boxplus(v); }
    SE3 boxplus(const Vec6& v) const { return SE3::exp(v) * (*this); }
    Vec6 operator-(const SE3& T2) const { return boxminus(T2); }
    Vec6 boxminus(const SE3& T2) const { return ((*this) * T2.inverse()).log(); }

    static SE3 Random() { return SE3(SO3<T>::Random(), Vec3::Random()); }

    SO3<T> r_;
    Vec3 t_;
};
