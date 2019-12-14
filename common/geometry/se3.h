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

    Vec3 operator*(const Vec3& v) const { return transformp(v); }
    Vec3 transformp(const Vec3& v) const { return r_ * v + t_; }
    Vec3 transforma(const Vec3& v) const { return r_.inverse() * (v - t_); }

    static SE3 exp(const Vec6& wu)
    {
        auto w = wu.template head<3>();
        auto v = wu.template tail<3>();
        const T th2 = w.squaredNorm();

        T a, b, c;
        if (th2 < 4e-6)
        {
            const T th4 = th2 * th2;
            a = 1 - th2 / 6. + th4 / 120.;
            b = 0.5 - th2 / 24. + th4 / 720.;
            c = 1. / 6. - th2 / 120. + th4 / 5040.;
        }
        else
        {
            const T th = sqrt(th2);
            const T st = sin(th);
            a = st / th;
            b = (1. - cos(th)) / th2;
            c = (1. - a) / th2;
        }

        // There is redudancy here
        const Mat3 sk_w = skew(w);
        const Mat3 R = Mat3::Identity() + a * sk_w + b * sk_w * sk_w;
        const Mat3 V = Mat3::Identity() + b * sk_w + c * sk_w * sk_w;
        return SE3(R, V * v);
    }

    Vec6 log() const
    {
        Vec6 wv;
        auto w = wv.template head<3>();
        w = r_.log();

        const T th2 = w.squaredNorm();
        const T th = std::sqrt(th2);
        const T A = std::sin(th) / th;
        const T B = ((T)1. - std::cos(th)) / th2;
        const T C = ((T)1. - A) / th2;

        T D;
        if (th < 1e-4)
        {
            D = (1. / 12.) + th2 * ((1. / 720.) + th2 * (1. / 30240.));
        }
        else
        {
            D = (B * 0.5 - C) / A;
        }

        Vec3 wxt = w.cross(t_);
        Vec3 w2xt = w.cross(wxt);

        wv.template tail<3>() = t_ - (0.5 * wxt) + (D * w2xt);

        return wv;
    }

    static SE3 Random() { return SE3(SO3<T>::Random(), Vec3::Random()); }

    SO3<T> r_;
    Vec3 t_;
};
