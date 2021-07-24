#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

#include "common/defs.h"
#include "common/math/so3.h"
#include "common/matrix_defs.h"

namespace mc {
namespace math {

template <typename T>
class SO3;

template <typename T = double>
class Quat
{
 private:
    typedef Eigen::Matrix<T, 4, 1> Vec4;
    typedef Eigen::Matrix<T, 3, 1> Vec3;
    typedef Eigen::Matrix<T, 3, 3> Mat3;
    alignas(64) T buf_[4];

 public:
    static constexpr int DOF = 3;
    using Scalar = T;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Quat() : buf_{1, 0, 0, 0}, arr_(buf_) {}

    Quat(const T& w, const T& x, const T& y, const T& z) : buf_{w, x, y, z}, arr_(buf_) {}

    Quat(const Eigen::Ref<const Vec4> arr) : arr_(const_cast<T*>(arr.data())) {}

    Quat(const Quat& q) : arr_(buf_) { arr_ = q.arr_; }

    Quat(const T* data) : arr_(const_cast<T*>(data)) {}

    Quat(const SO3<T>& rot) { this = Quat<T>::from_R(rot); }

    Quat(const T& roll, const T& pitch, const T& yaw) : arr_(buf_)
    {
        const T cp = cos(roll / 2.0);
        const T ct = cos(pitch / 2.0);
        const T cs = cos(yaw / 2.0);
        const T sp = sin(roll / 2.0);
        const T st = sin(pitch / 2.0);
        const T ss = sin(yaw / 2.0);

        // clang-format off
        arr_ << cp * ct * cs + sp * st * ss,
                sp * ct * cs - cp * st * ss,
                cp * st * cs + sp * ct * ss,
                cp * ct * ss - sp * st * cs;
        // clang-format on
    }

    inline T* data() { return arr_.data(); }

    Eigen::Map<Vec4> arr_;

    inline T& operator[](int i) { return arr_[i]; }
    inline const T& operator[](int i) const { return arr_[i]; }
    inline T& w() { return arr_(0); }
    inline T& x() { return arr_(1); }
    inline T& y() { return arr_(2); }
    inline T& z() { return arr_(3); }
    inline const T& w() const { return arr_(0); }
    inline const T& x() const { return arr_(1); }
    inline const T& y() const { return arr_(2); }
    inline const T& z() const { return arr_(3); }
    inline const Vec4 elements() const { return arr_; }
    inline const T* data() const { return arr_.data(); }

    Quat operator+(const Quat& q2) const
    {
        Quat q3;
        q3.arr_ = arr_ + q2.arr_;
        return q3;
    }
    Quat operator-(const Quat& q2) const
    {
        Quat q3;
        q3.arr_ = arr_ - q2.arr_;
        return q3;
    }
    Quat operator*(const T& s) const
    {
        Quat q3;
        q3.arr_ = s * arr_;
        return q3;
    }

    template <typename T2>
    Quat operator*(const Quat<T2>& q) const
    {
        return otimes(q);
    }
    Quat& operator*=(const Quat& q)
    {
        const T _w = w();
        const T _x = x();
        const T _y = y();
        const T _z = z();

        // clang-format off
        arr_ << _w * q.w() - _x * q.x() - _y * q.y() - _z * q.z(),
                _w * q.x() + _x * q.w() + _y * q.z() - _z * q.y(),
                _w * q.y() - _x * q.z() + _y * q.w() + _z * q.x(),
                _w * q.z() + _x * q.y() - _y * q.x() + _z * q.w();
        // clang-format on
        return *this;
    }

    template <typename Tout = T, typename T2>
    Quat<Tout> otimes(const Quat<T2>& q) const
    {
        Quat<Tout> qout;
        // clang-format off
        qout.arr_ << w() * q.w() - x() * q.x() - y() * q.y() - z() * q.z(),
                     w() * q.x() + x() * q.w() + y() * q.z() - z() * q.y(),
                     w() * q.y() - x() * q.z() + y() * q.w() + z() * q.x(),
                     w() * q.z() + x() * q.y() - y() * q.x() + z() * q.w();
        // clang-format on
        return qout;
    }

    bool operator==(const Quat& other) const { return (arr_ == other.arr_); }

    void setRandom()
    {
        arr_.setRandom();
        normalize();
    }

    Quat& operator=(const Quat& q)
    {
        arr_ = q.elements();
        return *this;
    }

    template <typename Derived>
    Quat& operator=(Eigen::MatrixBase<Derived> const& in)
    {
        arr_ = in;
        return *this;
    }

    template <typename T2>
    Quat<T2> cast() const
    {
        Quat<T2> q;
        q.arr_ = arr_.template cast<T2>();
        return q;
    }

    static Quat exp(const Vec3& v)
    {
        const T norm_v = v.norm();

        if (norm_v > (T)4e-6)
        {
            const T v_scale = sin(norm_v / 2.0) / norm_v;
            return Quat(cos(norm_v / 2.0), v_scale * v(0), v_scale * v(1), v_scale * v(2));
        }
        else
        {
            // Use Taylor series
            const T norm_v2 = norm_v * norm_v;
            const T norm_v4 = norm_v2 * norm_v2;
            const T v_scale = 0.5 - norm_v2 / 48.0 + norm_v4 / 3840.0;
            Quat q(cos(norm_v / 2.0), v_scale * v(0), v_scale * v(1), v_scale * v(2));
            q.normalize();
            return q;
        }
    }

    template <JacobianSide SIDE = JacobianSide::LEFT>
    static Quat exp(const Vec3& w, Mat3* jac)
    {
        const T th2 = w.squaredNorm();
        const T th = w.norm();

        T a, b, c, s;
        if (th > (T)4e-6)
        {
            a = sin(th) / th;
            b = (1 - cos(th)) / th2;
            c = (1 - a) / th2;
            s = sin(th / 2.0) / th;
        }
        else
        {
            // Use Taylor series
            const T th4 = th2 * th2;
            a = 1. - th2 / 6. + th4 / 120.;
            b = 1. / 2. - th2 / 24. + th4 / 720.;
            c = 1. / 6. - th2 / 120. + th4 / 5040.;
            s = 0.5 - th2 / 48.0 + th4 / 3840.0;
        }

        Quat q(cos(th / 2.0), s * w(0), s * w(1), s * w(2));
        q.normalize();

        if constexpr (SIDE == JacobianSide::LEFT)
        {
            *jac = a * Mat3::Identity() + b * skew(w) + c * w * w.transpose();
        }
        else
        {
            *jac = a * Mat3::Identity() - b * skew(w) + c * w * w.transpose();
        }

        return q;
    }

    Vec3 log() const
    {
        const auto v = arr_.template block<3, 1>(1, 0);
        const T norm_v = v.norm();

        if (norm_v > (T)4e-6)
        {
            return (T)2.0 * atan2(norm_v, w()) * v / norm_v;
        }
        else
        {
            // Use Taylor series of arctan(x/w)/norm_v
            const T norm_v2 = norm_v * norm_v;
            const T norm_v4 = norm_v2 * norm_v2;
            const T w3 = w() * w() * w();
            const T w5 = w3 * w() * w();
            return (T)2.0 * v * (1. / w() - norm_v2 / (3. * w3) + norm_v4 / (5. * w5));
        }
    }

    template <JacobianSide SIDE = JacobianSide::LEFT>
    Vec3 log(Mat3* jac) const
    {
        const auto v = arr_.template block<3, 1>(1, 0);
        const T norm_v = v.norm();

        T th, e;
        Vec3 r = v / norm_v;
        if (norm_v > (T)4e-6)
        {
            th = 2. * atan2(norm_v, w());
            const T th2 = th * th;
            const T a = sin(th) / th;
            const T b = (1 - cos(th)) / th2;
            const T c = (1 - a) / th2;
            e = (b - 2 * c) / (2 * a);
        }
        else
        {
            const T n2 = norm_v * norm_v;
            const T n3 = n2 * norm_v;
            const T n5 = n3 * n2;
            const T w2 = w() * w();
            const T w3 = w2 * w();
            const T w5 = w3 * w2;
            th = 2. * (norm_v / w() - n3 / (3 * w3) + (n5 / 5 * w5));
            const T th2 = th * th;
            const T th4 = th2 * th2;
            e = 1. / 12. + th2 / 720. + th4 / 30240.;
            if (norm_v == 0)
            {
                // There is no vector, just choose one
                r = Vec3::Unit(0);
            }
        }

        const Vec3 omg = th * r;

        const Mat3 sk_w = skew(omg);
        const Mat3 sk_w2 = sk_w * sk_w;
        if constexpr (SIDE == JacobianSide::LEFT)
        {
            *jac << Mat3::Identity() - 1. / 2. * sk_w + e * sk_w2;
        }
        else
        {
            *jac << Mat3::Identity() + 1. / 2. * sk_w + e * sk_w2;
        }
        return omg;
    }

    static Quat make_pure(const Vec3& v)
    {
        Quat q;
        q.arr_ << 0, v;
        return q;
    }

    static Quat from_R(const Eigen::Matrix<T, 3, 3>& m)
    {
        Quat q;
        T tr = m.trace();

        if (tr > 0)
        {
            T S = sqrt(tr + 1.0) * 2.;
            q.arr_ << 0.25 * S, (m(1, 2) - m(2, 1)) / S, (m(2, 0) - m(0, 2)) / S,
                (m(0, 1) - m(1, 0)) / S;
        }
        else if ((m(0, 0) > m(1, 1)) && (m(0, 0) > m(2, 2)))
        {
            T S = sqrt(1.0 + m(0, 0) - m(1, 1) - m(2, 2)) * 2.;
            q.arr_ << (m(1, 2) - m(2, 1)) / S, 0.25 * S, (m(1, 0) + m(0, 1)) / S,
                (m(2, 0) + m(0, 2)) / S;
        }
        else if (m(1, 1) > m(2, 2))
        {
            T S = sqrt(1.0 + m(1, 1) - m(0, 0) - m(2, 2)) * 2.;
            q.arr_ << (m(2, 0) - m(0, 2)) / S, (m(1, 0) + m(0, 1)) / S, 0.25 * S,
                (m(2, 1) + m(1, 2)) / S;
        }
        else
        {
            T S = sqrt(1.0 + m(2, 2) - m(0, 0) - m(1, 1)) * 2.;
            q.arr_ << (m(0, 1) - m(1, 0)) / S, (m(2, 0) + m(0, 2)) / S, (m(2, 1) + m(1, 2)) / S,
                0.25 * S;
        }
        return q;
    }

    static Quat from_axis_angle(const Vec3& axis, const T angle)
    {
        T alpha_2 = angle / 2.0;
        T sin_a2 = sin(alpha_2);
        Quat out;
        out.arr_(0) = cos(alpha_2);
        out.arr_(1) = axis(0) * sin_a2;
        out.arr_(2) = axis(1) * sin_a2;
        out.arr_(3) = axis(2) * sin_a2;
        out.arr_ /= out.arr_.norm();
        return out;
    }

    static Quat from_euler(const T& roll, const T& pitch, const T& yaw)
    {
        T cp = cos(roll / 2.0);
        T ct = cos(pitch / 2.0);
        T cs = cos(yaw / 2.0);
        T sp = sin(roll / 2.0);
        T st = sin(pitch / 2.0);
        T ss = sin(yaw / 2.0);

        Quat q;
        q.arr_ << cp * ct * cs + sp * st * ss, sp * ct * cs - cp * st * ss,
            cp * st * cs + sp * ct * ss, cp * ct * ss - sp * st * cs;
        return q;
    }

    static Quat from_two_unit_vectors(const Vec3& u, const Vec3& v)
    {
        const T cos_theta = u.dot(v);
        const Vec3 axis = u.cross(v).normalized();
        return from_axis_angle(axis, -acos(cos_theta));
    }

    static Quat Identity()
    {
        Quat q;
        q.arr_ << 1.0, 0, 0, 0;
        return q;
    }

    static Quat Random()
    {
        Quat q_out;
        q_out.arr_.setRandom();
        q_out.arr_ /= q_out.arr_.norm();
        return q_out;
    }

    T roll() const
    {
        return atan2(T(2.0) * (w() * x() + y() * z()), T(1.0) - T(2.0) * (x() * x() + y() * y()));
    }

    T pitch() const
    {
        const T val = T(2.0) * (w() * y() - x() * z());

        // hold at 90 degrees if invalid
        if (fabs(val) > T(1.0))
            return copysign(T(1.0), val) * T(M_PI) / T(2.0);
        else
            return asin(val);
    }

    T yaw() const
    {
        return atan2(T(2.0) * (w() * z() + x() * y()), T(1.0) - T(2.0) * (y() * y() + z() * z()));
    }

    Vec3 euler() const
    {
        Vec3 out;
        out << roll(), pitch(), yaw();
        return out;
    }

    Eigen::VectorBlock<const Eigen::Map<Vec4>, 3> bar() const
    {
        return arr_.template segment<3>(1);
    }
    Eigen::VectorBlock<Eigen::Map<Vec4>, 3> bar() { return arr_.template segment<3>(1); }

    SO3<T> R() const
    {
        T wx = w() * x();
        T wy = w() * y();
        T wz = w() * z();
        T xx = x() * x();
        T xy = x() * y();
        T xz = x() * z();
        T yy = y() * y();
        T yz = y() * z();
        T zz = z() * z();
        SO3<T> out;
        // clang-format off
        out.matrix() << 1. - 2. * yy - 2. * zz, 2. * xy + 2. * wz,      2. * xz - 2. * wy,
                        2. * xy - 2. * wz,      1. - 2. * xx - 2. * zz, 2. * yz + 2. * wx,
                        2. * xz + 2. * wy,      2. * yz - 2. * wx,      1. - 2. * xx - 2. * yy;
        // clang-format on
        return out;
    }

    Quat copy() const
    {
        Quat tmp;
        tmp.arr_ = arr_;
        return tmp;
    }

    void normalize() { arr_ /= arr_.norm(); }

    Eigen::Matrix<T, 3, 2> doublerota(const Eigen::Matrix<T, 3, 2>& v) const
    {
        Eigen::Matrix<T, 3, 2> out(3, 2);
        Vec3 t;
        for (int i = 0; i < 2; ++i)
        {
            t = 2.0 * v.col(i).cross(bar());
            out.col(i) = v.col(i) - w() * t + t.cross(bar());
        }
        return out;
    }

    Eigen::Matrix<T, 3, 2> doublerotp(const Eigen::Matrix<T, 3, 2>& v) const
    {
        Eigen::Matrix<T, 3, 2> out(3, 2);
        Vec3 t;
        for (int i = 0; i < 2; ++i)
        {
            t = 2.0 * v.col(i).cross(bar());
            out.col(i) = v.col(i) + w() * t + t.cross(bar());
        }
        return out;
    }

    // template <typename Derived3>
    // static cross(const Vec3& x, const Eigen::MatrixBase<Derived3> m)
    // {
    // }

    // The same as R.T * v but faster
    template <typename Derived3>
    Eigen::Matrix<T, 3, Derived3::ColsAtCompileTime> rota(
        const Eigen::MatrixBase<Derived3>& v) const
    {
        Eigen::Matrix<T, 3, Derived3::ColsAtCompileTime> t =
            (T)2.0 * v.colwise().cross(arr_.template segment<3>(1));
        return v - w() * t + t.colwise().cross(arr_.template segment<3>(1));
    }

    // The same as R * v but faster
    template <typename Derived3>
    Eigen::Matrix<T, 3, Derived3::ColsAtCompileTime> rotp(
        const Eigen::MatrixBase<Derived3>& v) const
    {
        Eigen::Matrix<T, 3, Derived3::ColsAtCompileTime> t =
            (T)2.0 * v.colwise().cross(arr_.template segment<3>(1));
        return v + w() * t + t.colwise().cross(arr_.template segment<3>(1));
    }

    Quat& invert() { arr_.template block<3, 1>(1, 0) *= (T)-1.0; }

    Quat inverse() const
    {
        Quat tmp;
        tmp.arr_(0) = arr_(0);
        tmp.arr_(1) = -arr_(1);
        tmp.arr_(2) = -arr_(2);
        tmp.arr_(3) = -arr_(3);
        return tmp;
    }

    Mat3 Ad() const { return R().transpose(); }

    inline T norm() const { return arr_.norm(); }

    Eigen::Matrix<double, 4, 3> dParamDGen() const
    {
        // clang-format off
        Eigen::Matrix<double, 4, 3> dadq;
        dadq << -x()/2.0, -y()/2.0, -z()/2.0,
                 w()/2.0, -z()/2.0,  y()/2.0,
                 z()/2.0,  w()/2.0, -x()/2.0,
                -y()/2.0,  x()/2.0,  w()/2.0;
        // clang-format on
        return dadq;
    }

    Eigen::Matrix<double, 3, 4> dGenDParam() const
    {
        // clang-format off
        Eigen::Matrix<double, 3, 4> dadq;
        dadq << -x()*2.0,  w()*2.0,  z()*2.0, -y()*2.0,
                -y()*2.0, -z()*2.0,  w()*2.0,  x()*2.0,
                -z()*2.0,  y()*2.0, -x()*2.0,  w()*2.0;
        // clang-format on
        return dadq;
    }
};

// // Specialized double-versions of rotation
// template <>
// Vec3 Quat<double>::rotp(const Vec3& v) const
// {
//     // clang-format off
//         const double qvw = x() * v.x() + y() * v.y() + z() * v.z();
//         const double qvx = w() * v.x() - y() * v.z() + z() * v.y();
//         const double qvy = w() * v.y() + x() * v.z() - z() * v.x();
//         const double qvz = w() * v.z() - x() * v.y() + y() * v.x();

//         Vec3 out(qvw * x() + qvx * w() + qvy * z() - qvz * y(),
//                  qvw * y() - qvx * z() + qvy * w() + qvz * x(),
//                  qvw * z() + qvx * y() - qvy * x() + qvz * w());
//     // clang-format on
//     return out;
// }

// template <>
// Vec3 Quat<double>::rota(const Vec3& v) const
// {
//     // clang-format off
//         const double qvw = -x() * v.x() - y() * v.y() - z() * v.z();
//         const double qvx =  w() * v.x() + y() * v.z() - z() * v.y();
//         const double qvy =  w() * v.y() - x() * v.z() + z() * v.x();
//         const double qvz =  w() * v.z() + x() * v.y() - y() * v.x() ;

//         Vec3 out(-qvw * x() + qvx * w() - qvy * z() + qvz * y(),
//                  -qvw * y() + qvx * z() + qvy * w() - qvz * x(),
//                  -qvw * z() - qvx * y() + qvy * x() + qvz * w());
//     // clang-format on
//     return out;
// }

template <typename T>
inline std::ostream& operator<<(std::ostream& os, const Quat<T>& q)
{
    os << "[ " << q.w() << ", " << q.x() << "i, " << q.y() << "j, " << q.z() << "k]";
    return os;
}

template <typename T>
Quat<T> operator*(const T& s, const Quat<T>& q)
{
    Quat<T> q2;
    q2.arr_ = q.arr_ * s;
    return q2;
}

typedef Quat<double> Quatd;

}  // namespace math

namespace detail {
template <typename T>
struct is_lie_group<math::Quat<T>>
{
    static constexpr std::true_type value = std::true_type();
};

}  // namespace detail

template <typename T>
bool isFinite(const math::Quat<T> x)
{
    return isFinite(x.arr_);
}

}  // namespace mc
