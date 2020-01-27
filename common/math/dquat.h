#pragma once

#include <iostream>

#include <Eigen/Core>

#include "common/math/quat.h"
#include "common/matrix_defs.h"
#include "common/print.h"

namespace mc {
namespace math {

template <typename T>
class DQuat
{
    using Vec4 = Eigen::Matrix<T, 4, 1>;
    using Mat4 = Eigen::Matrix<T, 4, 4>;
    using Vec3 = Eigen::Matrix<T, 3, 1>;
    using Mat3 = Eigen::Matrix<T, 3, 3>;
    using Vec6 = Eigen::Matrix<T, 6, 1>;
    using Mat6 = Eigen::Matrix<T, 6, 6>;
    using Vec8 = Eigen::Matrix<T, 8, 1>;

    T buf_[8];

 public:
    static constexpr int DOF = 6;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Map<Vec8> arr_;
    Quat<T> r_;
    Quat<T> d_;

    DQuat() : DQuat(buf_)
    {
        arr_.setZero();
        arr_[0] = (T)1;
    }
    DQuat(const T* data) : arr_(const_cast<T*>(data)), r_(arr_.data()), d_(arr_.data() + 4) {}

    DQuat(const T& rw,
          const T& rx,
          const T& ry,
          const T& rz,
          const T& dw,
          const T& dx,
          const T& dy,
          const T& dz)
        : buf_{rw, rx, ry, rz, dw, dx, dy, dz}, arr_(buf_)
    {
    }

    DQuat(const Eigen::Ref<const Vec8> arr) : DQuat(arr.data()) {}
    DQuat(const DQuat& q) : DQuat(buf_) { arr_ = q.arr_; }
    DQuat(const Quat<T>& r, const Quat<T>& d) : DQuat(buf_)
    {
        r_ = r;
        d_ = d;
    }
    DQuat(const Quat<T>& q, const Vec3& t) : DQuat(buf_)
    {
        r_ = q;
        d_ = (T)0.5 * (Quat<T>::make_pure(t) * q);
    }

    T* data() { return arr_.data(); }
    const T* data() const { return arr_.data(); }

    T& operator[](int i) { return arr_[i]; }
    const T& operator[](int i) const { return arr_[i]; }
    Quat<T>& real() { return r_; }
    const Quat<T>& real() const { return r_; }
    Quat<T>& dual() { return d_; }
    const Quat<T>& dual() const { return d_; }
    void setReal(const Quat<T>& q) { r_ = q; }
    void setDual(const Quat<T>& q) { d_ = q; }

    DQuat operator*(const DQuat& q) const { return otimes(q); }
    DQuat& operator=(const Quat<T>& q)
    {
        arr_ = q.arr_;
        return *this;
    }

    template <typename Derived>
    Quat<T>& operator=(Eigen::MatrixBase<Derived> const& in)
    {
        arr_ = in;
        return *this;
    }

    DQuat otimes(const DQuat& q) const
    {
        DQuat out;
        out.setReal(r_ * q.real());
        out.setDual(r_ * q.dual() + d_ * q.real());
        return out;
    }

    Vec3 translation() const { return (T)2.0 * (d_ * r_.inverse()).bar(); }
    const Quat<T>& rotation() const { return r_; }

    Vec3 transformp(const Vec3& v) const { return r_.rotp(v - translation()); }
    Vec3 transforma(const Vec3& v) const { return r_.rota(v) + translation(); }

    static DQuat Random() { return DQuat(Quat<T>::Random(), Vec3::Random()); }
    static DQuat identity() { return DQuat(); }
    static DQuat from_4x4(const Mat4& m)
    {
        Quat<T> q = Quat<T>::from_R(m.template block<3, 3>(0, 0));
        return DQuat(q, q.rota(m.template block<3, 1>(0, 3)));
    }

    DQuat inverse() const { return DQuat(real().inverse(), dual().inverse()); }

    static DQuat exp(const Vec6& wv)
    {
        const auto w = wv.template head<3>();
        const auto v = wv.template tail<3>();

        const T th2 = w.squaredNorm();
        const T th = sqrt(th2);
        const T gm = w.dot(v);
        const T ct = cos(th / 2.0);
        T sinct, gross;

        if (th2 > (T)4e-6)
        {
            sinct = sin(th / 2.0) / th;
            gross = gm * ((0.5 * ct - sinct) / th2);
        }
        else
        {
            // Use taylor series
            const T th4 = th2 * th2;
            sinct = 0.5 - 1. / 48. * th2 + 1. / 3840. * th4;
            gross = gm * (-1. / 24. + 1. / 960. * th2 - 1. / 107520. * th4);
        }

        DQuat Q;
        Q.r_.arr_ << ct, sinct * w;
        Q.d_.arr_ << -gm / 2.0 * sinct, (gross * w + sinct * v);
        return Q;
    }

    static DQuat exp(const Vec6& wv, Mat6* jac)
    {
        const auto w = wv.template head<3>();
        const auto v = wv.template tail<3>();

        const T th2 = w.squaredNorm();
        const T th = sqrt(th2);
        const T gm = w.dot(v);
        const T ct = cos(th / 2.0);
        T sinct, gross;

        T a, b, c;
        T C1, C2;
        Mat3 A;
        Quat<T>::exp(w, &A);
        if (th2 > (T)4e-6)
        {
            sinct = sin(th / 2.0) / th;
            gross = gm * ((0.5 * ct - sinct) / th2);
            a = sin(th) / th;
            b = (1 - cos(th)) / th2;
            c = (1 - a) / th2;
            C1 = (a - 2 * b) / th2;
            C2 = (b - 3 * c) / th2;
        }
        else
        {
            // Use taylor series
            const T th4 = th2 * th2;
            sinct = 0.5 - 1. / 48. * th2 + 1. / 3840. * th4;
            gross = gm * (-1. / 24. + 1. / 960. * th2 - 1. / 107520. * th4);
            a = 1 - th2 / 6. + th4 / 120.;
            b = 0.5 - th2 / 24. + th4 / 720.;
            c = 1. / 6. - th2 / 120. + th4 / 5040.;
            C1 = -1. / 12. + th2 / 180. - th4 / 6720.;    // (a-2b)/th2
            C2 = -1. / 60. + th2 / 1260. - th4 / 60480.;  // (b-3c)/th2
        }

        DQuat Q;
        Q.r_.arr_ << ct, sinct * w;
        Q.d_.arr_ << -gm / 2.0 * sinct, (gross * w + sinct * v);

        const Mat3 B = w * v.transpose() + v * w.transpose();
        const Mat3 sk_w = skew(w);
        const Mat3 C = (c - b) * Mat3::Identity() + C1 * sk_w + C2 * w * w.transpose();
        const Mat3 D = b * skew(v) + c * B + gm * C;

        *jac << A, Mat3::Zero(), D, A;

        return Q;
    }

    Vec6 log() const
    {
        const auto r = real().bar();
        const auto d = dual().bar();
        const auto r0 = real().w();
        const auto d0 = dual().w();

        const T th2 = r.squaredNorm();
        const T gm = r.dot(d);

        Vec6 wv;
        if (th2 > (T)1e-8)
        {
            const T th = sqrt(th2);
            const T a = atan2(th, r0) / th;
            const T b = gm / th2;
            // clang-format off
            wv << (T)2.0 * a * r,
                  (T)2.0 * (a * d + ((r0 - a) * b - d0) * r);
            // clang-format on
        }
        else
        {
            const T r02 = r0 * r0;
            const T r03 = r02 * r0;
            const T r05 = r03 * r02;
            const T th4 = th2 * th2;
            const T th6 = th4 * th2;
            const T a = 1. - 1. / (3. * r03) * th2 + 1. / (5. * r05) * th4;
            const T c =
                5. / 24. - 379. / 1920. * th2 + 46073. / 322560. * th4 - 127431. / 1146880. * th6;
            // clang-format off
            wv << (T)2.0 * a * r,
                  (T)2.0 * (a * d + (gm*c - d0)*r);
            // clang-format on
        }
        return wv;
    }

    Vec6 log(Mat6* jac) const
    {
        const auto r = real().bar();
        const auto d = dual().bar();
        const auto r0 = real().w();
        const auto d0 = dual().w();

        const T s2 = r.squaredNorm();
        const T gm = r.dot(d);

        Vec6 wv;
        auto w = wv.template head<3>();
        auto v = wv.template tail<3>();
        Mat3 Ainv;
        w = r_.log(&Ainv);

        const T th2 = w.squaredNorm();
        const T th = sqrt(th2);

        T a, b, c;
        T C1, C2;
        if (th2 > (T)4e-6)
        {
            const T s = sqrt(s2);
            const T a1 = atan2(s, r0) / s;
            const T b1 = gm / s2;
            v = (T)2.0 * (a1 * d + ((r0 - a1) * b1 - d0) * r);

            a = sin(th) / th;
            b = ((T)1. - cos(th)) / th2;
            c = ((T)1. - a) / th2;
            C1 = (a - 2 * b) / th2;
            C2 = (b - 3 * c) / th2;
        }
        else
        {
            const T r02 = r0 * r0;
            const T r03 = r02 * r0;
            const T r05 = r03 * r02;
            const T s4 = s2 * s2;
            const T s6 = s4 * s2;
            const T a1 = 1. - 1. / (3. * r03) * s2 + 1. / (5. * r05) * s4;
            const T c1 =
                5. / 24. - 379. / 1920. * s2 + 46073. / 322560. * s4 - 127431. / 1146880. * s6;
            v = (T)2.0 * (a1 * d + (gm * c1 - d0) * r);

            const double th4 = th2 * th2;
            a = 1 - th2 / 6. + th4 / 120.;
            b = 0.5 - th2 / 24. + th4 / 720.;
            c = 1. / 6. - th2 / 120. + th4 / 5040.;
            C1 = -1. / 12. + th2 / 180. - th4 / 6720.;    // (a-2b)/th2
            C2 = -1. / 60. + th2 / 1260. - th4 / 60480.;  // (b-3c)/th2
        }

        const T gm2 = v.dot(w);
        const Mat3 B = w * v.transpose() + v * w.transpose();
        const Mat3 sk_w = skew(w);
        const Mat3 C = (c - b) * Mat3::Identity() + C1 * sk_w + C2 * w * w.transpose();
        const Mat3 D = b * skew(v) + c * B + gm2 * C;

        *jac << Ainv, Mat3::Zero(), -Ainv * D * Ainv, Ainv;

        return wv;
    }

    Mat6 Ad() const
    {
        const Mat3 RT = r_.R().transpose();
        Mat6 A;
        A.template topLeftCorner<3, 3>() = RT;
        A.template topRightCorner<3, 3>().setZero();
        A.template bottomLeftCorner<3, 3>() = skew(translation()) * RT;
        A.template bottomRightCorner<3, 3>() = RT;
        return A;
    }

    SO3<T> R() const { return r_.R(); }
};

template <typename T>
inline std::ostream& operator<<(std::ostream& os, const DQuat<T>& q)
{
    os << "[ " << q.real().w() << ", " << q.real().x() << "i, " << q.real().y() << "j, "
       << q.real().z() << "k] + ϵ[ " << q.dual().w() << ", " << q.dual().x() << "i, "
       << q.dual().y() << "j, " << q.dual().z() << "k]";
    return os;
}

}  // namespace math
}  // namespace mc
