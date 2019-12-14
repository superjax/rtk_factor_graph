#pragma once

#include <Eigen/Core>
#include <iostream>
#include "common/matrix_defs.h"

template <typename T>
class SO3
{
 public:
    using Vec3 = Eigen::Matrix<T, 3, 1>;
    using Mat3 = Eigen::Matrix<T, 3, 3>;

    SO3() = default;
    SO3(const Mat3& rot) : rot_(rot) {}

    static SO3 identity() { return SO3(Mat3::Identity()); }

    SO3 operator*(const SO3& other) const { return SO3(rot_ * other.rot_); }
    Vec3 operator*(const Vec3& v) const { return rot_ * v; }

    SO3 transpose() const { return SO3(rot_.transpose()); }
    SO3 inverse() const { return SO3(rot_.transpose()); }
    SO3& rectify()
    {
        orthonormalize(rot_);
        return *this;
    }

    SO3 rectified() const
    {
        Mat3 orth = rot_;
        orthonormalize(orth);
        return SO3(orth);
    }

    const Mat3& matrix() const { return rot_; }
    Mat3& matrix() { return rot_; }

    static SO3 exp(const Vec3& w)
    {
        const T th2 = w.squaredNorm();

        T a, b;
        if (th2 < 4e-6)
        {
            const T th4 = th2 * th2;
            a = 1 - 1. / 6. * th2 + 1. / 120. * th4;
            b = 0.5 - 1. / 24. * th2 + 1. / 720. * th4;
        }
        else
        {
            const T th = sqrt(th2);
            a = sin(th) / th;
            b = (1 - cos(th)) / th2;
        }

        // There is redundant effort here that could be worked out.
        const Mat3 w_skew = skew(w);
        const Mat3 R = Mat3::Identity() + a * w_skew + b * w_skew * w_skew;
        return SO3(R);
    }

    Vec3 log() const
    {
        // https://math.stackexchange.com/questions/83874/efficient-and-accurate-numerical-implementation-of-the-inverse-rodrigues-rotatio
        // and the quat.h file for the third case, because ^ doesn't always work
        const T t = rot_.trace();
        static constexpr T eps = 1e-8;

        // clang-format off
        const Vec3 r(rot_(2,1) - rot_(1,2),
                     rot_(0,2) - rot_(2,0),
                     rot_(1,0) - rot_(0,1));
        // clang-format on

        if (t >= 3.0 - eps)  // Theta close to zero
        {
            return (1. / 2. - (t - 3.) / (12.)) * r;
        }
        else if (3. - eps > t && t > -1. + eps)
        {
            const T theta = std::acos((t - 1.) / 2.);
            return (theta / (2. * std::sin(theta))) * r;
        }
        else  // theta close to Pi
        {
            // First, go to quaternion, then generate the rotation matrix
            Vec4 qarr;
            const Mat3& m(rot_);
            if ((m(0, 0) > m(1, 1)) && (m(0, 0) > m(2, 2)))
            {
                T S = sqrt(1.0 + m(0, 0) - m(1, 1) - m(2, 2)) * 2.;
                qarr << (m(1, 2) - m(2, 1)) / S, 0.25 * S, (m(1, 0) + m(0, 1)) / S,
                    (m(2, 0) + m(0, 2)) / S;
            }
            else if (m(1, 1) > m(2, 2))
            {
                T S = sqrt(1.0 + m(1, 1) - m(0, 0) - m(2, 2)) * 2.;
                qarr << (m(2, 0) - m(0, 2)) / S, (m(1, 0) + m(0, 1)) / S, 0.25 * S,
                    (m(2, 1) + m(1, 2)) / S;
            }
            else
            {
                T S = sqrt(1.0 + m(2, 2) - m(0, 0) - m(1, 1)) * 2.;
                qarr << (m(0, 1) - m(1, 0)) / S, (m(2, 0) + m(0, 2)) / S, (m(2, 1) + m(1, 2)) / S,
                    0.25 * S;
            }
            auto v = qarr.tail<3>();
            T& w = qarr[0];
            T norm_v = v.norm();
            return 2.0 * std::atan2(norm_v, w) * v / norm_v;
        }
    }

    SO3 times(const SO3& other) const { return SO3(rot_ * other.rot_); }

    static SO3 Random()
    {
        Vec3 w;
        w.setRandom();
        return SO3::exp(w);
    }

    Vec3 rotp(const Vec3& v) const { return rot_ * v; }
    Vec3 rota(const Vec3& v) const { return rot_.transpose() * v; }

    static SO3 from_axis_angle(const Vec3& axis, const T angle)
    {
        return exp(-axis.normalized() * angle);
    }

    static SO3 from_two_unit_vectors(const Vec3& _u1, const Vec3& _u2)
    {
        const Vec3 u1 = _u1.normalized();
        const Vec3 u2 = _u2.normalized();

        const Vec3 axis = u1.cross(u2);
        const T c = u1.dot(u2);  // cosine of angle

        return exp(axis.normalized() * acos(c));
    }

 private:
    static void orthonormalize(Mat3& mat)
    {
        // Gram-Schmidt would be faster, but this is easy
        static Eigen::JacobiSVD<Mat3> solver(3, 3, Eigen::ComputeFullU | Eigen::ComputeFullV);
        solver.compute(mat);
        mat = solver.matrixU() * solver.matrixV().transpose();
    }
    Mat3 rot_;
};
