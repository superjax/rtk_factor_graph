#pragma once

#include <Eigen/Core>

#include "common/math/dquat.h"
#include "common/matrix_defs.h"
#include "common/utctime.h"

namespace mc {
namespace utils {

struct WGS84
{
 private:
    static constexpr double A = 6378137.0;       // WGS-84 Earth semimajor axis (m)
    static constexpr double B = 6356752.314245;  // Derived Earth semiminor axis (m)
    static constexpr double F = (A - B) / A;     // Ellipsoid Flatness
    static constexpr double F_INV = 1.0 / F;     // Inverse flattening
    static constexpr double A2 = A * A;
    static constexpr double B2 = B * B;
    static constexpr double E2 = F * (2 - F);  // Square of Eccentricity

 public:
    static Eigen::Vector3d ecef2lla(const Eigen::Vector3d& ecef);
    static void ecef2lla(const Eigen::Vector3d& ecef, Eigen::Vector3d* lla);

    static Eigen::Vector3d lla2ecef(const Eigen::Vector3d& lla);
    static void lla2ecef(const Eigen::Vector3d& lla, Eigen::Vector3d* ecef);

    static void dq_ecef2ned(const Eigen::Vector3d& ecef, math::DQuat<double>* dq_e2n);
    static math::DQuat<double> dq_ecef2ned(const Eigen::Vector3d& ecef);

    static Eigen::Vector3d ned2ecef(const math::DQuat<double>& dq_e2n, const Eigen::Vector3d& ned);
    static void ned2ecef(const math::DQuat<double>& dq_e2n,
                         const Eigen::Vector3d& ned,
                         Eigen::Vector3d* ecef);

    static Eigen::Vector3d ecef2ned(const math::DQuat<double>& dq_e2n, const Eigen::Vector3d& ecef);
    static void ecef2ned(const math::DQuat<double>& dq_e2n,
                         const Eigen::Vector3d& ecef,
                         Eigen::Vector3d* ned);

    static void lla2ned(const Eigen::Vector3d& lla0,
                        const Eigen::Vector3d& lla,
                        Eigen::Vector3d* ned);
    static Eigen::Vector3d lla2ned(const Eigen::Vector3d& lla0, const Eigen::Vector3d& lla);

    static void ned2lla(const Eigen::Vector3d& lla0,
                        const Eigen::Vector3d& ned,
                        Eigen::Vector3d* lla);
    static Eigen::Vector3d ned2lla(const Eigen::Vector3d& lla0, const Eigen::Vector3d& ned);

    static math::Quat<double> q_e2n(const Eigen::Vector3d& lla);
};

std::string printLla(const Eigen::Vector3d& lla);
}  // namespace utils
}  // namespace mc
