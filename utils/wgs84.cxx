#include "utils/wgs84.h"

namespace mc {
namespace utils {

constexpr double WGS84::A;
constexpr double WGS84::B;
constexpr double WGS84::F;
constexpr double WGS84::F_INV;
constexpr double WGS84::A2;
constexpr double WGS84::B2;
constexpr double WGS84::E2;

using math::DQuat;
using math::Quat;

Eigen::Vector3d WGS84::ecef2lla(const Eigen::Vector3d& ecef)
{
    Eigen::Vector3d lla;
    ecef2lla(ecef, &lla);
    return lla;
}

void WGS84::ecef2lla(const Eigen::Vector3d& ecef, Eigen::Vector3d* lla)
{
    double r2 = ecef.x() * ecef.x() + ecef.y() * ecef.y();
    double z = ecef.z();
    double v;
    double zk;

    // This is an iterative procedure
    do
    {
        zk = z;
        double sinp = z / std::sqrt(r2 + z * z);
        v = A / std::sqrt(1.0 - E2 * sinp * sinp);
        z = ecef.z() + v * E2 * sinp;
    } while (std::abs(z - zk) >= 1e-4);

    if (r2 > 1e-12)
    {
        lla->x() = std::atan(z / std::sqrt(r2));
        lla->y() = std::atan2(ecef.y(), ecef.x());
    }
    else if (ecef.z() > 0.0)
    {
        lla->x() = M_PI / 2.0;
        lla->y() = 0.0;
    }
    else
    {
        lla->x() = -M_PI / 2.0;
        lla->y() = 0.0;
    }
    lla->z() = std::sqrt(r2 + z * z) - v;
}

Eigen::Vector3d WGS84::lla2ecef(const Eigen::Vector3d& lla)
{
    Eigen::Vector3d ecef;
    lla2ecef(lla, &ecef);
    return ecef;
}

void WGS84::lla2ecef(const Eigen::Vector3d& lla, Eigen::Vector3d* ecef)
{
    double sinp = sin(lla[0]);
    double cosp = cos(lla[0]);
    double sinl = sin(lla[1]);
    double cosl = cos(lla[1]);
    double e2 = F * (2.0 - F);
    double v = A / sqrt(1.0 - e2 * sinp * sinp);

    ecef->x() = (v + lla[2]) * cosp * cosl;
    ecef->y() = (v + lla[2]) * cosp * sinl;
    ecef->z() = (v * (1.0 - e2) + lla[2]) * sinp;
}

void WGS84::dq_ecef2ned(const Eigen::Vector3d& ecef, DQuat<double>* dq_e2n)
{
    *dq_e2n = DQuat<double>(q_e2n(ecef2lla(ecef)), ecef);
}

DQuat<double> WGS84::dq_ecef2ned(const Eigen::Vector3d& ecef)
{
    return DQuat<double>(q_e2n(ecef2lla(ecef)), ecef);
}

Eigen::Vector3d WGS84::ned2ecef(const DQuat<double>& dq_e2n, const Eigen::Vector3d& ned)
{
    return dq_e2n.transforma(ned);
}

void WGS84::ned2ecef(const DQuat<double>& dq_e2n, const Eigen::Vector3d& ned, Eigen::Vector3d* ecef)
{
    *ecef = dq_e2n.transforma(ned);
}

Eigen::Vector3d WGS84::ecef2ned(const DQuat<double>& dq_e2n, const Eigen::Vector3d& ecef)
{
    return dq_e2n.transformp(ecef);
}

void WGS84::ecef2ned(const DQuat<double>& dq_e2n, const Eigen::Vector3d& ecef, Eigen::Vector3d* ned)
{
    *ned = dq_e2n.transformp(ecef);
}

void WGS84::lla2ned(const Eigen::Vector3d& lla0, const Eigen::Vector3d& lla, Eigen::Vector3d* ned)
{
    DQuat<double> dq_e2n(q_e2n(lla0), lla2ecef(lla0));
    ecef2ned(dq_e2n, lla2ecef(lla), ned);
}

Eigen::Vector3d WGS84::lla2ned(const Eigen::Vector3d& lla0, const Eigen::Vector3d& lla)
{
    Eigen::Vector3d ned;
    lla2ned(lla0, lla, &ned);
    return ned;
}

void WGS84::ned2lla(const Eigen::Vector3d& lla0, const Eigen::Vector3d& ned, Eigen::Vector3d* lla)
{
    DQuat<double> dq_e2n = DQuat<double>(q_e2n(lla0), lla2ecef(lla0));
    ecef2lla(ned2ecef(dq_e2n, ned), lla);
}

Eigen::Vector3d WGS84::ned2lla(const Eigen::Vector3d& lla0, const Eigen::Vector3d& ned)
{
    Eigen::Vector3d lla;
    ned2lla(lla0, ned, &lla);
    return lla;
}

Quat<double> WGS84::q_e2n(const Eigen::Vector3d& lla)
{
    Quat<double> q1, q2;
    q1 = Quat<double>::from_axis_angle(e_z, lla(1));
    q2 = Quat<double>::from_axis_angle(e_y, -M_PI / 2.0 - lla(0));
    return q1 * q2;
}

std::string printLla(const Eigen::Vector3d& lla)
{
    std::stringstream ss;
    ss << lla(0) * 180.0 / M_PI << ", " << lla(1) * 180.0 / M_PI << ", " << lla(2);
    return ss.str();
}

}  // namespace utils
}  // namespace mc
