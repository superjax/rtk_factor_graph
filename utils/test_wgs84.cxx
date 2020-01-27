#include <gtest/gtest.h>

#include "utils/wgs84.h"

#include "common/defs.h"
#include "common/matrix_defs.h"
#include "common/test_helpers.h"

namespace mc {
namespace utils {

TEST(WGS84, lla2ecef)
{
    Vec3 lla = {deg2Rad(40.246184), -deg2Rad(111.647769), 1387.997511};  // BYU Campus
    Vec3 ecef_known = {-1798810.23, -4532232.54, 4099784.74};

    std::cout << printLla(lla) << std::endl;

    Vec3 ecef_calc = WGS84::lla2ecef(lla);

    MATRIX_CLOSE(ecef_known, ecef_calc, 1e-2);
}

TEST(WGS84, ecef2lla)
{
    Vec3 ecef = {-1798810.23, -4532232.54, 4099784.74};
    Vec3 lla_known = {deg2Rad(40.246184), -deg2Rad(111.647769), 1387.998309};

    Vec3 lla_calc = WGS84::ecef2lla(ecef);

    MATRIX_CLOSE(lla_known, lla_calc, 1e-6);
}

TEST(WGS84, ecef2lla2ecef)
{
    Vec3 ecef = {-1798810.23, -4532232.54, 4099784.74};
    Vec3 lla_known = {deg2Rad(40.246184), -deg2Rad(111.647769), 1387.998309};

    Vec3 lla_calc = WGS84::ecef2lla(ecef);
    Vec3 ecef_calc = WGS84::lla2ecef(lla_calc);

    MATRIX_CLOSE(ecef_calc, ecef, 1e-6);
    MATRIX_CLOSE(lla_known, lla_calc, 1e-6);
}

TEST(WGS84, ecef2lla_singularity)
{
    const std::array<Vec3, 2> lla = {Vec3{M_PI / 2.0, 0, 0}, Vec3{-M_PI / 2.0, 0, 0}};

    for (const auto& test_lla : lla)
    {
        const Vec3 ecef = WGS84::lla2ecef(test_lla);
        const Vec3 lla_calc = WGS84::ecef2lla(ecef);
        MATRIX_CLOSE(test_lla, lla_calc, 1e-6);
    }
}

TEST(WGS84, x_ned2ecef)
{
    Vec3 lla0 = {deg2Rad(40.247082), -deg2Rad(111.647776), 1387.998309};
    Vec3 ecef0 = WGS84::lla2ecef(lla0);

    Vec3 ned1 = {-54.976484, 1.276565, 0.000237};
    Vec3 lla1 = {deg2Rad(40.246587), -deg2Rad(111.647761), 1387.998309};
    Vec3 ecef1 = WGS84::lla2ecef(lla1);

    math::DQuat<double> dq_e2n = WGS84::dq_ecef2ned(ecef0);
    Vec3 ecef_hat = dq_e2n.transforma(ned1);
    Vec3 ned1_hat = dq_e2n.transformp(ecef1);

    MATRIX_CLOSE(ecef_hat, ecef1, 1e-6);
    MATRIX_CLOSE(ned1_hat, ned1, 1e-3);

    WGS84::dq_ecef2ned(ecef0, &dq_e2n);
    WGS84::ned2ecef(dq_e2n, ned1, &ecef_hat);
    WGS84::ecef2ned(dq_e2n, ecef1, &ned1_hat);

    MATRIX_CLOSE(ecef_hat, ecef1, 1e-6);
    MATRIX_CLOSE(ned1_hat, ned1, 1e-3);
}

TEST(WGS84, ecef2ned_check_axes)
{
    Vec3 lla0 = {deg2Rad(40.247082), -deg2Rad(111.647776), 1387.998309};
    Vec3 ecef0 = WGS84::lla2ecef(lla0);
    math::DQuat<double> dq_e2n = WGS84::dq_ecef2ned(ecef0);

    double sp = std::sin(lla0(0));
    double cp = std::cos(lla0(0));
    double sl = std::sin(lla0(1));
    double cl = std::cos(lla0(1));

    Mat3 R;
    // clang-format off
    // https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates
    R << -sp * cl, -sl, -cp * cl,
         -sp * sl,  cl, -cp * sl,
          cp,       0,  -sp;
    // clang-format on
    MATRIX_CLOSE(dq_e2n.R().matrix(), R.transpose(), 1e-8);

    Vec3 E_r_N_E = 1.0 * ecef0;
    E_r_N_E /= E_r_N_E.stableNorm();
    Vec3 E_z_N = dq_e2n.real().rota(e_z);
    MATRIX_CLOSE(-1.0 * E_z_N, E_r_N_E, 3e-3);
}

TEST(WGS84, ned2ecef)
{
    Vec3 lla0 = {deg2Rad(40.247082), -deg2Rad(111.647776), 1387.998309};
    Vec3 ecef0 = WGS84::lla2ecef(lla0);

    Vec3 ned1 = {-54.976484, 1.276565, 0.000237};
    Vec3 lla1 = {deg2Rad(40.246587), -deg2Rad(111.647761), 1387.998309};
    Vec3 ecef1 = WGS84::lla2ecef(lla1);

    math::DQuat<double> dq_e2n = WGS84::dq_ecef2ned(ecef0);
    Vec3 ecef_hat = WGS84::ned2ecef(dq_e2n, ned1);
    Vec3 ned1_hat = WGS84::ecef2ned(dq_e2n, ecef1);

    MATRIX_CLOSE(ecef_hat, ecef1, 1e-6);
    MATRIX_CLOSE(ned1_hat, ned1, 1e-6);
}

TEST(WGS84, lla2ned)
{
    Vec3 lla0 = {deg2Rad(40.247082), -deg2Rad(111.647776), 1387.998309};
    Vec3 lla1 = {deg2Rad(40.246587), -deg2Rad(111.647761), 1387.998309};
    Vec3 ned1 = {-54.976484, 1.276565, 0.000237};

    Vec3 ned1_hat = WGS84::lla2ned(lla0, lla1);
    Vec3 lla1_hat = WGS84::ned2lla(lla0, ned1);

    MATRIX_CLOSE(lla1_hat, lla1, 1e-6);
    MATRIX_CLOSE(ned1_hat, ned1, 1e-6);
}

}  // namespace utils
}  // namespace mc
