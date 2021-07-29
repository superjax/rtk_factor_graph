#include "common/satellite/satellite_cache.h"
#include "core/ekf/rtk_ekf.h"

namespace mc {
namespace ekf {

#define T transpose()

ErrorState dynamics(const State& x, const Input& u, StateJac* dxdx, InputJac* dxdu)
{
    const Vec3 omega = u.gyro - x.gyro_bias;
    const Vec3 acc = u.accel - x.acc_bias;
    static const Vec3 gravity = Vec3::Unit(2) * 9.80665;

    if (dxdx)
    {
        dxdx->setZero();
        auto dRotdRot = dxdx->dPose_dPose.block<3, 3>(0, 0);
        auto dPosdRot = dxdx->dPose_dPose.block<3, 3>(3, 0);
        auto dPosdVel = dxdx->dPose_dVel.block<3, 3>(3, 0);
        auto dRotdBg = dxdx->dPose_dGyroBias.block<3, 3>(0, 0);
        auto dVeldRot = dxdx->dVel_dPose.block<3, 3>(0, 0);

        dRotdRot = -skew(omega);
        dPosdRot = -skew(x.vel);
        dPosdVel.setIdentity();
        dRotdBg = -Mat3::Identity();
        dVeldRot = -skew(acc);

        dxdx->dVel_dAccBias = -Mat3::Identity();
    }

    if (dxdu)
    {
        dxdu->setZero();
        dxdu->dPose_dGyro.topRows<3>().setIdentity();
        dxdu->dVel_dAccel.setIdentity();
    }

    ErrorState xdot;
    xdot.pose.head<3>() = omega;
    xdot.pose.tail<3>() = x.vel;
    xdot.vel = x.pose.rota(gravity) + acc;
    xdot.acc_bias.setZero();
    xdot.gyro_bias.setZero();
    xdot.gps_clk.setZero();
    xdot.gal_clk.setZero();
    xdot.glo_clk.setZero();
    xdot.gps_clk << x.gps_clk(1), 0;
    xdot.gal_clk << x.gal_clk(1), 0;
    xdot.glo_clk << x.glo_clk(1), 0;
    xdot.p_b2g.setZero();
    xdot.T_I2e.setZero();
    xdot.sd_vec(x.num_sd).setZero();

    return xdot;
}

ErrorState errorStateDynamics(const ErrorState& dx,
                              const State& x,
                              const Input& u,
                              const Input& eta)
{
    ErrorState out = ErrorState::Zero();
    out.pose.head<3>() = -(u.gyro - x.gyro_bias).cross(dx.pose.head<3>()) - dx.gyro_bias + eta.gyro;
    out.pose.tail<3>() = dx.vel - x.vel.cross(dx.pose.head<3>());
    out.vel = (-dx.acc_bias + eta.accel) - (u.accel - x.acc_bias).cross(dx.pose.head<3>());
    return out;
}

template <>
pointPosMeas::Residual h<pointPosMeas>(const pointPosMeas::ZType& z,
                                       const State& x,
                                       pointPosMeas::Jac* jac,
                                       const Input& u)
{
    const Vec3 p_I2g_I = x.pose.transforma(x.p_b2g);
    const Vec3 p_e2g_e = x.T_I2e.transformp(p_I2g_I);
    const Vec3 v_I2g_b = u.gyro.cross(x.p_b2g) + x.vel;
    const math::Quat<double> q_b2e = (x.pose.rotation().inverse() * x.T_I2e.rotation());

    const Vec3 v_I2g_e = q_b2e.rotp(v_I2g_b);

    const Vec6 zhat = (Vec6() << p_e2g_e, v_I2g_e).finished();

    if (jac)
    {
        const Mat3 R_b2e = q_b2e.R();
        jac->dPose.topLeftCorner<3, 3>() = q_b2e.rotp(skew(x.p_b2g));
        jac->dPose.topRightCorner<3, 3>() = -R_b2e;
        jac->dPose.bottomLeftCorner<3, 3>() = x.T_I2e.rotp(x.pose.rota(skew(v_I2g_b)));
        jac->dPose.bottomRightCorner<3, 3>().setZero();

        jac->dPB2g.topRows<3>() = -R_b2e;
        jac->dPB2g.bottomRows<3>() = -q_b2e.rotp(skew(u.gyro));

        jac->dTI2e.topLeftCorner<3, 3>() = -skew(p_e2g_e);
        jac->dTI2e.topRightCorner<3, 3>() = Mat3::Identity();
        jac->dTI2e.bottomLeftCorner<3, 3>() = -skew(x.T_I2e.rotp(x.pose.rota(v_I2g_b)));
        jac->dTI2e.bottomRightCorner<3, 3>().setZero();

        jac->dVel.topRows<3>().setZero();
        jac->dVel.bottomRows<3>() = -R_b2e;
    }

    return z - zhat;
}

template <>
obsMeas::Residual h<obsMeas>(const obsMeas::ZType& z,
                             const State& x,
                             obsMeas::Jac* jac,
                             const Input& u)
{
    const Vec3 p_I2g_I = x.pose.transforma(x.p_b2g);
    const Vec3 p_e2g_e = x.T_I2e.transformp(p_I2g_I);
    const Vec3 v_I2g_b = u.gyro.cross(x.p_b2g) + x.vel;
    const math::Quat<double> q_b2e = (x.pose.rotation().inverse() * x.T_I2e.rotation());
    const Vec3 v_I2g_e = q_b2e.rotp(v_I2g_b);

    obsMeas::Residual residual;

    for (size_t i = 0; i < z.size(); ++i)
    {
        const Vec2& z_i = z[i].z;
        const satellite::SatelliteCache& sat = z[i].sat;
        const int gnss_id = z[i].gnss_id;

        check(gnss_id == GnssID::GPS || gnss_id == GnssID::Galileo || gnss_id == GnssID::Glonass);
        // clang-format off
        const Vec2& clk = gnss_id == GnssID::GPS ? x.gps_clk :
                          gnss_id == GnssID::Galileo ? x.gal_clk :
                          x.glo_clk;
        // clang-format on

        const Vec3 los_to_sat = sat.state.pos - p_e2g_e;

        const double los_norm = los_to_sat.stableNorm();
        const double range = los_norm + sat.sagnac;

        const double dclock = clk(0) / 1e9 - sat.state.clk(0);
        const double dclock_rate = clk(1) / 1e9 - sat.state.clk(1);
        const Vec3 e = los_to_sat / los_norm;

        const double est_prange =
            range + sat.atm.ionospheric_delay_m + sat.atm.tropospheric_delay_m + C_LIGHT * dclock;
        const double est_doppler =
            (sat.state.vel - v_I2g_e).dot(e) +
            OMEGA_EARTH / C_LIGHT *
                (sat.state.vel.y() * p_e2g_e.x() + sat.state.pos.y() * v_I2g_e.x() -
                 sat.state.vel.x() * p_e2g_e.y() - sat.state.pos.x() * v_I2g_e.y()) +
            C_LIGHT * dclock_rate;

        const Vec2 zhat_i = {est_prange, est_doppler};
        residual.segment<2>(i * 2) = z_i - zhat_i;

        if (jac)
        {
            const Mat3 R_b2e = q_b2e.R();

            jac->dPose(i).topLeftCorner<1, 3>() = -e.T * R_b2e * skew(x.p_b2g);
            jac->dPose(i).topRightCorner<1, 3>() = e.T * R_b2e;
            jac->dPose(i).bottomLeftCorner<1, 3>() = -e.T * R_b2e * skew(v_I2g_b);
            jac->dPose(i).bottomRightCorner<1, 3>().setZero();

            jac->dVel(i).topRows<1>().setZero();
            jac->dVel(i).bottomRows<1>() = e.T * R_b2e;

            jac->dTI2e(i).topLeftCorner<1, 3>() = e.T * skew(p_e2g_e);
            jac->dTI2e(i).topRightCorner<1, 3>() = -e.T * Mat3::Identity();
            jac->dTI2e(i).bottomLeftCorner<1, 3>() = e.T * skew(R_b2e * v_I2g_b);
            jac->dTI2e(i).bottomRightCorner<1, 3>().setZero();

            jac->dPB2g(i).topRows<1>() = e.T * R_b2e;
            jac->dPB2g(i).bottomRows<1>() = e.T * q_b2e.rotp(skew(u.gyro));

            jac->dGpsClk(i).setZero();
            jac->dGalClk(i).setZero();
            jac->dGloClk(i).setZero();
            if (gnss_id == GnssID::GPS)
            {
                jac->dGpsClk(i).diagonal().setConstant(-C_LIGHT / 1e9);
            }
            else if (gnss_id == GnssID::Galileo)
            {
                jac->dGalClk(i).diagonal().setConstant(-C_LIGHT / 1e9);
            }
            else if (gnss_id == GnssID::Glonass)
            {
                jac->dGloClk(i).diagonal().setConstant(-C_LIGHT / 1e9);
            }
        }
    }
    return residual;
}

template <>
fixAndHoldMeas::Residual h<fixAndHoldMeas>(const fixAndHoldMeas::ZType& z,
                                           const State& x,
                                           fixAndHoldMeas::Jac* jac)
{
    const int num_sd = x.num_sd;
    const int num_fix_and_hold = z.size();

    check(num_sd == num_fix_and_hold,
          "Number of fix-and-hold measurements must equal the side of the single-differences "
          "estimate.  num_sd = {}, num_fix_and_hold = {}",
          fmt(num_sd, num_fix_and_hold));

    fixAndHoldMeas::Residual res;
    for (int i = 0; i < num_sd; ++i)
    {
        res[i] = (z[i] - x.sd[i])[0];
    }

    if (jac)
    {
        jac->dSd_all(num_sd, num_fix_and_hold).setZero();
        jac->dSd_all(num_sd, num_fix_and_hold).diagonal().setConstant(-1.0);
    }

    return res;
}

}  // namespace ekf
}  // namespace mc
