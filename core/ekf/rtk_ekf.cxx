#include "core/ekf/rtk_ekf.h"

namespace mc {
namespace ekf {

#define T transpose()

template <>
pointPosMeas::Residual RtkEkf::h<pointPosMeas>(const pointPosMeas::ZType& z,
                                               const State& x,
                                               pointPosMeas::Jac* jac,
                                               const Input& u) const
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
        jac->dPose().topLeftCorner<3, 3>() = q_b2e.rotp(skew(x.p_b2g));
        jac->dPose().topRightCorner<3, 3>() = -R_b2e;
        jac->dPose().bottomLeftCorner<3, 3>() = x.T_I2e.rotp(x.pose.rota(skew(v_I2g_b)));

        jac->dP_b2g().topRows<3>() = -R_b2e;
        jac->dP_b2g().bottomRows<3>() = -q_b2e.rotp(skew(u.gyro));

        jac->dT_i2e().topLeftCorner<3, 3>() = -skew(p_e2g_e);
        jac->dT_i2e().topRightCorner<3, 3>() = Mat3::Identity();
        jac->dT_i2e().bottomLeftCorner<3, 3>() = -skew(x.T_I2e.rotp(x.pose.rota(v_I2g_b)));

        jac->dVel().bottomRows<3>() = -R_b2e;
    }

    return z - zhat;
}

template <>
gpsObsMeas::Residual RtkEkf::h<gpsObsMeas>(const Vec2& z,
                                           const State& x,
                                           gpsObsMeas::Jac* jac,
                                           const Input& u,
                                           const satellite::SatelliteCache& sat) const
{
    const Vec3 p_I2g_I = x.pose.transforma(x.p_b2g);
    const Vec3 p_e2g_e = x.T_I2e.transformp(p_I2g_I);
    const Vec3 v_I2g_b = u.gyro.cross(x.p_b2g) + x.vel;
    const math::Quat<double> q_b2e = (x.pose.rotation().inverse() * x.T_I2e.rotation());

    const Vec3 v_I2g_e = q_b2e.rotp(v_I2g_b);

    const Vec3 los_to_sat = sat.state.pos - p_e2g_e;

    const double los_norm = los_to_sat.stableNorm();
    const double range = los_norm + sat.sagnac;

    const double dclock = x.gps_clk(0) / 1e9 - sat.state.clk(0);
    const double dclock_rate = x.gps_clk(1) / 1e9 - sat.state.clk(1);
    const Vec3 e = los_to_sat / los_norm;

    const double est_prange =
        range + sat.atm.ionospheric_delay_m + sat.atm.tropospheric_delay_m + C_LIGHT * dclock;
    const double est_doppler =
        (sat.state.vel - v_I2g_e).dot(e) +
        OMEGA_EARTH / C_LIGHT *
            (sat.state.vel.y() * p_e2g_e.x() + sat.state.pos.y() * v_I2g_e.x() -
             sat.state.vel.x() * p_e2g_e.y() - sat.state.pos.x() * v_I2g_e.y()) +
        C_LIGHT * dclock_rate;

    const Vec2 zhat = {est_prange, est_doppler};

    if (jac)
    {
        const Mat3 R_b2e = q_b2e.R();

        jac->dPose().topLeftCorner<1, 3>() = -e.T * R_b2e * skew(x.p_b2g);
        jac->dPose().topRightCorner<1, 3>() = e.T * R_b2e;
        jac->dPose().bottomLeftCorner<1, 3>() = -e.T * R_b2e * skew(v_I2g_b);

        jac->dVel().bottomRows<1>() = e.T * R_b2e;

        jac->dT_i2e().topLeftCorner<1, 3>() = e.T * skew(p_e2g_e);
        jac->dT_i2e().topRightCorner<1, 3>() = -e.T * Mat3::Identity();
        jac->dT_i2e().bottomLeftCorner<1, 3>() = e.T * skew(R_b2e * v_I2g_b);

        jac->dP_b2g().topRows<1>() = e.T * R_b2e;
        jac->dP_b2g().bottomRows<1>() = e.T * q_b2e.rotp(skew(u.gyro));

        jac->dGps_clk()(0, 0) = -(C_LIGHT / 1e9);
        jac->dGps_clk()(1, 1) = -(C_LIGHT / 1e9);
    }
    return z - zhat;
}

template <>
galObsMeas::Residual RtkEkf::h<galObsMeas>(const Vec2& z,
                                           const State& x,
                                           galObsMeas::Jac* jac,
                                           const Input& u,
                                           const satellite::SatelliteCache& sat) const
{
    const Vec3 p_I2g_I = x.pose.transforma(x.p_b2g);
    const Vec3 p_e2g_e = x.T_I2e.transformp(p_I2g_I);
    const Vec3 v_I2g_b = u.gyro.cross(x.p_b2g) + x.vel;
    const math::Quat<double> q_b2e = (x.pose.rotation().inverse() * x.T_I2e.rotation());

    const Vec3 v_I2g_e = q_b2e.rotp(v_I2g_b);

    const Vec3 los_to_sat = sat.state.pos - p_e2g_e;

    const double los_norm = los_to_sat.stableNorm();
    const double range = los_norm + sat.sagnac;

    const double dclock = x.gal_clk(0) / 1e9 - sat.state.clk(0);
    const double dclock_rate = x.gal_clk(1) / 1e9 - sat.state.clk(1);
    const Vec3 e = los_to_sat / los_norm;

    const double est_prange =
        range + sat.atm.ionospheric_delay_m + sat.atm.tropospheric_delay_m + C_LIGHT * dclock;
    const double est_doppler =
        (sat.state.vel - v_I2g_e).dot(e) +
        OMEGA_EARTH / C_LIGHT *
            (sat.state.vel.y() * p_e2g_e.x() + sat.state.pos.y() * v_I2g_e.x() -
             sat.state.vel.x() * p_e2g_e.y() - sat.state.pos.x() * v_I2g_e.y()) +
        C_LIGHT * dclock_rate;

    const Vec2 zhat = {est_prange, est_doppler};

    if (jac)
    {
        const Mat3 R_b2e = q_b2e.R();

        jac->dPose().topLeftCorner<1, 3>() = -e.T * R_b2e * skew(x.p_b2g);
        jac->dPose().topRightCorner<1, 3>() = e.T * R_b2e;
        jac->dPose().bottomLeftCorner<1, 3>() = -e.T * R_b2e * skew(v_I2g_b);

        jac->dVel().bottomRows<1>() = e.T * R_b2e;

        jac->dT_i2e().topLeftCorner<1, 3>() = e.T * skew(p_e2g_e);
        jac->dT_i2e().topRightCorner<1, 3>() = -e.T * Mat3::Identity();
        jac->dT_i2e().bottomLeftCorner<1, 3>() = e.T * skew(R_b2e * v_I2g_b);

        jac->dP_b2g().topRows<1>() = e.T * R_b2e;
        jac->dP_b2g().bottomRows<1>() = e.T * q_b2e.rotp(skew(u.gyro));

        jac->dGal_clk()(0, 0) = -(C_LIGHT / 1e9);
        jac->dGal_clk()(1, 1) = -(C_LIGHT / 1e9);
    }
    return z - zhat;
}

template <>
gloObsMeas::Residual RtkEkf::h<gloObsMeas>(const Vec2& z,
                                           const State& x,
                                           gloObsMeas::Jac* jac,
                                           const Input& u,
                                           const satellite::SatelliteCache& sat) const
{
    const Vec3 p_I2g_I = x.pose.transforma(x.p_b2g);
    const Vec3 p_e2g_e = x.T_I2e.transformp(p_I2g_I);
    const Vec3 v_I2g_b = u.gyro.cross(x.p_b2g) + x.vel;
    const math::Quat<double> q_b2e = (x.pose.rotation().inverse() * x.T_I2e.rotation());

    const Vec3 v_I2g_e = q_b2e.rotp(v_I2g_b);

    const Vec3 los_to_sat = sat.state.pos - p_e2g_e;

    const double los_norm = los_to_sat.stableNorm();
    const double range = los_norm + sat.sagnac;

    const double dclock = x.glo_clk(0) / 1e9 - sat.state.clk(0);
    const double dclock_rate = x.glo_clk(1) / 1e9 - sat.state.clk(1);
    const Vec3 e = los_to_sat / los_norm;

    const double est_prange =
        range + sat.atm.ionospheric_delay_m + sat.atm.tropospheric_delay_m + C_LIGHT * dclock;
    const double est_doppler =
        (sat.state.vel - v_I2g_e).dot(e) +
        OMEGA_EARTH / C_LIGHT *
            (sat.state.vel.y() * p_e2g_e.x() + sat.state.pos.y() * v_I2g_e.x() -
             sat.state.vel.x() * p_e2g_e.y() - sat.state.pos.x() * v_I2g_e.y()) +
        C_LIGHT * dclock_rate;

    const Vec2 zhat = {est_prange, est_doppler};

    if (jac)
    {
        const Mat3 R_b2e = q_b2e.R();

        jac->dPose().topLeftCorner<1, 3>() = -e.T * R_b2e * skew(x.p_b2g);
        jac->dPose().topRightCorner<1, 3>() = e.T * R_b2e;
        jac->dPose().bottomLeftCorner<1, 3>() = -e.T * R_b2e * skew(v_I2g_b);

        jac->dVel().bottomRows<1>() = e.T * R_b2e;

        jac->dT_i2e().topLeftCorner<1, 3>() = e.T * skew(p_e2g_e);
        jac->dT_i2e().topRightCorner<1, 3>() = -e.T * Mat3::Identity();
        jac->dT_i2e().bottomLeftCorner<1, 3>() = e.T * skew(R_b2e * v_I2g_b);

        jac->dP_b2g().topRows<1>() = e.T * R_b2e;
        jac->dP_b2g().bottomRows<1>() = e.T * q_b2e.rotp(skew(u.gyro));

        jac->dGlo_clk()(0, 0) = -(C_LIGHT / 1e9);
        jac->dGlo_clk()(1, 1) = -(C_LIGHT / 1e9);
    }
    return z - zhat;
}

template <>
fixAndHoldMeas::Residual RtkEkf::h<fixAndHoldMeas>(const fixAndHoldMeas::Residual& z,
                                                   const State& x,
                                                   fixAndHoldMeas::Jac* jac) const
{
    const auto& zhat = x.sd;

    if (jac)
    {
        jac->dSd().setIdentity();
        jac->dSd().diagonal() *= -1;
    }

    return z - zhat;
}

ErrorState RtkEkf::dynamics(const State& x,
                            const Input& u,
                            RtkEkf::StateJac* dxdx,
                            RtkEkf::InputJac* dxdu)
{
    const Vec3 omega = u.gyro - x.gyro_bias;
    const Vec3 acc = u.accel - x.acc_bias;
    static const Vec3 gravity = Vec3::Unit(2) * 9.80665;

    if (dxdx)
    {
        dxdx->setZero();
        auto dRotdRot = dxdx->dPose_dPose().block<3, 3>(0, 0);
        auto dPosdRot = dxdx->dPose_dPose().block<3, 3>(3, 0);
        auto dPosdVel = dxdx->dPose_dVel().block<3, 3>(3, 0);
        auto dRotdBg = dxdx->dPose_dGyrobias().block<3, 3>(0, 0);
        auto dVeldRot = dxdx->dVel_dPose().block<3, 3>(0, 0);

        dRotdRot = -skew(omega);
        dPosdRot = -skew(x.vel);
        dPosdVel.setIdentity();
        dRotdBg = -Mat3::Identity();
        dVeldRot = -skew(acc);

        dxdx->dVel_dAccbias() = -Mat3::Identity();
    }
    if (dxdu)
    {
        dxdu->setZero();
        dxdu->dPosedGyro().topRows<3>().setIdentity();
        dxdu->dVeldAccel().setIdentity();
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
    xdot.sd.setZero();

    return xdot;
}

ErrorState RtkEkf::errorStateDynamics(const ErrorState& dx,
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

}  // namespace ekf
}  // namespace mc
