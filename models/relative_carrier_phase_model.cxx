#include "models/relative_carrier_phase_model.h"

#include "common/check.h"
#include "common/defs.h"
#include "common/print.h"

namespace mc {
namespace models {

using DQ = math::DQuat<double>;
using Eigen::Map;
#define T transpose()

RelativeCarrierPhase::RelativeCarrierPhase(const meas::GnssObservation& obs1,
                                           const meas::GnssObservation& obs2,
                                           const satellite::SatelliteBase& sat,
                                           const Vec3& rec_pos_ecef,
                                           const math::DQuat<double>& T_e2r,
                                           const double& Xi)
    : dPhi_(obs2.carrier_phase - obs1.carrier_phase),
      lambda_(sat.carrierWavelength(obs1.freq)),
      Xi_(Xi),
      T_e2r_(T_e2r)
{
    check(sat.almanacSize() > 0, "Cannot create factor without ephemeris");
    check(obs1.gnss_id == obs2.gnss_id,
          "Must perform relative carrier phase differencing with same GNSS constallation");
    check(obs1.sat_num == obs2.sat_num,
          "Must perform relative carrier phase differencing with same satellite");
    check(obs1.freq == obs2.freq,
          "Must perform relative carrier phase differencing with same frequency");
    check(obs1.gnss_id == sat.gnssId(), "observation and satellite not from same constellation");
    check(obs1.sat_num == sat.sat_num(), "observation and satellite not the same ID");

    computeSatState(obs1, rec_pos_ecef, sat, Out(sat1_));
    computeSatState(obs2, rec_pos_ecef, sat, Out(sat2_));
}

double RelativeCarrierPhase::computeSagnac(const satellite::SatelliteState& sat,
                                           const Vec3& rec_pos_ecef) const
{
    return OMEGA_EARTH * (sat.pos.x() * rec_pos_ecef.y() - sat.pos.y() * rec_pos_ecef.x()) /
           C_LIGHT;
}

void RelativeCarrierPhase::computeSatState(const meas::GnssObservation& obs,
                                           const Vec3& rec_pos_ecef,
                                           const satellite::SatelliteBase& sat,
                                           Out<satellite::SatelliteState> sat_state) const
{
    sat.getState(obs.t, sat_state);
    double dt = std::numeric_limits<double>::max();
    while (abs(dt) * sat_state->vel.norm() > 1e-3)  // get millimeter-accurate
    {
        // Compute how long it took for the signal to get here (including Sagnac Correction)
        const Vec3 los_to_sat = sat_state->pos - rec_pos_ecef;
        const double sagnac = computeSagnac(*sat_state, rec_pos_ecef);
        const double range = los_to_sat.norm() + sagnac;

        const double tau = range / C_LIGHT;
        dt = tau - (obs.t - sat_state->t).toSec();
        sat.getState(sat_state->t - dt, sat_state);
    }
}

struct PoseJacobian final : public Vec6
{
    auto drot() { return head<3>(); }
    auto dpos() { return tail<3>(); }

    Vec8 by_elements(const DQ& Q) { return (*this).transpose() * Q.dGenDParam(); }
};

bool RelativeCarrierPhase::Evaluate(const double* const* parameters,
                                    double* residual,
                                    double** jacobians) const
{
    const DQ T_n2b1(parameters[0]);
    const DQ T_n2b2(parameters[1]);

    Map<const Vec2> clk1(parameters[2]);
    Map<const Vec2> clk2(parameters[3]);

    const double& sw_var1(*parameters[4]);
    const double& sw_var2(*parameters[5]);

    const DQ T_r2n(parameters[6]);
    Map<const Vec3> p_b2g(parameters[7]);

    // Recevier position at time 1 and time 2
    const Vec3 p_e2b1 = T_e2r_.rota(T_r2n.transforma(T_n2b1.translation() + T_n2b1.rota(p_b2g))) +
                        T_e2r_.translation();
    const Vec3 p_e2b2 = T_e2r_.rota(T_r2n.transforma(T_n2b2.translation() + T_n2b2.rota(p_b2g))) +
                        T_e2r_.translation();

    /// TODO: Consider the effect that moving the receiver has on our estimate of the satellite
    /// position and clock bias, and perhaps re-compute satellite position if receiver position has
    /// changed significantly

    // Line-of sight vector to satellite at time 1 and 2
    const Vec3 los1 = sat1_.pos - p_e2b1;
    const Vec3 los2 = sat2_.pos - p_e2b2;

    // Compute range to satellite (including sagnac effect)
    const double los1_norm = los1.stableNorm();
    const double los2_norm = los2.stableNorm();
    const double range1 = los1_norm + computeSagnac(sat1_, p_e2b1);
    const double range2 = los2_norm + computeSagnac(sat2_, p_e2b2);

    // estimated carrier-phase measurement at t1 and t2 in meters
    const double est1 = range1 + C_LIGHT * (clk1(0) / 1e9 - sat1_.clk(0));
    const double est2 = range2 + C_LIGHT * (clk2(0) / 1e9 - sat2_.clk(0));

    // estimated change in carrier phase assuming no loss of lock between t1 and t2 (in cycles)
    const double dPhi_hat = lambda_ * (est2 - est1);

    // Compute gain
    const double gain = Xi_ * sw_var1 * sw_var2;
    const double delta = dPhi_ - dPhi_hat;

    // Compute residual
    *residual = gain * delta;

    if (jacobians)
    {
        const Vec3 e1 = los1 / los1_norm;
        const Vec3 e2 = los2 / los2_norm;
        const Mat3 R_E2n = T_r2n.rotation().R() * T_e2r_.rotation().R();
        const Mat3 R_e2r = T_e2r_.rotation().R();
        const Mat3 R_r2n = T_r2n.rotation().R();
        const Mat3 R_n2b1 = T_n2b1.rotation().R();
        const Mat3 R_n2b2 = T_n2b2.rotation().R();

        // res_by_T_n2b1
        if (jacobians[0])
        {
            PoseJacobian jac;
            jac.drot() = gain * lambda_ * e1.T * R_E2n.T * R_n2b1.T * skew(p_b2g);
            jac.dpos() = gain * lambda_ * -e1.T * R_E2n.T * R_n2b1.T;
            Map<Vec8> out(jacobians[0]);
            out = jac.by_elements(T_n2b1);
        }
        // res_by_T_n2b2
        if (jacobians[1])
        {
            PoseJacobian jac;
            jac.drot() = gain * lambda_ * -e2.T * R_E2n.T * R_n2b2.T * skew(p_b2g);
            jac.dpos() = gain * lambda_ * e2.T * R_E2n.T * R_n2b2.T;
            Map<Vec8> out(jacobians[1]);
            out = jac.by_elements(T_n2b2);
        }
        // res_by_clk1
        if (jacobians[2])
        {
            jacobians[2][0] = gain * lambda_ * C_LIGHT / 1e9;
            jacobians[2][1] = 0;
        }
        // res_by_clk2
        if (jacobians[3])
        {
            jacobians[3][0] = -gain * lambda_ * C_LIGHT / 1e9;
            jacobians[3][1] = 0;
        }
        // res_by_sw1
        if (jacobians[4])
        {
            jacobians[4][0] = Xi_ * sw_var2 * delta;
        }
        // res_by_sw2
        if (jacobians[5])
        {
            jacobians[5][0] = Xi_ * sw_var1 * delta;
        }
        // res_by_T_r2n
        if (jacobians[6])
        {
            const Vec3 rot_by_1 =
                e1.T * R_e2r.T * R_r2n.T * skew(T_n2b1.translation() + T_n2b1.rota(p_b2g));
            const Vec3 rot_by_2 =
                e2.T * R_e2r.T * R_r2n.T * skew(T_n2b2.translation() + T_n2b2.rota(p_b2g));
            const Vec3 pos_by_1 = -e1.T * R_E2n.T;
            const Vec3 pos_by_2 = -e2.T * R_E2n.T;
            PoseJacobian jac;
            jac.drot() = gain * lambda_ * (rot_by_1 - rot_by_2);
            jac.dpos() = gain * lambda_ * (pos_by_1 - pos_by_2);

            Map<Vec8> out(jacobians[6]);
            out = jac.by_elements(T_r2n);
        }
        // res_by_p_b2g
        if (jacobians[7])
        {
            const Vec3 by_1 = e1.T * R_e2r.T * R_r2n.T * R_n2b1.T;
            const Vec3 by_2 = e2.T * R_e2r.T * R_r2n.T * R_n2b2.T;
            Map<Vec3> jac(jacobians[7]);
            jac = gain * lambda_ * (by_2 - by_1);
        }
    }
    return true;
}

}  // namespace models
}  // namespace mc
