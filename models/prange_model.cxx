#include "models/prange_model.h"
#include "common/check.h"
#include "common/defs.h"
#include "common/matrix_defs.h"

#include <limits>

namespace mc {
namespace models {

using DQ = math::DQuat<double>;

#define T transpose()

PseudorangeModel::PseudorangeModel(const meas::GnssObservation& obs,
                                   const satellite::SatelliteBase& sat,
                                   const Vec3& rec_pos_ecef,
                                   const Mat3& cov,
                                   const DQ& T_e2r)
    : sat_(sat), obs_(obs), T_e2r_(T_e2r)
{
    computeConstants(rec_pos_ecef);
    sqrt_inv_cov_ = cov.inverse().llt().matrixL().transpose();
}

void PseudorangeModel::computeConstants(const Vec3& rec_pos_ecef)
{
    check(sat_.almanacSize() > 0, "Cannot create factor without ephemeris");

    sat_.getState(obs_.t, Out(sat_state_));
    double dt = std::numeric_limits<double>::max();
    while (abs(dt) * sat_state_.vel.norm() > 1e-3)  // get millimeter-accurate
    {
        // Compute how long it took for the signal to get here (including Sagnac Correction)
        Vec3 los_to_sat = sat_state_.pos - rec_pos_ecef;
        sagnac_ = computeSagnac(rec_pos_ecef);
        double range = los_to_sat.norm() + sagnac_;

        double tau = range / C_LIGHT;
        dt = tau - (obs_.t - sat_state_.t).toSec();
        sat_.getState(sat_state_.t - dt, Out(sat_state_));
    }

    satellite::computeAtmCorrection(obs_.t, rec_pos_ecef, sat_state_, Out(atm_));
}

double PseudorangeModel::computeSagnac(const Vec3& rec_pos_ecef)
{
    return OMEGA_EARTH *
           (sat_state_.pos.x() * rec_pos_ecef.y() - sat_state_.pos.y() * rec_pos_ecef.x()) /
           C_LIGHT;
}

struct PoseJacobian final : public MatRM36
{
    auto drho_drot() { return block<1, 3>(0, 0); }
    auto drho_dpos() { return block<1, 3>(0, 3); }
    auto ddop_drot() { return block<1, 3>(1, 0); }
    auto ddop_dpos() { return block<1, 3>(1, 3); }
    auto dsw_dQ() { return bottomRows<1>(); }

    Mat38 by_elements(const DQ& Q) { return (*this) * Q.dGenDParam(); }
};

template <int N>
struct VecJacobian final : public Eigen::Map<Eigen::Matrix<double, 3, N, Eigen::RowMajor>>
{
    VecJacobian(double* ptr) : Eigen::Map<Eigen::Matrix<double, 3, N, Eigen::RowMajor>>(ptr) {}
    auto drho() { return this->row(0); }
    auto ddop() { return this->row(1); }
    auto dsw() { return this->row(2); }
};

bool PseudorangeModel::Evaluate(const double* const* parameters,
                                double* _residual,
                                double** jacobians) const
{
    const DQ T_n2b(parameters[0]);
    Eigen::Map<const Vec3> velocity_b(parameters[1]);
    Eigen::Map<const Vec2> clk(parameters[2]);
    const DQ T_r2n(parameters[3]);
    const double& sw_var(*parameters[4]);
    Eigen::Map<const Vec3> p_b2g(parameters[5]);
    Eigen::Map<Vec3> res(_residual);

    const Vec3 vel_ecef = T_e2r_.rota(T_r2n.rota(T_n2b.rota(velocity_b)));
    const Vec3 p_ecef = T_e2r_.rota(T_r2n.transforma(T_n2b.translation() + T_n2b.rota(p_b2g))) +
                        T_e2r_.translation();
    const Vec3 los_to_sat = sat_state_.pos - p_ecef;

    const double los_norm = los_to_sat.stableNorm();
    double range = los_norm + sagnac_;
    const double dclock = clk(0) - sat_state_.clk(0);
    const double est_prange =
        range + atm_.ionospheric_delay_m + atm_.tropospheric_delay_m + (C_LIGHT / 1e9) * dclock;
    const double dclock_rate = clk(1) - sat_state_.clk(1);
    const double est_doppler =
        (sat_state_.vel - vel_ecef).dot(los_to_sat / range) +
        OMEGA_EARTH / C_LIGHT *
            (sat_state_.vel.y() * p_ecef.x() + sat_state_.pos.y() * vel_ecef.x() -
             sat_state_.vel.x() * p_ecef.y() - sat_state_.pos.x() * vel_ecef.y()) +
        (C_LIGHT / 1e9) * dclock_rate;

    const Vec3 residual(sw_var * (est_prange - obs_.pseudorange),
                        sw_var * (est_doppler - obs_.doppler), 1.0 - sw_var);
    res = sqrt_inv_cov_ * residual;

    if (jacobians)
    {
        const Vec3 e = los_to_sat / los_norm;
        const Mat3 R_E2n = T_r2n.rotation().R() * T_e2r_.rotation().R();
        const Mat3 R_e2r = T_e2r_.rotation().R();
        const Mat3 R_r2n = T_r2n.rotation().R();
        const Mat3 R_n2b = T_n2b.rotation().R();

        // res_by_T_n2b
        if (jacobians[0])
        {
            PoseJacobian jac;
            jac.drho_drot() = sw_var * e.T * R_E2n.T * R_n2b.T * skew(p_b2g);
            jac.drho_dpos() = sw_var * -e.T * R_E2n.T * R_n2b.T;
            jac.ddop_drot() = sw_var * e.T * R_E2n.T * R_n2b.T * skew(velocity_b);  // approx
            jac.ddop_dpos().setZero();                                              // approx
            jac.dsw_dQ().setZero();
            Eigen::Map<MatRM38> out(jacobians[0]);
            out = sqrt_inv_cov_ * jac.by_elements(T_n2b);
        }

        // res_by_vel_b
        if (jacobians[1])
        {
            VecJacobian<3> jac(jacobians[1]);
            jac.drho().setZero();
            jac.ddop() = sw_var * -e.T * sqrt_inv_cov_(1, 1) * R_E2n.T * R_n2b.T;
            jac.dsw().setZero();
        }

        // res_by_clk
        if (jacobians[2])
        {
            VecJacobian<2> jac(jacobians[2]);
            jac.drho() << sw_var * C_LIGHT / 1e9 * sqrt_inv_cov_(0, 0), 0;
            jac.ddop() << 0, sw_var * C_LIGHT / 1e9 * sqrt_inv_cov_(1, 1);
            jac.dsw().setZero();
        }

        // res_by_T_r2n
        if (jacobians[3])
        {
            PoseJacobian jac;
            jac.drho_drot() =
                sw_var * e.T * R_e2r.T * R_r2n.T * skew(T_n2b.translation() + T_n2b.rota(p_b2g));
            jac.drho_dpos() = sw_var * -e.T * R_E2n.T;
            jac.ddop_drot() = sw_var * e.T * R_E2n.T * skew(R_n2b.T * velocity_b);
            jac.ddop_dpos().setZero();
            jac.dsw_dQ().setZero();
            Eigen::Map<MatRM38> out(jacobians[3]);
            out = sqrt_inv_cov_ * jac.by_elements(T_r2n);
        }
        // res_by_sw_var
        if (jacobians[4])
        {
            Eigen::Map<Vec3> jac(jacobians[4]);
            jac(0) = sqrt_inv_cov_(0, 0) * (est_prange - obs_.pseudorange);
            jac(1) = sqrt_inv_cov_(1, 1) * (est_doppler - obs_.doppler);
            jac(2) = -sqrt_inv_cov_(2, 2);
        }
        // res_by_p_b2g
        if (jacobians[5])
        {
            VecJacobian<3> jac(jacobians[5]);
            jac.drho() = -sw_var * sqrt_inv_cov_(0, 0) * e.T * R_e2r.T * R_r2n.T * R_n2b.T;
            jac.ddop().setZero();
            jac.dsw().setZero();
        }
    }

    return true;
}

}  // namespace models
}  // namespace mc
