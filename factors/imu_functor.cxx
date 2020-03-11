#include "factors/imu_functor.h"
#include "common/assert.h"
#include "common/matrix_defs.h"

namespace mc {
namespace factors {

using namespace Eigen;

static const Vec3 GRAVITY = 9.80665 * Vec3::UnitZ();

ImuFunctor::ImuFunctor(const UTCTime& t0, const Vec6& bias)
{
    t0_ = tf_ = t0;

    bias_ = bias;
    num_updates_ = 0;
    state_.setIdentity();
    covariance_.setZero();
    dstate_dbias_.setZero();
}

void ImuFunctor::errorStateDynamics(const ImuState& state,
                                    const ImuErrorState& dstate,
                                    const meas::ImuSample& imu,
                                    const Vec6& input_noise,
                                    Out<ImuErrorState> error_state_dot) const
{
    error_state_dot->alpha = dstate.beta;
    error_state_dot->beta =
        -state.gamma.rota((imu.accel - accelBias()).cross(dstate.gamma) + input_noise.head<3>());
    error_state_dot->gamma = -(imu.gyro - gyroBias()).cross(dstate.gamma) - input_noise.tail<3>();
}

void ImuFunctor::dynamics(const ImuState& state,
                          const meas::ImuSample& imu,
                          Out<ImuErrorState> state_dot,
                          Out<Mat9> A,
                          Out<Mat96> B) const
{
    const Vec3 accel = imu.accel - accelBias();
    const Vec3 omega = imu.gyro - gyroBias();

    state_dot->alpha = state.beta;
    state_dot->beta = state.gamma.rota(accel);
    state_dot->gamma = omega;

    static constexpr int ALPHA = 0;
    static constexpr int BETA = 3;
    static constexpr int GAMMA = 6;

    static constexpr int ACCEL = 0;
    static constexpr int OMEGA = 3;

    A->block<3, 3>(ALPHA, ALPHA).setZero();
    A->block<3, 3>(ALPHA, BETA) = Mat3::Identity();
    A->block<3, 3>(ALPHA, GAMMA).setZero();

    A->block<3, 3>(BETA, ALPHA).setZero();
    A->block<3, 3>(BETA, BETA).setZero();
    A->block<3, 3>(BETA, GAMMA) = -state.gamma.R().transpose() * skew(accel);

    A->block<3, 3>(GAMMA, ALPHA).setZero();
    A->block<3, 3>(GAMMA, BETA).setZero();
    A->block<3, 3>(GAMMA, GAMMA) = -skew(omega);

    B->block<3, 3>(ALPHA, ACCEL).setZero();
    B->block<3, 3>(ALPHA, OMEGA).setZero();

    B->block<3, 3>(BETA, ACCEL) = -state.gamma.R().transpose().matrix();
    B->block<3, 3>(BETA, OMEGA).setZero();

    B->block<3, 3>(GAMMA, ACCEL).setZero();
    B->block<3, 3>(GAMMA, OMEGA) = -Mat3::Identity();
}

Error ImuFunctor::integrate(const meas::ImuSample& imu, const Mat6& imu_cov)
{
    ASSERT(isFinite(imu_cov), "NaN detected in covariance on propagation");
    ASSERT(isFinite(imu.accel), "NaN detected in imu acceleration data");
    ASSERT(isFinite(imu.gyro), "NaN detected in imu gyroscope data");

    num_updates_++;

    double dt = (imu.t - tf_).toSec();
    tf_ = imu.t;

    ImuErrorState dstate;
    Mat9 A;
    Mat96 B;

    prev_imu_ = imu;
    prev_imu_covariance_ = imu_cov;

    dynamics(state_, imu, Out(dstate), Out(A), Out(B));
    state_ = state_ + (dstate * dt);

    const Mat9 Adt = A * dt;

    A = Mat9::Identity() + Adt * (Mat9::Identity() + 0.5 * Adt);
    B = B * dt;

    covariance_ = A * covariance_ * A.transpose() + B * imu_cov * B.transpose();
    dstate_dbias_ = A * dstate_dbias_ + B;

    ASSERT(isFinite(A), "NaN detected in state transition matrix on propagation");
    ASSERT(isFinite(covariance_), "NaN detected in covariance on propagation");

    return Error::none();
}

Error ImuFunctor::finished()
{
    if (num_updates_ <= 2)
    {
        info("less than two IMU measurements in integrator");
        covariance_ = covariance_ + Mat9::Identity() * 1e-15;
    }
    covariance_inv_sqrt_ = covariance_.inverse().llt().matrixL().transpose();

    ASSERT(isFinite(covariance_inv_sqrt_), "Nan detected in IMU information matrix.");
    return Error::none();
}

struct PoseJacobian
{
    PoseJacobian(double* ptr) : mat(ptr) {}

    auto dr1_drot() { return mat.block<3, 3>(0, 0); }
    auto dr2_drot() { return mat.block<3, 3>(3, 0); }
    auto dr3_drot() { return mat.block<3, 3>(6, 0); }

    auto dr1_dpos() { return mat.block<3, 3>(0, 3); }
    auto dr2_dpos() { return mat.block<3, 3>(3, 3); }
    auto dr3_dpos() { return mat.block<3, 3>(6, 3); }

    void linearize_local_param(const math::DQuat<double>& Q)
    {
        mat = mat.block<9, 6>(0, 0) * Q.dGenDParam();
    }

    //  private:
    Map<Matrix<double, 9, 8, RowMajor>> mat;
};

bool ImuFunctor::Evaluate(const double* const* parameters, double* _res, double** _jac) const
{
    const math::DQuat<double> start_pose(parameters[0]);
    const math::DQuat<double> end_pose(parameters[1]);
    const Map<const Vec3> start_vel(parameters[2]);
    const Map<const Vec3> end_vel(parameters[3]);
    const Map<const Vec3> accel_bias(parameters[4]);
    const Map<const Vec3> gyro_bias(parameters[4] + 3);
    const Map<const Vec6> bias(parameters[4]);

    Map<Vec9> residual(_res);

    const ImuErrorState bias_adjustment = dstate_dbias_ * bias;  //(bias - bias_);
    ImuState adjusted_state = state_ + bias_adjustment;

    const double _dt = dt();

    residual.segment<3>(0) =
        start_pose.rotation().rotp(end_pose.translation() - start_pose.translation() -
                                   0.5 * GRAVITY * _dt * _dt) -
        start_vel * _dt - adjusted_state.alpha;
    residual.segment<3>(3) = adjusted_state.gamma.rota(end_vel) - start_vel -
                             start_pose.rotation().rotp(GRAVITY) * _dt - adjusted_state.beta;

    math::Quat<double> rot_error =
        (adjusted_state.gamma.inverse() * (start_pose.rotation().inverse() * end_pose.rotation()));
    Mat3 log_jac;
    residual.segment<3>(6) = rot_error.log(&log_jac);

    if (_jac)
    {
        if (_jac[0])
        {
            PoseJacobian jac(_jac[0]);

            jac.dr1_drot() = skew(start_pose.rotation().rotp(
                end_pose.translation() - start_pose.translation() - 0.5 * GRAVITY * _dt * _dt));
            jac.dr2_drot() = -_dt * skew(start_pose.rotation().rotp(GRAVITY));
            jac.dr3_drot() = -log_jac * adjusted_state.gamma.R() * start_pose.rotation().R();

            jac.dr1_dpos() = -start_pose.rotation().R().matrix();
            jac.dr2_dpos().setZero();
            jac.dr3_dpos().setZero();

            jac.linearize_local_param(start_pose);
        }

        if (_jac[1])
        {
            PoseJacobian jac(_jac[1]);

            jac.dr1_drot().setZero();
            jac.dr2_drot().setZero();
            jac.dr3_drot() = log_jac * adjusted_state.gamma.R() * start_pose.rotation().R();

            jac.dr1_dpos() = start_pose.rotation().R().matrix();
            jac.dr2_dpos().setZero();
            jac.dr3_dpos().setZero();

            jac.linearize_local_param(end_pose);
        }

        if (_jac[2])
        {
            Map<Matrix<double, 9, 3, RowMajor>> jac(_jac[2]);

            jac.block<3, 3>(0, 0) = -Mat3::Identity() * _dt;
            jac.block<3, 3>(3, 0) = -Mat3::Identity();
            jac.block<3, 3>(6, 0).setZero();
        }

        if (_jac[3])
        {
            Map<Matrix<double, 9, 3, RowMajor>> jac(_jac[3]);

            jac.block<3, 3>(0, 0).setZero();
            jac.block<3, 3>(3, 0) = adjusted_state.gamma.R().transpose().matrix();
            jac.block<3, 3>(6, 0).setZero();
        }

        if (_jac[4])
        {
            Mat3 dexp;
            math::Quat<double>::exp(bias_adjustment.gamma, &dexp);
            Mat9 r_by_adjusted;
            r_by_adjusted.block<3, 3>(0, 0) = -Mat3::Identity();
            r_by_adjusted.block<3, 3>(3, 0).setZero();
            r_by_adjusted.block<3, 3>(6, 0).setZero();

            r_by_adjusted.block<3, 3>(0, 3).setZero();
            r_by_adjusted.block<3, 3>(3, 3) = -Mat3::Identity();
            r_by_adjusted.block<3, 3>(6, 3).setZero();

            r_by_adjusted.block<3, 3>(0, 6).setZero();
            r_by_adjusted.block<3, 3>(3, 6) = -adjusted_state.gamma.R().transpose() * skew(end_vel);
            r_by_adjusted.block<3, 3>(6, 6) = -(rot_error.Ad() * log_jac).transpose();

            Mat9 adjusted_by_state;
            adjusted_by_state.setZero();
            adjusted_by_state.block<3, 3>(0, 0).setIdentity();
            adjusted_by_state.block<3, 3>(3, 3).setIdentity();
            adjusted_by_state.block<3, 3>(6, 6) = dexp.transpose();

            Eigen::Map<Eigen::Matrix<double, 9, 6, RowMajor>> r_by_bias(_jac[4]);
            r_by_bias = r_by_adjusted * adjusted_by_state * dstate_dbias_;
        }
    }

    return true;
}

}  // namespace factors
}  // namespace mc
