#include "models/imu_model.h"

#include "common/check.h"
#include "common/defs.h"
#include "common/matrix_defs.h"
#include "common/quantized_time.h"

namespace mc {
namespace models {

using namespace Eigen;

static const Vec3 GRAVITY = 9.80665 * Vec3::UnitZ();

ImuModel::ImuModel(const meas::ImuSample& z0, const Vec6& bias, const Vec6& R) : R_(R)
{
    bias_ = bias;

    history_.emplace_back();
    history_.back().z = z0;
    state().setIdentity();
    covariance().setZero();
    dStatedBias().setZero();
}

void ImuModel::errorStateDynamics(const ImuState& state,
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

void ImuModel::dynamics(const ImuState& state,
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

Error ImuModel::integrate(const meas::ImuSample& imu)
{
    check(isFinite(R_), "NaN detected in covariance on propagation");
    check(isFinite(imu.accel), "NaN detected in imu acceleration data");
    check(isFinite(imu.gyro), "NaN detected in imu gyroscope data");

    double dt = (imu.t - tf()).toSec();
    check(dt >= TIME_QUANTIZATION,
          "IMU Samples must be in order, and spaced more than {}s apart (dt = {})",
          fmt(TIME_QUANTIZATION, dt));

    ImuErrorState dstate;
    Mat9 A;
    Mat96 B;

    meas::ImuSample avg_imu;
    avg_imu.accel = (imu.accel + history_.back().z.accel) / 2.0;
    avg_imu.gyro = (imu.gyro + history_.back().z.gyro) / 2.0;

    dynamics(state(), avg_imu, Out(dstate), Out(A), Out(B));

    const Mat9 Adt = A * dt;

    A = Mat9::Identity() + Adt * (Mat9::Identity() + 0.5 * Adt);
    B = B * dt;

    History history_item;
    history_item.z = imu;
    history_item.state = state() + (dstate * dt);
    history_item.covariance =
        A * covariance() * A.transpose() + B * R_.asDiagonal() * B.transpose();
    history_item.dstate_dbias = A * dStatedBias() + B;
    history_.emplace_back(std::move(history_item));

    check(isFinite(A), "NaN detected in state transition matrix on propagation");
    check(isFinite(covariance()), "NaN detected in covariance on propagation");

    return Error::none();
}

Error ImuModel::finished()
{
    check(numUpdates() > 0, "finished called on empty IMU model");

    if (numUpdates() <= 2)
    {
        warn("less than two IMU measurements in integrator");
        covariance() = covariance() + Mat9::Identity() * 1e-15;
    }
    covariance_inv_sqrt_ = covariance().inverse().llt().matrixL().transpose();

    check(isFinite(covariance_inv_sqrt_), "Nan detected in IMU information matrix.");
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

bool ImuModel::Evaluate(const double* const* parameters, double* _res, double** _jac) const
{
    const math::DQuat<double> start_pose(parameters[0]);
    const math::DQuat<double> end_pose(parameters[1]);
    const Map<const Vec3> start_vel(parameters[2]);
    const Map<const Vec3> end_vel(parameters[3]);
    const Map<const Vec3> accel_bias(parameters[4]);
    const Map<const Vec3> gyro_bias(parameters[4] + 3);
    const Map<const Vec6> bias(parameters[4]);

    Map<Vec9> residual(_res);

    const ImuErrorState bias_adjustment = dStatedBias() * bias;  //(bias - bias_);
    ImuState adjusted_state = state() + bias_adjustment;

    const double dt = delta_t();

    residual.segment<3>(0) =
        start_pose.rotation().rotp(end_pose.translation() - start_pose.translation() -
                                   0.5 * GRAVITY * dt * dt) -
        start_vel * dt - adjusted_state.alpha;
    residual.segment<3>(3) = adjusted_state.gamma.rota(end_vel) - start_vel -
                             start_pose.rotation().rotp(GRAVITY) * dt - adjusted_state.beta;

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
                end_pose.translation() - start_pose.translation() - 0.5 * GRAVITY * dt * dt));
            jac.dr2_drot() = -dt * skew(start_pose.rotation().rotp(GRAVITY));
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

            jac.block<3, 3>(0, 0) = -Mat3::Identity() * dt;
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
            r_by_bias = r_by_adjusted * adjusted_by_state * dStatedBias();
        }
    }

    return true;
}

Error ImuModel::computeEndState(const math::DQuat<double>& start,
                                const Vec3& start_vel,
                                Out<math::DQuat<double>> end,
                                Out<Vec3> end_vel) const
{
    check(std::abs(1.0 - state().gamma.arr_.norm()) < 1e-8, "Itegrated rotation left manifold.");
    check(std::abs(1.0 - start.real().norm()) < 1e-8, "start pose rotation left manifold.");
    check(tf() >= t0(), "Imu integration time going backwards.");

    if (tf() == t0())
    {
        *end = start;
        *end_vel = start_vel;
        return Error::create("Zero integration time");
    }

    const auto& pose_i = start;
    const auto& vel_i = start_vel;
    const double dt = delta_t();

    // TODO: this could probably be significantly optimized
    const Vec3 translation_j = pose_i.translation() +
                               pose_i.rotation().rota(vel_i * dt + state().alpha) +
                               0.5 * GRAVITY * dt * dt;
    const math::Quat<double> rot_j = pose_i.rotation() * state().gamma;
    *end = math::DQuat<double>(rot_j, translation_j);
    *end_vel = state().gamma.rotp(vel_i + pose_i.rotation().rotp(GRAVITY) * dt + state().beta);
    return Error::none();
}

ImuModel ImuModel::split(const UTCTime& t)
{
    const auto t_split = t.quantized();

    check(t_split > t0() && t_split < tf(), "split time must be in interval");

    auto it = history_.begin();

    while (it->z.t <= t_split)
    {
        ++it;
    }

    auto this_end = it;
    auto prev_it = (it - 1);
    double dt = (it->z.t - prev_it->z.t).toSec();
    double l = (t - prev_it->z.t).toSec();
    double scale = l / dt;

    meas::ImuSample split_sample;
    split_sample.accel = scale * it->z.accel + (1.0 - scale) * prev_it->z.accel;
    split_sample.gyro = scale * it->z.gyro + (1.0 - scale) * prev_it->z.gyro;
    split_sample.t = t_split;

    ImuModel second(split_sample, bias_, R_);
    while (it != history_.end())
    {
        second.integrate(it->z);
        ++it;
    }

    // Remove the history that was transferred to the new integrator
    history_.erase(this_end, history_.end());

    // If the split didn't happen directly on a sample boundary, then re-do the last sample in the
    // series so the end time is correct
    if (t_split != history_.back().z.t)
    {
        integrate(split_sample);
    }

    return second;
}

}  // namespace models
}  // namespace mc
