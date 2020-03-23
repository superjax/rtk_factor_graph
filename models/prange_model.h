#pragma once

#include "common/measurements/gnss_observation.h"
#include "common/satellite/atm_correction.h"
#include "common/satellite/satellite.h"

namespace mc {
namespace models {

class PseudorangeModel
{
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PseudorangeModel(const meas::GnssObservation& obs,
                     const satellite::SatelliteBase& sat,
                     const Vec3& rec_pos_ecef,
                     const Mat3& cov,
                     const math::DQuat<double>& T_e2r);

    bool Evaluate(const double* const* parameters, double* residual, double** jacobians) const;

    void computeConstants(const Vec3& rec_pos_ecef);
    double computeSagnac(const Vec3& rec_pos_ecef);

 private:
    satellite::SatelliteState sat_state_;
    const satellite::SatelliteBase& sat_;
    satellite::AtmosphericCorrection atm_;
    double sagnac_;

    meas::GnssObservation obs_;
    Mat3 sqrt_inv_cov_;
    const math::DQuat<double>& T_e2r_;
};

}  // namespace models
}  // namespace mc
