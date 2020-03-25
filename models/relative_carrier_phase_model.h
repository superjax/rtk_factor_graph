#pragma once

#include "common/measurements/gnss_observation.h"
#include "common/out.h"
#include "common/satellite/satellite.h"

#include "common/math/dquat.h"

namespace mc {
namespace models {

class RelativeCarrierPhase
{
 public:
    RelativeCarrierPhase(const meas::GnssObservation& obs1,
                         const meas::GnssObservation& obs2,
                         const satellite::SatelliteBase& sat,
                         const Vec3& rec_pos_ecef,
                         const math::DQuat<double>& T_e2r,
                         const double& Xi);

    double computeSagnac(const satellite::SatelliteState& sat_ecef, const Vec3& rec_pos_ecef) const;
    void computeSatState(const meas::GnssObservation& obs,
                         const Vec3& rec_pos_ecef,
                         const satellite::SatelliteBase& sat,
                         Out<satellite::SatelliteState> sat_state) const;

    bool Evaluate(const double* const* parameters, double* residual, double** jacobians) const;

 private:
    const double dPhi_;    // change in carrier phase (cycles)
    const double lambda_;  // wavelength of carrier-phase signal (meters)
    const double Xi_;      // sqrt inverse variance
    satellite::SatelliteState sat1_;
    satellite::SatelliteState sat2_;
    const math::DQuat<double>& T_e2r_;
};

}  // namespace models
}  // namespace mc
