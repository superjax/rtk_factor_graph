#pragma once

#include "common/circular_buffer.h"
#include "common/error.h"
#include "core/factors/factors.h"
#include "core/imu_bias.h"
#include "core/node.h"
#include "core/solver/solver.h"
#include "models/gnss_dynamics_model.h"
#include "models/imu_model.h"
#include "models/pose_error_model.h"
#include "models/prange_model.h"
#include "models/relative_carrier_phase_model.h"

namespace mc {
namespace core {

class FactorGraph
{
 public:
    struct Options
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        // Model weights
        Vec6 Xi_r2n_;
    };

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FactorGraph();

    Error build();
    Error solve();

 private:
    static constexpr int MAX_STATE_LEN = 25;
    static constexpr int MAX_SAT = 30;

    // Allocate space for all our variables
    CircularBuffer<Node, MAX_STATE_LEN + 1> nodes_;
    Vec3 p_b2g_;
    ImuBias imu_bias_;
    math::DQuat<double> T_r2n_;

    // Allocate space for the functors

    Options options_;

    std::unique_ptr<Solver> solver;
};

}  // namespace core
}  // namespace mc
