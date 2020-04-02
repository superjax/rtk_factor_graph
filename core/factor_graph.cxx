#include "core/factor_graph.h"

namespace mc {
namespace core {

FactorGraph::FactorGraph(const FactorGraph::Options& options) : options_(options)
{
    nodes_.emplace_back();
    nodes_.back().t = options_.t0_;
    nodes_.back().T_n2b = options_.x0_;
    nodes_.back().vel = options_.vel0_;
    nodes_.back().clk = options_.clk0_;
    for (auto& k : nodes_.back().sw)
    {
        k = 1.0;
    }

    imu_bias_ = options.imu_bias0_;
    p_b2g_ = options.p_b2g0_;
    T_r2n_ = options.T_r2n0_;
}
}  // namespace core
}  // namespace mc
