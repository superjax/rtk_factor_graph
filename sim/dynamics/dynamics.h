#pragma once

#include "common/math/two_jet.h"
#include "common/matrix_defs.h"
#include "common/out.h"
#include "common/utctime.h"

namespace mc {
namespace sim {
namespace dynamics {

class Dynamics
{
 public:
    struct Options
    {
        double mass = 1.0;
        Vec3 inertia_ = (Vec3() << 1, 1, 1).finished();
    };

    Dynamics(const UTCTime& t, const Options& options);

    virtual void compute_wrench(const math::TwoJet<double>& x,
                                const Eigen::VectorXd& u,
                                Out<typename math::DQuat<double>::TangentVector> wrench) const = 0;

    void step(const UTCTime& t, const Eigen::VectorXd& u);
    void f(const math::TwoJet<double>& x,
           const Eigen::VectorXd& u,
           Out<typename math::DQuat<double>::TangentVector> d2x) const;
    const UTCTime& t() const { return prev_t_; }

    math::TwoJet<double> x;

 protected:
    Options options_;
    UTCTime prev_t_;
};

}  // namespace dynamics
}  // namespace sim
}  // namespace mc
