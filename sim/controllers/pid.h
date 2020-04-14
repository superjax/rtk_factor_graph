#pragma once

#include <limits>

namespace mc {
namespace sim {
namespace controllers {

class PID
{
 public:
    struct Options
    {
        double kp = 1.0;
        double ki = 0.1;
        double kd = 0.1;
        double max = std::numeric_limits<double>::max();
        double tau = 0.05;
    };

    PID(const Options& options);

    double run(double dt, double x, double x_c, bool update_integrator);
    double run(double dt, double x, double x_c, bool update_integrator, double xdot);

    Options options_;
    double integrator_;
    double differentiator_;
    double prev_x_;
};

}  // namespace controllers
}  // namespace sim
}  // namespace mc
