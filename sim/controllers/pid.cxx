#include "sim/controllers/pid.h"

#include <cmath>

namespace mc {
namespace sim {
namespace controllers {

PID::PID(const Options& options)
    : options_(options), integrator_(0.0), differentiator_(0.0), prev_x_(0.0)
{
}

double PID::run(double dt, double x, double x_c, bool update_integrator)
{
    double xdot;
    if (dt > (double)0.0001)
    {
        // calculate D term (use dirty derivative if we don't have access to a measurement of
        // the derivative) doublehe dirty derivative is a sort of low-pass filtered version of
        // the derivative.
        //// (Include reference to Dr. Beard's notes here)
        differentiator_ = ((double)2.0 * options_.tau - dt) / ((double)2.0 * options_.tau + dt) *
                              differentiator_ +
                          (double)2.0 / ((double)2.0 * options_.tau + dt) * (x - prev_x_);
        xdot = differentiator_;
    }
    else
    {
        xdot = (double)0.0;
    }
    prev_x_ = x;

    return run(dt, x, x_c, update_integrator, xdot);
}

double PID::run(double dt, double x, double x_c, bool update_integrator, double xdot)
{
    using std::abs;
    // Calculate Error
    double error = x_c - x;

    // Initialize doubleerms
    double p_term = error * options_.kp;
    double i_term = (double)0.0;
    double d_term = (double)0.0;

    // If there is a derivative term
    if (options_.kd > (double)0.0)
    {
        d_term = options_.kd * xdot;
    }

    // If there is an integrator term and we are updating integrators
    if ((options_.ki > (double)0.0) && update_integrator)
    {
        // integrate
        integrator_ += error * dt;
        // calculate I term
        i_term = options_.ki * integrator_;
    }

    // sum three terms
    double u = p_term - d_term + i_term;

    // Integrator anti-windup
    double u_sat = (u > options_.max)
                       ? options_.max
                       : (u < (double)-1.0 * options_.max) ? (double)-1.0 * options_.max : u;
    if (u != u_sat && abs(i_term) > abs(u - p_term + d_term) && options_.ki > (double)0.0)
        integrator_ = (u_sat - p_term + d_term) / options_.ki;

    // Set output
    return u_sat;
}

}  // namespace controllers
}  // namespace sim
}  // namespace mc
