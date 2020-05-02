#include <gtest/gtest.h>

#include "sim/sim.h"

namespace mc {
namespace sim {

TEST(Sim, Compile)
{
    Sim::Options options;
    options.wp_options.waypoints = {{1, 1, 0}};
    Sim sim(options);
}

}  // namespace sim
}  // namespace mc
