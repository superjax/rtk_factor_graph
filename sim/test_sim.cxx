#include <gtest/gtest.h>

#include "sim/sim.h"

namespace mc {
namespace sim {

TEST(Sim, Compile)
{
    Sim::Options options;
    Sim sim(options);
}

}  // namespace sim
}  // namespace mc
