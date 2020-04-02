#include <gtest/gtest.h>

#include "core/factor_graph.h"

namespace mc {
namespace core {

TEST(FactorGraph, Compile)
{
    FactorGraph::Options options;
    FactorGraph graph(options);
}

}  // namespace core
}  // namespace mc
