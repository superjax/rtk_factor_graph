#include <gtest/gtest.h>

#include "core/estimator.h"

namespace mc {
namespace core {

TEST(Estimator, compile)
{
    Estimator::Options options;
    Estimator estimator(options);
}

}  // namespace core
}  // namespace mc
