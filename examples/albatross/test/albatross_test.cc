#include "drake/examples/albatross/albatross.h"

#include <gtest/gtest.h>

#include "drake/systems/framework/test_utilities/scalar_conversion.h"

namespace drake {
namespace examples {
namespace albatross {
namespace {

GTEST_TEST(AlbatrossTest, ScalarConversionTest) {
  Albatross<double> alb;
  EXPECT_TRUE(is_autodiffxd_convertible(alb));
  EXPECT_TRUE(is_symbolic_convertible(alb));
}

}  // namespace
}  // namespace albatross
}  // namespace examples
}  // namespace drake
