#include <cstdio>
#include <memory>

#include "drake/examples/albatross/albatross.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"

namespace drake {
namespace examples {
namespace albatross {
namespace {

int trajectory_optimization() {
  printf("albatross trajectory optimization...\n");

  auto albatross = std::make_unique<Albatross<double>>();
  // albatross->set_name("albatross");
  auto context = albatross->CreateDefaultContext();

  const int kNumTimeSamples = 41;
  const double kMinimumTimeStep = 6./(kNumTimeSamples-1);
  const double kMaximumTimeStep = 7.5/(kNumTimeSamples-1);
  systems::trajectory_optimization::DirectCollocation dircol(
      albatross.get(), *context, kNumTimeSamples, kMinimumTimeStep,
      kMaximumTimeStep);
  dircol.AddEqualTimeIntervalsConstraints();

  const Eigen::Vector2d initial_state(-0.1144, 2.0578);
  dircol.AddLinearConstraint(dircol.initial_state()(0) <= initial_state(0)/2);
  dircol.AddLinearConstraint(dircol.initial_state()(1) >= initial_state(1)/2);
  dircol.AddLinearConstraint(dircol.final_state()(0) - dircol.initial_state()(0) <= 5);
  dircol.AddLinearConstraint(dircol.final_state()(0) - dircol.initial_state()(0) >= -5);
  dircol.AddLinearConstraint(dircol.final_state()(1) - dircol.initial_state()(1) <= 5);
  dircol.AddLinearConstraint(dircol.final_state()(1) - dircol.initial_state()(1) >= -5);

  printf("num_vars = %d\n", dircol.num_vars());

  auto result = dircol.Solve();
  if (result != solvers::SolutionResult::kSolutionFound) {
    fprintf(stderr, "optimization failed!\n");
    return 1;
  }

  auto traj = dircol.ReconstructStateTrajectory();
  auto t = traj.get_segment_times();
  for (size_t i = 0; i < t.size(); i++) {
    printf("%3.3f: %3.3f %3.3f\n", t[i], traj.value(t[i]).coeff(0), traj.value(t[i]).coeff(1));
  }

  return 0;
}

}  // namespace
}  // namespace albatross
}  // namespace examples
}  // namespace drake

int main(int argc, char** argv) {
  return drake::examples::albatross::trajectory_optimization();
}

