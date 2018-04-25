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
  auto context = albatross->CreateDefaultContext();

  const int N = 21;
  const double dt_min = .05;
  const double dt_max = .5;

  systems::trajectory_optimization::DirectCollocation dircol(
      albatross.get(), *context, N, dt_min, dt_max);
  dircol.AddEqualTimeIntervalsConstraints();

  const Eigen::Vector3d initial_state(25, 0, 0);
  dircol.AddLinearConstraint(dircol.initial_state() == initial_state);

  dircol.AddConstraintToAllKnotPoints(dircol.state()(1) >= 0);

  dircol.AddConstraintToAllKnotPoints(dircol.input()(0) >= 0);
  dircol.AddConstraintToAllKnotPoints(dircol.input()(0) <= 1.2);

  dircol.AddRunningCost(dircol.input()(0));

  printf("num_vars = %d\n", dircol.num_vars());

  auto result = dircol.Solve();
  if (result != solvers::SolutionResult::kSolutionFound) {
    fprintf(stderr, "optimization failed!\n");
    return 1;
  }

  auto inputs = dircol.ReconstructInputTrajectory();
  auto traj = dircol.ReconstructStateTrajectory();
  auto t = traj.get_segment_times();
  for (size_t i = 0; i < t.size(); i++) {
    printf("%3.3f: %3.3f %3.3f %3.3f <<- %3.3f\n", t[i], traj.value(t[i]).coeff(0), traj.value(t[i]).coeff(1), traj.value(t[i]).coeff(2), inputs.value(t[i]).coeff(0));
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

