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

  const int N = 101;
  const double dt_min = .03;
  const double dt_max = .3;

  systems::trajectory_optimization::DirectCollocation dircol(
      albatross.get(), *context, N, dt_min, dt_max);
  dircol.AddEqualTimeIntervalsConstraints();

  //const Eigen::Vector4d initial_state(25, 0, 0, 0);
  //dircol.AddLinearConstraint(dircol.initial_state() == initial_state);
  dircol.AddLinearConstraint(dircol.initial_state()(1) == 0); // no pitch
  dircol.AddLinearConstraint(dircol.initial_state()(3) == 0); // altitude 0
  dircol.AddConstraintToAllKnotPoints(dircol.state()(3) >= 0); // other altitudes higher

  //auto uu = dircol.NewContinuousVariables(1, "uu");
  //dircol.AddConstraintToAllKnotPoints(dircol.input()(0) == uu(0));
  //auto rr = dircol.NewContinuousVariables(1, "rr");
  //dircol.AddConstraintToAllKnotPoints(dircol.input()(1) == rr(0));

  dircol.AddConstraintToAllKnotPoints(dircol.state()(0) >= 13); // characteristic speed from 2017 paper
  dircol.AddConstraintToAllKnotPoints(dircol.state()(0) <= 60);

  dircol.AddConstraintToAllKnotPoints(dircol.state()(1) >= -acos(0.));
  dircol.AddConstraintToAllKnotPoints(dircol.state()(1) <= acos(0.));

  dircol.AddConstraintToAllKnotPoints(dircol.state()(2) >= -6*acos(0.));
  dircol.AddConstraintToAllKnotPoints(dircol.state()(2) <= 6*acos(0.));

  dircol.AddConstraintToAllKnotPoints(dircol.state()(3) >= -100);
  dircol.AddConstraintToAllKnotPoints(dircol.state()(3) <= 100);

  dircol.AddConstraintToAllKnotPoints(dircol.input()(0) >= 0);
  dircol.AddConstraintToAllKnotPoints(dircol.input()(0) <= 1.2);

  dircol.AddConstraintToAllKnotPoints(dircol.input()(1) >= -acos(0.)/4);
  dircol.AddConstraintToAllKnotPoints(dircol.input()(1) <= acos(0.)/4);

  dircol.AddConstraintToAllKnotPoints(dircol.input()(2) == 1);

  printf("num_vars = %d\n", dircol.num_vars());

  {
    auto result = dircol.Solve();
    if (result != solvers::SolutionResult::kSolutionFound) {
      fprintf(stderr, "solving failed!\n");
      return 1;
    }

    auto inputs = dircol.ReconstructInputTrajectory();
    auto traj = dircol.ReconstructStateTrajectory();
    auto timestamps = traj.get_segment_times();
    for (size_t i = 0; i < timestamps.size(); i++) {
      auto t = timestamps[i];
      auto V = traj.value(timestamps[i]).coeff(0);
      auto pitch = traj.value(timestamps[i]).coeff(1);
      auto yaw = traj.value(timestamps[i]).coeff(2);
      auto altitude = traj.value(timestamps[i]).coeff(3);
      auto cL = inputs.value(timestamps[i]).coeff(0);
      auto roll = inputs.value(timestamps[i]).coeff(1);
      printf("%3.3f: %3.3f %3.3f %3.3f %3.3f <<- %3.3f %3.3f | %3.3f\n", t, V, pitch, yaw, altitude, cL, roll, 9.8*altitude + .5*V*V);
    }
    printf("\n");
  }

  {
    dircol.SetInitialTrajectory(dircol.ReconstructInputTrajectory(), dircol.ReconstructStateTrajectory());

    dircol.AddRunningCost(1/N*dircol.input()(0)*dircol.input()(0));
    dircol.AddRunningCost(1/N*dircol.input()(1)*dircol.input()(1));
    dircol.AddFinalCost(-(9.8*dircol.state()(3) + .5*dircol.state()(0)*dircol.state()(0)) ); // m(gh + .5vv)
    //dircol.AddFinalCost(1000.0*dircol.time());
    dircol.AddFinalCost((dircol.state()(0) - dircol.initial_state()(0))*(dircol.state()(0) - dircol.initial_state()(0)));
    dircol.AddFinalCost(10*(dircol.state()(1) - dircol.initial_state()(1))*(dircol.state()(1) - dircol.initial_state()(1)));
    dircol.AddFinalCost(500*(dircol.state()(2)+4*acos(0) - dircol.initial_state()(2))*(dircol.state()(2)+4*acos(0) - dircol.initial_state()(2)));
    dircol.AddFinalCost((dircol.state()(3) - dircol.initial_state()(3))*(dircol.state()(3) - dircol.initial_state()(3)));

    auto result = dircol.Solve();
    if (result != solvers::SolutionResult::kSolutionFound) {
      fprintf(stderr, "optimization failed!\n");
      return 1;
    }

    auto inputs = dircol.ReconstructInputTrajectory();
    auto traj = dircol.ReconstructStateTrajectory();
    auto timestamps = traj.get_segment_times();
    for (size_t i = 0; i < timestamps.size(); i++) {
      auto t = timestamps[i];
      auto V = traj.value(timestamps[i]).coeff(0);
      auto pitch = traj.value(timestamps[i]).coeff(1);
      auto yaw = traj.value(timestamps[i]).coeff(2);
      auto altitude = traj.value(timestamps[i]).coeff(3);
      auto cL = inputs.value(timestamps[i]).coeff(0);
      auto roll = inputs.value(timestamps[i]).coeff(1);
      printf("%3.3f: %3.3f %3.3f %3.3f %3.3f <<- %3.3f %3.3f | %3.3f\n", t, V, pitch, yaw, altitude, cL, roll, 9.8*altitude + .5*V*V);
    }
    printf("\n");
  }

  {
    dircol.SetInitialTrajectory(dircol.ReconstructInputTrajectory(), dircol.ReconstructStateTrajectory());

    dircol.AddConstraint(dircol.initial_state()(0) <= dircol.final_state()(0));
    //dircol.AddConstraint(dircol.initial_state()(1) == dircol.final_state()(1));
    dircol.AddConstraint(dircol.initial_state()(2) == dircol.final_state()(2) + 4*acos(0));
    dircol.AddConstraint(dircol.initial_state()(3) <= dircol.final_state()(3));

    auto result = dircol.Solve();
    if (result != solvers::SolutionResult::kSolutionFound) {
      fprintf(stderr, "constraining failed!\n");
      return 1;
    }

    auto inputs = dircol.ReconstructInputTrajectory();
    auto traj = dircol.ReconstructStateTrajectory();
    auto timestamps = traj.get_segment_times();
    for (size_t i = 0; i < timestamps.size(); i++) {
      auto t = timestamps[i];
      auto V = traj.value(timestamps[i]).coeff(0);
      auto pitch = traj.value(timestamps[i]).coeff(1);
      auto yaw = traj.value(timestamps[i]).coeff(2);
      auto altitude = traj.value(timestamps[i]).coeff(3);
      auto cL = inputs.value(timestamps[i]).coeff(0);
      auto roll = inputs.value(timestamps[i]).coeff(1);
      printf("%3.3f: %3.3f %3.3f %3.3f %3.3f <<- %3.3f %3.3f | %3.3f\n", t, V, pitch, yaw, altitude, cL, roll, 9.8*altitude + .5*V*V);
    }
    printf("\n");
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
