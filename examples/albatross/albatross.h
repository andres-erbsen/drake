#pragma once

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace albatross {

template <typename T>
class Albatross final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Albatross)

  Albatross();

  /// Scalar-converting copy constructor.
  template <typename U>
  explicit Albatross(const Albatross<U>&) : Albatross() {}

  /// Returns the output port containing the output configuration (only).
  const systems::OutputPort<T>& get_position_output_port() const {
    return this->get_output_port(0);
  }

  /// Returns the output port containing the full state.  This is
  /// provided primarily as a tool for debugging/visualization.
  const systems::OutputPort<T>& get_full_state_output_port() const {
    return this->get_output_port(1);
  }

 private:
  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  void CopyPositionToOutput(const systems::Context<T>& context,
                            systems::BasicVector<T>* output) const;

  void CopyFullStateToOutput(const systems::Context<T>& context,
                             systems::BasicVector<T>* output) const;
};

}  // namespace albatross
}  // namespace examples
}  // namespace drake
