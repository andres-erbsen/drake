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

  const systems::InputPortDescriptor<T>& get_input_port() const;

 private:
  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;
};

}  // namespace albatross
}  // namespace examples
}  // namespace drake
