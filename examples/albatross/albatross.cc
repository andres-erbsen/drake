#include "drake/examples/albatross/albatross.h"

#include "drake/common/default_scalars.h"
#include "drake/examples/albatross/gen/albatross_input.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/system_constraint.h"

namespace drake {
namespace examples {
namespace albatross {

template <typename T>
Albatross<T>::Albatross()
: systems::LeafSystem<T>(systems::SystemTypeTag<albatross::Albatross>{}) {
  // state: speed, pitch, yaw
  this->DeclareContinuousState(3, 0, 0);
  // input: lift coefficient, roll angle
  this->DeclareVectorInputPort(AlbatrossInput<T>());
}

template <typename T>
const systems::InputPortDescriptor<T>& Albatross<T>::get_input_port() const {
  return systems::System<T>::get_input_port(0);
}

template <typename T>
void Albatross<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  // TODO: parametrize
  // DS Kinetic 60
  const double m = 2;
  const double g = 10;
  const double cD0 = .005;
  const double k = .08;
  const double S = .23;
  const double rho = 1; // TODO real value

  const T speed = context.get_continuous_state().get_generalized_position().GetAtIndex(0);
  const T pitch = context.get_continuous_state().get_generalized_position().GetAtIndex(1);
  const T yaw   = context.get_continuous_state().get_generalized_position().GetAtIndex(2);
  //const T altitude = context.get_continuous_state().get_generalized_position().GetAtIndex(3);

  const T cL = this->EvalVectorInput(context, 0)->GetAtIndex(0);
  const T roll = this->EvalVectorInput(context, 0)->GetAtIndex(1);

  const T cD = cD0 + k*cL*cL;
  const T D = .5*cD*rho*S*speed*speed;
  const T L = .5*cL*rho*S*speed*speed;

  const T altitude_dot = speed*sin(pitch);
  const T Wd = 0*altitude_dot;

  derivatives->get_mutable_generalized_position().SetAtIndex(0, 1/(m)*(                              -D      - m*g*sin(pitch) + m*Wd*cos(pitch)*sin(yaw)));
  derivatives->get_mutable_generalized_position().SetAtIndex(1, 1/(m*(.0001+speed))*(            L*cos(roll) - m*g*cos(pitch) - m*Wd*sin(pitch)*sin(yaw)));
  derivatives->get_mutable_generalized_position().SetAtIndex(2, 1/(m*(.0001+speed)*cos(pitch))*( L*sin(roll)                  + m*Wd           *cos(yaw)));
  //derivatives->get_mutable_generalized_position().SetAtIndex(3, altitude_dot);
}

}  // namespace albatross
}  // namespace examples
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::examples::albatross::Albatross)
