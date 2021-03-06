#pragma once

#include "drake/common/constants.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_optional.h"
#include "drake/systems/framework/framework_common.h"

namespace drake {
namespace systems {

template <typename T>
class System;

/// InputPortDescriptor is a notation for specifying the kind of input a
/// System accepts, on a given port. It is not a mechanism for handling any
/// actual input data.
///
/// @tparam T The mathematical type of the context, which must be a valid Eigen
///           scalar.
template <typename T>
class InputPortDescriptor {
 public:
  /// @param system The system to which this descriptor belongs.
  /// @param index The index of the input port described, starting from zero and
  ///              incrementing by one per port.
  /// @param data_type Whether the port described is vector or abstract valued.
  /// @param size If the port described is vector-valued, the number of
  ///             elements, or kAutoSize if determined by connections.
  /// @param random_type Input ports may optionally be labeled as random, if the
  ///                    port is intended to model a random-source "noise" or
  ///                    "disturbance" input.
  InputPortDescriptor(const System<T>* system, InputPortIndex index,
                      PortDataType data_type, int size,
                      const optional<RandomDistribution>& random_type)
      : system_(system),
        index_(index),
        data_type_(data_type),
        size_(size),
        random_type_(random_type) {
    if (size_ == kAutoSize) {
      DRAKE_ABORT_MSG("Auto-size ports are not yet implemented.");
    }
    if (is_random() && data_type_ != kVectorValued) {
      DRAKE_ABORT_MSG("Random input ports must be vector valued.");
    }
  }

  /// @name Basic Concepts
  /// MoveConstructible only; not CopyConstructible; not Copy/Move-Assignable.
  /// @{
  //
  // Implementation note: This class aliases a pointer to the system that
  // contains it and captures its own index within that system's port vector,
  // so we must be careful not to allow C++ copying to extend the lifetime of
  // the system alias or duplicate our claim to the index.  Thus, this class is
  // `MoveConstructible` but neither copyable nor assignable: it supports move
  // to populate a vector, but is non-copyable in order remain the "one true
  // descriptor" after construction and non-assignable in order to remain
  // const.  Code that wishes to refer to this descriptor after insertion into
  // the vector should use a reference (not copy) of this descriptor.
  InputPortDescriptor() = delete;
  InputPortDescriptor(InputPortDescriptor&& other) = default;
  InputPortDescriptor(const InputPortDescriptor&) = delete;
  InputPortDescriptor& operator=(InputPortDescriptor&&) = delete;
  InputPortDescriptor& operator=(const InputPortDescriptor&) = delete;
  ~InputPortDescriptor() = default;
  /// @}

  const System<T>* get_system() const { return system_; }
  InputPortIndex get_index() const { return index_; }
  PortDataType get_data_type() const { return data_type_; }
  int size() const { return size_; }
  bool is_random() const { return static_cast<bool>(random_type_); }
  optional<RandomDistribution> get_random_type() const { return random_type_; }

 private:
  const System<T>* const system_;
  const InputPortIndex index_;
  const PortDataType data_type_;
  const int size_;
  const optional<RandomDistribution> random_type_;
};

}  // namespace systems
}  // namespace drake
