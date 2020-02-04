#pragma once

#include "Angle.h"
#include "Velocity.h"

template <std::int32_t friction_, std::int32_t acceleration_>
class Simulate
{
public:
  static void friction(Velocity& v);
  static void accelerate(Velocity& v, Angle yaw);

  static constexpr float s_friction     = friction_;
  static constexpr float s_acceleration = acceleration_;
};

extern template class Simulate<6, 10>;
extern template class Simulate<0, 1>;

using Ground = Simulate<6, 10>;
using Air    = Simulate<0, 1>;
