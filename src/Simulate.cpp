#include "Simulate.h"

#include "Constants.h"
#include "UsrCmd.h"

#include <array>

#define SHORT2ANGLE(x) ((x) * (360.0 / 65536))

namespace
{
static std::array<std::array<float, 2>, 65536> cosSinTable = [] {
  std::array<std::array<float, 2>, 65536> table;
  for (int yaw = 0; yaw < 65536; ++yaw)
  {
    // PM_UpdateViewAngles
    float const yaw_DEG = SHORT2ANGLE(yaw);

    float const yaw_RAD = yaw_DEG * (M_PI * 2 / 360);
    float const sy      = std::sin(yaw_RAD);
    float const cy      = std::cos(yaw_RAD);
    table[yaw]          = { cy, sy };
  }
  return table;
}();
} // namespace

template class Simulate<6, 10>;
template class Simulate<0, 1>;

template <std::int32_t friction_, std::int32_t acceleration_>
void Simulate<friction_, acceleration_>::friction(Velocity& v)
{
  if constexpr (s_friction == 0) return;

  float const speed = v.VectorLength();
  if (speed < 1)
  {
    v = 0;
    return;
  }

  // apply ground friction
  float const control = speed < pm_stopspeed ? pm_stopspeed : speed;
  float const drop    = control * s_friction * frametime;

  // scale the velocity
  float newspeed = speed - drop;
  if (newspeed < 0) newspeed = 0;
  newspeed /= speed;

  v *= newspeed;
}

template <std::int32_t friction_, std::int32_t acceleration_>
void Simulate<friction_, acceleration_>::accelerate(Velocity& v, Angle yaw)
{
  // AngleVectors
  auto const& lc_CosSinTable = cosSinTable[yaw.value()];
  auto const& forward_x      = lc_CosSinTable[0];
  auto const& forward_y      = lc_CosSinTable[1];
  auto const& right_x        = forward_y;
  auto const& right_y        = -forward_x;
  // forward[2] = 0;
  // right[2] = 0;

  constexpr float const scale = cmd.PM_CmdScale();

  // forward.VectorNormalize();
  // right.VectorNormalize();

#ifndef FM_ONLY
  Array wishdir(
    forward_x * cmd.forwardmove + right_x * cmd.rightmove, forward_y * cmd.forwardmove + right_y * cmd.rightmove);
  float wishspeed = wishdir.VectorNormalize();
  wishspeed *= scale;

  // clamp the speed lower if ducking
  if constexpr (cmd.upmove < 0)
  {
    if (wishspeed > 320 * pm_duckScale)
    {
      wishspeed = 320 * pm_duckScale;
    }
  }
#else
  auto const&     wishdir   = lc_CosSinTable;
  constexpr float wishspeed = [&] {
    float const wishspeed = cmd.forwardmove * scale;
    // clamp the speed lower if ducking
    if (cmd.upmove < 0 && wishspeed > 320 * pm_duckScale)
    {
      return 320 * pm_duckScale;
    }
    return wishspeed;
  }();
#endif

  // PM_Accelerate
  {
    // q2 style
    float const currentspeed = v[0] * wishdir[0] + v[1] * wishdir[1];
    float const addspeed     = wishspeed - currentspeed;
    if (addspeed <= 0)
    {
      return;
    }
    float accelspeed = s_acceleration * frametime * wishspeed;
    if (accelspeed > addspeed)
    {
      accelspeed = addspeed;
    }

    v[0] += accelspeed * wishdir[0];
    v[1] += accelspeed * wishdir[1];
  }

  /*
  float const vel = VectorLength();

  VectorNormalize();
  *this *= vel;
  */
}
