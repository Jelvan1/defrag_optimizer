#include "Node.h"

#include "Constants.h"
#include "Matrix.h"
#include "Simulate.h"

#include <cassert>
#include <cmath>

namespace
{
void add_neighbour(std::vector<std::pair<Velocity, Angle>>& neighbours, Velocity const& neighbour, std::int32_t yaw)
{
  // // (-vfy, vfx) * neighbour
  // float const dot = -vf[1] * neighbour[0] + vf[0] * neighbour[1];
  // if (dot >= 0)
  // {
  neighbours.emplace_back(neighbour, yaw);
  // }
}

template <std::int32_t... args>
void bisection(
  Velocity const&                          vf,
  int                                      vx_min,
  int                                      vy_min,
  StaticMatrix<bool, args...>&             visited,
  std::vector<std::pair<Velocity, Angle>>& neighbours,
  Velocity const&                          min,
  Velocity const&                          max,
  std::int32_t                             yaw_min,
  std::int32_t                             yaw_max)
{
  assert(yaw_min < yaw_max);
  auto const yaw_range = yaw_max - yaw_min;
  assert(yaw_range % 2 == 0);
  auto const yaw = yaw_range / 2 + yaw_min;

  Velocity neighbour = vf;
  Ground::accelerate(neighbour, Angle(yaw));
  neighbour.snap();

  if (yaw_range == 2)
  {
    if (auto& entry = visited.value(static_cast<int>(neighbour[0]) - vx_min, static_cast<int>(neighbour[1]) - vy_min);
        !entry)
    {
      entry = true;
      add_neighbour(neighbours, neighbour, yaw);
    }
    // Visited all angles.
    return;
  }

  if (neighbour == min)
  {
    bisection(vf, vx_min, vy_min, visited, neighbours, min, max, yaw, yaw_max);
  }
  else if (neighbour == max)
  {
    bisection(vf, vx_min, vy_min, visited, neighbours, min, max, yaw_min, yaw);
  }
  else
  {
    if (auto& entry = visited.value(static_cast<int>(neighbour[0]) - vx_min, static_cast<int>(neighbour[1]) - vy_min);
        !entry)
    {
      entry = true;
      add_neighbour(neighbours, neighbour, yaw);
    }
    bisection(vf, vx_min, vy_min, visited, neighbours, min, neighbour, yaw_min, yaw);
    bisection(vf, vx_min, vy_min, visited, neighbours, neighbour, max, yaw, yaw_max);
  }
}
} // namespace

std::vector<std::pair<Velocity, Angle>> Node::neighbours() const
{
  std::vector<std::pair<Velocity, Angle>> neighbours;

  Velocity vf = m_v;
  Ground::friction(vf);

  constexpr float a      = 320 * Ground::s_acceleration * frametime;
  int const       vx_min = std::rint(vf[0] - a);
  int const       vx_max = std::rint(vf[0] + a);
  int const       vy_min = std::rint(vf[1] - a);
  int const       vy_max = std::rint(vf[1] + a);

  constexpr int a_int = 2 + 2 * std::round(a);
  assert(a_int >= (vx_max - vx_min + 1));
  assert(a_int >= (vy_max - vy_min + 1));
  assert(a_int <= 100);
  StaticMatrix<bool, 0, a_int - 1, 0, a_int - 1> visited;

  float const d_vel = std::atan2(vf[1], vf[0]);
  float const d_opt = std::acos((320 - a) / vf.VectorLength());

  // Use the bisection method to find (almost) all neighbours.
  // Start searching from the velocity direction such that we don't miss that one either.
  std::int32_t init = std::round(d_vel * 32768 / M_PI);

  Velocity neighbour = vf;
  Ground::accelerate(neighbour, Angle(init));
  neighbour.snap();

  {
    auto& entry = visited.value(static_cast<int>(neighbour[0]) - vx_min, static_cast<int>(neighbour[1]) - vy_min);
    entry       = true;
    add_neighbour(neighbours, neighbour, init);
  }
  bisection(vf, vx_min, vy_min, visited, neighbours, neighbour, neighbour, init, init + 65536);

  // The bisection method can miss the neighbours around the optimal accel angles.
  // Positive optimal accel angle.
  std::int32_t const pos_opt = std::ceil((d_vel + d_opt) * 32768 / M_PI);
  neighbour                  = vf;
  Ground::accelerate(neighbour, Angle(pos_opt));
  neighbour.snap();

  if (auto& entry = visited.value(static_cast<int>(neighbour[0]) - vx_min, static_cast<int>(neighbour[1]) - vy_min);
      !entry)
  {
    entry = true;
    add_neighbour(neighbours, neighbour, pos_opt);
  }
  // Negative optimal accel angle.
  std::int32_t const neg_opt = std::floor((d_vel - d_opt) * 32768 / M_PI);
  neighbour                  = vf;
  Ground::accelerate(neighbour, Angle(neg_opt));
  neighbour.snap();

  if (auto& entry = visited.value(static_cast<int>(neighbour[0]) - vx_min, static_cast<int>(neighbour[1]) - vy_min);
      !entry)
  {
    entry = true;
    add_neighbour(neighbours, neighbour, neg_opt);
  }

  return neighbours;
}

std::vector<std::pair<Velocity, Angle>> Node::neighbours2() const
{
  std::vector<std::pair<Velocity, Angle>> neighbours;

  Velocity vf = m_v;
  Ground::friction(vf);

  constexpr float a      = 320 * Ground::s_acceleration * frametime;
  int const       vx_min = std::rint(vf[0] - a);
  int const       vx_max = std::rint(vf[0] + a);
  int const       vy_min = std::rint(vf[1] - a);
  int const       vy_max = std::rint(vf[1] + a);

  constexpr int a_int = 2 + 2 * std::round(a);
  assert(a_int >= (vx_max - vx_min + 1));
  assert(a_int >= (vy_max - vy_min + 1));
  assert(a_int <= 100);
  StaticMatrix<bool, 0, a_int - 1, 0, a_int - 1> visited;

  for (std::int32_t yaw = 0; yaw < 65536; ++yaw)
  {
    Velocity neighbour = vf;
    Ground::accelerate(neighbour, Angle(yaw));
    neighbour.snap();

    if (auto& entry = visited.value(static_cast<int>(neighbour[0]) - vx_min, static_cast<int>(neighbour[1]) - vy_min);
        !entry)
    {
      entry = true;
      add_neighbour(neighbours, neighbour, yaw);
    }
  }
  return neighbours;
}
