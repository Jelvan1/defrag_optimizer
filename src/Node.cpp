#include "Node.h"

#include "Constants.h"
#include "Matrix.h"
#include "Simulate.h"

#include <cassert>
#define _USE_MATH_DEFINES
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
void visit(Velocity const&                          vf,
           int                                      vx_min,
           int                                      vy_min,
           StaticMatrix<bool, args...>&             visited,
           std::vector<std::pair<Velocity, Angle>>& neighbours,
           Velocity const&                          min,
           Velocity const&                          max,
           std::int32_t                             yaw_min,
           std::int32_t                             yaw_max)
{
  auto const yaw_range = yaw_max - yaw_min;
  assert(yaw_range % 2 == 0);
  auto const yaw = yaw_range / 2 + yaw_min;

  Velocity neighbour = vf;
  Ground::accelerate(neighbour, Angle(yaw));
  neighbour.snap();

  if (yaw_range == 2)
  {
    auto& entry = visited.value(static_cast<int>(neighbour[0]) - vx_min, static_cast<int>(neighbour[1]) - vy_min);
    if (!entry)
    {
      entry = true;
      add_neighbour(neighbours, neighbour, yaw);
    }
    // Visited all angles.
    return;
  }

  if (neighbour == min)
  {
    visit(vf, vx_min, vy_min, visited, neighbours, min, max, yaw, yaw_max);
  }
  else if (neighbour == max)
  {
    visit(vf, vx_min, vy_min, visited, neighbours, min, max, yaw_min, yaw);
  }
  else
  {
    auto& entry = visited.value(static_cast<int>(neighbour[0]) - vx_min, static_cast<int>(neighbour[1]) - vy_min);
    if (!entry)
    {
      entry = true;
      add_neighbour(neighbours, neighbour, yaw);
    }
    visit(vf, vx_min, vy_min, visited, neighbours, min, neighbour, yaw_min, yaw);
    visit(vf, vx_min, vy_min, visited, neighbours, neighbour, max, yaw, yaw_max);
  }
}
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

  Velocity neighbour = vf;
  Ground::accelerate(neighbour, Angle(0));
  neighbour.snap();

  auto& entry = visited.value(static_cast<int>(neighbour[0]) - vx_min, static_cast<int>(neighbour[1]) - vy_min);
  entry = true;
  add_neighbour(neighbours, neighbour, 0);

  visit(vf, vx_min, vy_min, visited, neighbours, neighbour, neighbour, 0, 65536);

  return neighbours;
}

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

  for (std::int32_t yaw = 0; yaw < 65536; ++yaw)
  {
    Velocity neighbour = vf;
    Ground::accelerate(neighbour, Angle(yaw));
    neighbour.snap();

    auto& entry = visited.value(static_cast<int>(neighbour[0]) - vx_min, static_cast<int>(neighbour[1]) - vy_min);
    if (!entry)
    {
      entry = true;
      add_neighbour(neighbours, neighbour, yaw);
    }
  }
  return neighbours;
}
