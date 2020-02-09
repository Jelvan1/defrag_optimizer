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
class Bisection
{
public:
  Bisection(
    Velocity const&                          vf,
    std::int32_t                             vx_min,
    std::int32_t                             vy_min,
    StaticMatrix<bool, args...>&             visited,
    std::vector<std::pair<Velocity, Angle>>& neighbours)
    : m_vf(vf), m_vx_min(vx_min), m_vy_min(vy_min), m_visited(visited), m_neighbours(neighbours)
  {
  }

  void operator()(Velocity const& min, Velocity const& max, std::int32_t yaw_min, std::int32_t yaw_max)
  {
    assert(yaw_min < yaw_max);
    assert((yaw_max - yaw_min) % 2 == 0);
    auto const yaw_range = yaw_max - yaw_min;
    auto const yaw       = yaw_range / 2 + yaw_min;

    Velocity neighbour = m_vf;
    Ground::accelerate(neighbour, Angle(yaw));
    neighbour.snap();

    if (yaw_range == 2)
    {
      if (auto& entry = m_visited.value(
            static_cast<std::int32_t>(neighbour[0]) - m_vx_min, static_cast<std::int32_t>(neighbour[1]) - m_vy_min);
          !entry)
      {
        entry = true;
        add_neighbour(m_neighbours, neighbour, yaw);
      }
      // Visited all angles.
      return;
    }

    if (neighbour == min)
    {
      operator()(min, max, yaw, yaw_max);
    }
    else if (neighbour == max)
    {
      operator()(min, max, yaw_min, yaw);
    }
    else
    {
      if (auto& entry = m_visited.value(
            static_cast<std::int32_t>(neighbour[0]) - m_vx_min, static_cast<std::int32_t>(neighbour[1]) - m_vy_min);
          !entry)
      {
        entry = true;
        add_neighbour(m_neighbours, neighbour, yaw);
      }
      operator()(min, neighbour, yaw_min, yaw);
      operator()(neighbour, max, yaw, yaw_max);
    }
  }

private:
  Velocity const&                          m_vf;
  std::int32_t const                       m_vx_min;
  std::int32_t const                       m_vy_min;
  StaticMatrix<bool, args...>&             m_visited;
  std::vector<std::pair<Velocity, Angle>>& m_neighbours;
};
} // namespace

std::vector<std::pair<Velocity, Angle>> Node::neighbours() const
{
  std::vector<std::pair<Velocity, Angle>> neighbours;

  Velocity vf = m_v;
  Ground::friction(vf);

  constexpr float    a      = 320 * Ground::s_acceleration * frametime;
  std::int32_t const vx_min = std::rint(vf[0] - a);
  std::int32_t const vy_min = std::rint(vf[1] - a);

  constexpr std::int32_t a_int = 2 + 2 * std::round(a);
  static_assert(a_int <= 100);
  StaticMatrix<bool, 0, a_int - 1, 0, a_int - 1> visited;

  constexpr float d_opt_num = 320 - a;
  float const     d_opt_den = vf.VectorLength();
  static_assert(d_opt_num > 0);
  float const d_opt = d_opt_num >= d_opt_den ? 0 : std::acos(d_opt_num / d_opt_den);
  float const d_vel = std::atan2(vf[1], vf[0]);

  // Use the bisection method to find (almost) all neighbours.
  // Start searching from the velocity direction such that we don't miss that one either.
  std::int32_t const init      = std::round(d_vel * 32768 / M_PI);
  Velocity           neighbour = vf;
  Ground::accelerate(neighbour, Angle(init));
  neighbour.snap();

  {
    auto& entry =
      visited.value(static_cast<std::int32_t>(neighbour[0]) - vx_min, static_cast<std::int32_t>(neighbour[1]) - vy_min);
    entry = true;
    add_neighbour(neighbours, neighbour, init);
  }
  Bisection bisection(vf, vx_min, vy_min, visited, neighbours);
  bisection(neighbour, neighbour, init, init + 65536);

  // The bisection method can miss the neighbours around the optimal accel angles.
  // Positive optimal accel angle.
  std::int32_t const pos_opt = std::ceil((d_vel + d_opt) * 32768 / M_PI);
  neighbour                  = vf;
  Ground::accelerate(neighbour, Angle(pos_opt));
  neighbour.snap();

  if (auto& entry = visited.value(
        static_cast<std::int32_t>(neighbour[0]) - vx_min, static_cast<std::int32_t>(neighbour[1]) - vy_min);
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

  if (auto& entry = visited.value(
        static_cast<std::int32_t>(neighbour[0]) - vx_min, static_cast<std::int32_t>(neighbour[1]) - vy_min);
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

  constexpr float    a      = 320 * Ground::s_acceleration * frametime;
  std::int32_t const vx_min = std::rint(vf[0] - a);
  std::int32_t const vy_min = std::rint(vf[1] - a);

  constexpr std::int32_t a_int = 2 + 2 * std::round(a);
  static_assert(a_int <= 100);
  StaticMatrix<bool, 0, a_int - 1, 0, a_int - 1> visited;

  for (std::int32_t yaw = 0; yaw < 65536; ++yaw)
  {
    Velocity neighbour = vf;
    Ground::accelerate(neighbour, Angle(yaw));
    neighbour.snap();

    if (auto& entry = visited.value(
          static_cast<std::int32_t>(neighbour[0]) - vx_min, static_cast<std::int32_t>(neighbour[1]) - vy_min);
        !entry)
    {
      entry = true;
      add_neighbour(neighbours, neighbour, yaw);
    }
  }
  return neighbours;
}
