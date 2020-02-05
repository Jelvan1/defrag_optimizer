#include "Angle.h"
#include "Array.h"
#include "Constants.h"
#include "Matrix.h"
#include "Node.h"
#include "Simulate.h"
#include "UsrCmd.h"
#include "Velocity.h"

#include <algorithm>
#include <array>
#include <cassert>
#define _USE_MATH_DEFINES
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <utility>
#include <memory>

static int hScore(Velocity const& v)
{
#if 1
  return 0;
  int      i  = 0;
  Velocity vf = v;
  while (vf[0] < 412)
  {
    Ground::friction(vf);
    vf[0] += 320 * Ground::s_acceleration * frametime;
    vf.snap();
    ++i;
  }
  return i;
#else
  int          i     = 0;
  double const v_abs = v.VectorLength();
  return std::ceil((609 - v_abs) / 3.1623);
#endif
}

class nodeSet
{
public:
  void emplace(Velocity const& v, int gScore, int hScore)
  {
    nodes.emplace(v, gScore, hScore);
  }

  void swap(Velocity const& v, int old_gScore, int new_gScore)
  {
    // TODO: recalc hScore unnecessary
    Node old_node(v, old_gScore, hScore(v));
    auto itLower = nodes.lower_bound(old_node);
    for (;; ++itLower)
    {
      assert(itLower != nodes.cend());
      if (*itLower == old_node) break;
    }
    Node new_node(*itLower, new_gScore);
    nodes.erase(itLower);
    nodes.emplace(std::move(new_node));
  }

  Node pop()
  {
    assert(!nodes.empty());
    return nodes.extract(nodes.cbegin()).value();
  }

  bool empty() const
  {
    return nodes.empty();
  }

  std::size_t size() const
  {
    return nodes.size();
  }

private:
  struct Compare
  {
    bool operator()(Node const& node1, Node const& node2) const
    {
      int const fScore1 = node1.m_gScore + node1.m_hScore;
      int const fScore2 = node2.m_gScore + node2.m_hScore;
      if (fScore1 < fScore2) return true;
      if (fScore1 > fScore2) return false;

      if (node1.m_gScore > node2.m_gScore) return true;
      if (node1.m_gScore < node2.m_gScore) return false;

      return node1.m_v[0] > node2.m_v[0];
    }
  };

public:
  std::multiset<Node, Compare> nodes;
};

static void reconstruct_path(std::ostream&                                         os,
                             std::map<Velocity, std::pair<Velocity, Angle>> const& cameFrom,
                             Velocity const&                                       current)
{
  auto const iter = cameFrom.find(current);
  if (iter == cameFrom.cend()) return;

  auto const& [parent_v, yaw] = iter->second;
  reconstruct_path(os, cameFrom, parent_v);
  os << cmd << " pyr 0 " << yaw << " 0; wait 2; // " << current << '\n';
}

static Velocity realVelocity(std::map<Velocity, std::pair<Velocity, Angle>> const& cameFrom, Velocity const& v)
{
  auto const iter = cameFrom.find(v);
  if (iter == cameFrom.cend()) return v;

  auto const& [parent_v, yaw] = iter->second;
  Velocity vf                 = parent_v;
  Ground::friction(vf);
  Ground::accelerate(vf, yaw);
  return vf;
}

static std::array init_vels = {
  Velocity(0, 0),
};

int main()
{
  for (auto const& init_vel : init_vels)
  {
    std::cout << "Initial velocity: " << init_vel << '\n';

    nodeSet openSet;
    // std::set<Velocity> closedSet;
    Matrix<bool> closedSet;

    std::map<Velocity, std::pair<Velocity, Angle>> cameFrom;

    // std::map<Velocity, int> gScore;
    auto gScoresPtr = std::make_unique<StaticMatrix<int, -500, 500, -500, 500>>();

    openSet.emplace(init_vel, 0, hScore(init_vel));
    gScoresPtr->value(init_vel[0], init_vel[1]) = 0;

    float best_v = 0;
    while (!openSet.empty())
    {
      Node const current                          = openSet.pop();
      closedSet.value(current.m_v[0], current.m_v[1]) = true;
      // auto const [it, inserted] = closedSet.emplace(current.m_v);
      // assert(inserted);

      // file << current.m_v[0] << ',' << current.m_v[1] << ',' << best_v << ',' <<
      //   current.m_gScore << ',' << current.m_hScore << ',' << current.m_gScore + current.m_hScore << '\n';

#ifndef WALK
      if (current.m_gScore + current.m_hScore > 98)
      {
        std::cout << "ABORT\n";
        break;
      }
#endif

      if (current.m_v.VectorLength() > best_v)
      {
        best_v = current.m_v.VectorLength();

        if (best_v >= 413)
        {
          return 0;
          // auto const real_v = realVelocity(cameFrom, current.m_v);
          // std::string const file = static_cast<std::ostringstream&>(std::ostringstream() << real_v).str() + ".cfg";
          std::string const file =
            static_cast<std::ostringstream&>(std::ostringstream() << "_" << best_v).str() + ".cfg";
          std::ofstream os(file);
          os << "// gScore: " << current.m_gScore << '\n';
          reconstruct_path(os, cameFrom, current.m_v);
        }
      }

      std::cout << openSet.size() << ": " << best_v << " => " << current.m_v << ' ' << current.m_gScore << '\n';

      if (current.m_v.VectorLength() >= 607)
      {
        std::cout << "FOUND" << std::endl;
        // auto const real_v = realVelocity(cameFrom, current.m_v);
        // std::string const file = static_cast<std::ostringstream&>(std::ostringstream() << real_v).str() + ".cfg";
        std::string const file =
          static_cast<std::ostringstream&>(std::ostringstream()
                                           << init_vel << "_" << current.m_v.VectorLength() << "_" << current.m_gScore)
            .str() +
          ".cfg";
        std::ofstream os(file);
        os << "// gScore: " << current.m_gScore << '\n';
        reconstruct_path(os, cameFrom, current.m_v);
      }

#ifndef WALK
      // Don't generate neighbours.
      if (current.m_gScore == 98) continue;
#endif

      // int const tentative_gScore = current.m_gScore + 1;

      auto const neighbours = current.neighbours();
      for (auto const& [neighbour, yaw] : neighbours)
      {
        if (closedSet.value(neighbour[0], neighbour[1])) continue;
        // if (closedSet.count(neighbour)) continue;

        // int const tentative_gScore = -neighbour.VectorLength();
        int const tentative_gScore = -neighbour[0];
        if (auto& iter = gScoresPtr->value(neighbour[0], neighbour[1]); iter == 0) // TODO: correct?
        // if (auto const iter = gScoresPtr->find(neighbour); iter == gScoresPtr->cend())
        {
          openSet.emplace(neighbour, tentative_gScore, hScore(neighbour));
          cameFrom[neighbour] = std::make_pair(current.m_v, yaw);
          iter                = tentative_gScore;
        }
        else if (tentative_gScore < iter)
        {
          openSet.swap(neighbour, iter, tentative_gScore);
          cameFrom[neighbour] = std::make_pair(current.m_v, yaw);
          iter                = tentative_gScore;
        }
      }
    }
  }
  std::cout << "UNSATISFIABLE" << std::endl;
}
