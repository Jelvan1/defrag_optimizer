#pragma once

#include "Angle.h"
#include "Velocity.h"

#include <utility>
#include <vector>

struct Node
{
  Node(Velocity const& v, int gScore, int hScore) : m_v(v), m_gScore(gScore), m_hScore(hScore)
  {
  }

  Node(Node const& node, int gScore) : m_v(node.m_v), m_gScore(gScore), m_hScore(node.m_hScore)
  {
    assert(m_gScore < node.m_gScore); // Check that we are creating a better node.
  }

  std::vector<std::pair<Velocity, Angle>> neighbours() const;
  std::vector<std::pair<Velocity, Angle>> neighbours2() const;

  bool operator==(Node const& node) const
  {
    return m_v == node.m_v && m_gScore == node.m_gScore;
  }

  Velocity m_v;
  int      m_gScore = 0;
  int      m_hScore = 0;
};
