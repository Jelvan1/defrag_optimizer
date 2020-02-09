#pragma once

#include <cassert>
#include <cstdint>
#include <iostream>

class Angle
{
public:
  constexpr Angle() = default;

  constexpr explicit Angle(std::int32_t angle) : m_angle(angle & 65535)
  {
  }

  constexpr std::uint16_t value() const
  {
    return m_angle;
  }

  Angle& operator++()
  {
    ++m_angle;
    return *this;
  }

  friend Angle operator+(Angle left, Angle right)
  {
    return Angle(left.m_angle + right.m_angle);
  }

  friend Angle operator-(Angle left, Angle right)
  {
    assert(left > right);
    return Angle(left.m_angle - right.m_angle);
  }

  friend Angle operator/(Angle angle, std::uint16_t divisor)
  {
    assert(angle.m_angle % divisor);
    return Angle(angle.m_angle / divisor);
  }

  friend std::uint16_t operator%(Angle angle, std::uint16_t divisor)
  {
    return angle.m_angle % divisor;
  }

  Angle& operator+=(Angle other)
  {
    m_angle = (m_angle + other.m_angle) & 65535;
    return *this;
  }

  bool operator==(Angle other) const
  {
    return m_angle == other.m_angle;
  }

  bool operator<(Angle other) const
  {
    return m_angle < other.m_angle;
  }

  bool operator<=(Angle other) const
  {
    return m_angle <= other.m_angle;
  }

  bool operator>(Angle other) const
  {
    return m_angle > other.m_angle;
  }

  bool operator>=(Angle other) const
  {
    return m_angle >= other.m_angle;
  }

  friend std::ostream& operator<<(std::ostream& os, Angle angle)
  {
    return os << angle.m_angle;
  }

private:
  std::uint16_t m_angle = 0;
};
