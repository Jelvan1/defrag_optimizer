#pragma once

#include <array>
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>

template <typename T, std::size_t N>
class Array
{
public:
  static_assert(N > 1);

  template <typename... U>
  constexpr Array(U... e)
    : m_array{ static_cast<T>(e)... }
  {
  }

  T const& operator[](std::size_t i) const
  {
    return m_array[i];
  }

  T& operator[](std::size_t i)
  {
    return m_array[i];
  }

  T sumOfSquares() const
  {
    static_assert(N == 2);
    return m_array[0] * m_array[0] + m_array[1] * m_array[1];
  }

  T VectorLength() const
  {
    return std::sqrt(sumOfSquares());
  }

  T VectorNormalize()
  {
    auto const length = VectorLength();

    if (length)
    {
      auto const ilength = 1 / length;
      static_assert(N == 2);
      *this *= ilength;
    }

    return length;
  }

  bool operator==(Array const& other) const
  {
    return m_array == other.m_array;
  }

  bool operator!=(Array const& other) const
  {
    return m_array != other.m_array;
  }

  Array& operator=(T const& value)
  {
    m_array.fill(value);
    return *this;
  }

  Array& operator*=(T const& value)
  {
    static_assert(N == 2);
    m_array[0] *= value;
    m_array[1] *= value;
    return *this;
  }

  friend std::ostream& operator<<(std::ostream& os, Array const& array)
  {
    os << '(';
    for (std::size_t i = 0; i < N - 1; ++i)
    {
      os << array[i] << ',';
    }
    return os << array.m_array.back() << ')';
  }

protected:
  std::array<T, N> m_array;
};
