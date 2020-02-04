#pragma once

#include <array>
#include <cassert>
#include <cstdint>
#include <deque>

template <typename T>
class Deque
{
public:
  T& operator[](std::int32_t i)
  {
    if (auto const size = m_deque.size(); size == 0)
    {
      m_offset = i;
      return m_deque.emplace_back();
    }
    else if (auto const diff = i - m_offset; diff < 0)
    {
      m_offset = i;
      return *m_deque.insert(m_deque.cbegin(), -diff, T{});
    }
    else if (static_cast<std::size_t>(diff) >= size)
    {
      m_deque.resize(diff + 1);
      return m_deque.back();
    }
    else
    {
      return m_deque[diff];
    }
  }

private:
  std::deque<T> m_deque;
  std::int32_t  m_offset;
};

template <typename T>
class Matrix
{
public:
  T& value(std::int32_t x, std::int32_t y)
  {
    return m_matrix[y][x];
  }

private:
  Deque<Deque<T>> m_matrix;
};

template <typename T, std::int32_t x_min, std::int32_t x_max, std::int32_t y_min, std::int32_t y_max>
class StaticMatrix
{
public:
  T& value(std::int32_t x, std::int32_t y)
  {
    assert(x >= x_min && x <= x_max);
    assert(y >= y_min && y <= y_max);
    return m_matrix[y - y_min][x - x_min];
  }

private:
  static_assert(x_max >= x_min);
  static_assert(y_max >= y_min);
  static constexpr std::size_t m_rows = y_max - y_min + 1;
  static constexpr std::size_t m_cols = x_max - x_min + 1;

  std::array<std::array<T, m_cols>, m_rows> m_matrix{};
};
