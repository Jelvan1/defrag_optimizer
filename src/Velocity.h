#pragma once

#include "Array.h"

class Velocity : public Array<float, 2>
{
public:
  using Array::Array;

  void snap()
  {
    // Sys_SnapVector
#if 0
    m_array[0] = std::rint(m_array[0]);
    m_array[1] = std::rint(m_array[1]);
#else
#  define QROUNDX87(src)                                                                                               \
    "flds " src                                                                                                        \
    "\n"                                                                                                               \
    "fistpl " src                                                                                                      \
    "\n"                                                                                                               \
    "fildl " src                                                                                                       \
    "\n"                                                                                                               \
    "fstps " src "\n"
    static const unsigned short cw037F = 0x037F;
    unsigned short              cwCurr;

    __asm__ volatile(
      "fnstcw %1\n"
      "fldcw %2\n" QROUNDX87("0(%0)") QROUNDX87("4(%0)") "fldcw %1\n"
      :
      : "r"(m_array.data()), "m"(cwCurr), "m"(cw037F)
      : "memory", "st");
#  undef QROUNDX87
#endif
  }

  bool operator<(Velocity const& other) const
  {
    if (m_array[0] == other.m_array[0]) return m_array[1] < other.m_array[1];
    return m_array[0] < other.m_array[0];
  }
};
