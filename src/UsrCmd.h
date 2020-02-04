#pragma once

struct usercmd_t
{
  constexpr float PM_CmdScale() const
  {
    int max = std::abs(forwardmove);
    if (std::abs(rightmove) > max) max = std::abs(rightmove);
    if (std::abs(upmove) > max) max = std::abs(upmove);
    if (!max) return 0;

    float const total = std::sqrt(forwardmove * forwardmove + rightmove * rightmove + upmove * upmove);
    return 320.f * max / (127.0 * total);
  }

  friend std::ostream& operator<<(std::ostream& os, usercmd_t const& ucmd)
  {
    return os << "fm " << ucmd.forwardmove << "; rm " << ucmd.rightmove << "; um " << ucmd.upmove << ";";
  }

  std::int8_t forwardmove;
  std::int8_t rightmove;
  std::int8_t upmove;
};

#define FM_ONLY
constexpr usercmd_t const cmd{ 127, 0, 0 };
