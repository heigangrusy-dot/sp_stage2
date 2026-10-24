#include "control_safety.hpp"

namespace control
{
bool ControlSafety::update(
  bool remote_valid, bool switch_down, bool motion_requested, bool system_ready)
{
  if (!remote_valid || !system_ready) {
    this->reset();
    return false;
  }

  if (switch_down) {
    this->down_observed_ = true;
    return false;
  }

  return this->down_observed_ && motion_requested;
}

void ControlSafety::reset() { this->down_observed_ = false; }
}  // namespace control
