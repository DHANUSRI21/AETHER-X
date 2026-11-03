// Copyright 2026 Dhanusri Veerappan
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef AETHER_CORE__SYSTEM_STATE_MANAGER_HPP_
#define AETHER_CORE__SYSTEM_STATE_MANAGER_HPP_

#include "aether_core/system_state.hpp"

namespace aether
{

class SystemStateManager
{
public:
  SystemStateManager()
  : current_state_(SystemState::INITIALIZING)
  {
  }

  SystemState get_state() const
  {
    return current_state_;
  }

  bool transition_to(SystemState new_state)
  {
    if (!is_valid_transition(current_state_, new_state)) {
      return false;
    }

    current_state_ = new_state;
    return true;
  }

private:
  bool is_valid_transition(
    SystemState current,
    SystemState next) const
  {
    switch (current) {
      case SystemState::INITIALIZING:
        return next == SystemState::OPERATIONAL ||
               next == SystemState::COMMUNICATION_LOST;

      case SystemState::OPERATIONAL:
        return next == SystemState::DEGRADED ||
               next == SystemState::COMMUNICATION_LOST;

      case SystemState::DEGRADED:
        return next == SystemState::OPERATIONAL ||
               next == SystemState::COMMUNICATION_LOST;

      case SystemState::COMMUNICATION_LOST:
        return next == SystemState::RECOVERING;

      case SystemState::RECOVERING:
        return next == SystemState::OPERATIONAL ||
               next == SystemState::COMMUNICATION_LOST;
    }

    return false;
  }

  SystemState current_state_;
};

}  // namespace aether

#endif  // AETHER_CORE__SYSTEM_STATE_MANAGER_HPP_
