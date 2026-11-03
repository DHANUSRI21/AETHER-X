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
#ifndef AETHER_CORE__SYSTEM_STATE_HPP_
#define AETHER_CORE__SYSTEM_STATE_HPP_

#include <string>

namespace aether
{

enum class SystemState
{
  INITIALIZING,
  OPERATIONAL,
  DEGRADED,
  COMMUNICATION_LOST,
  RECOVERING
};

inline std::string to_string(SystemState state)
{
  switch (state) {
    case SystemState::INITIALIZING:
      return "INITIALIZING";

    case SystemState::OPERATIONAL:
      return "OPERATIONAL";

    case SystemState::DEGRADED:
      return "DEGRADED";

    case SystemState::COMMUNICATION_LOST:
      return "COMMUNICATION_LOST";

    case SystemState::RECOVERING:
      return "RECOVERING";
  }

  return "UNKNOWN";
}

}  // namespace aether

#endif  // AETHER_CORE__SYSTEM_STATE_HPP_
