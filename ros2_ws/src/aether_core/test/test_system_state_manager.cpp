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
#include <gtest/gtest.h>

#include "aether_core/system_state_manager.hpp"

using aether::SystemState;
using aether::SystemStateManager;

TEST(SystemStateManagerTest, InitialStateIsInitializing)
{
    SystemStateManager manager;

    EXPECT_EQ(manager.get_state(), SystemState::INITIALIZING);
}

TEST(SystemStateManagerTest, AllowsValidOperationalTransition)
{
    SystemStateManager manager;

    EXPECT_TRUE(manager.transition_to(SystemState::OPERATIONAL));
    EXPECT_EQ(manager.get_state(), SystemState::OPERATIONAL);
}

TEST(SystemStateManagerTest, RejectsInvalidTransition)
{
    SystemStateManager manager;

    EXPECT_FALSE(manager.transition_to(SystemState::RECOVERING));
    EXPECT_EQ(manager.get_state(), SystemState::INITIALIZING);
}

TEST(SystemStateManagerTest, DetectsCommunicationLoss)
{
    SystemStateManager manager;

    ASSERT_TRUE(manager.transition_to(SystemState::OPERATIONAL));

    EXPECT_TRUE(manager.transition_to(SystemState::COMMUNICATION_LOST));
    EXPECT_EQ(manager.get_state(), SystemState::COMMUNICATION_LOST);
}

TEST(SystemStateManagerTest, SupportsRecovery)
{
    SystemStateManager manager;

    ASSERT_TRUE(manager.transition_to(SystemState::OPERATIONAL));
    ASSERT_TRUE(manager.transition_to(SystemState::COMMUNICATION_LOST));

    EXPECT_TRUE(manager.transition_to(SystemState::RECOVERING));
    EXPECT_EQ(manager.get_state(), SystemState::RECOVERING);

    EXPECT_TRUE(manager.transition_to(SystemState::OPERATIONAL));
    EXPECT_EQ(manager.get_state(), SystemState::OPERATIONAL);
}

TEST(SystemStateManagerTest, RejectsInvalidTransitionFromCommunicationLost)
{
    SystemStateManager manager;

    ASSERT_TRUE(manager.transition_to(SystemState::COMMUNICATION_LOST));

    EXPECT_FALSE(manager.transition_to(SystemState::OPERATIONAL));
    EXPECT_EQ(manager.get_state(), SystemState::COMMUNICATION_LOST);
}

TEST(SystemStateManagerTest, SupportsDegradedState)
{
    SystemStateManager manager;

    ASSERT_TRUE(manager.transition_to(SystemState::OPERATIONAL));

    EXPECT_TRUE(manager.transition_to(SystemState::DEGRADED));
    EXPECT_EQ(manager.get_state(), SystemState::DEGRADED);

    EXPECT_TRUE(manager.transition_to(SystemState::OPERATIONAL));
    EXPECT_EQ(manager.get_state(), SystemState::OPERATIONAL);
}
