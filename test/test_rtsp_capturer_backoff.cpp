// Copyright (c) 2024 Óscar Pons Fernández
// Copyright (c) 2024 Grupo Avispa, DTE, Universidad de Málaga
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gtest/gtest.h"
#include "ip_camera_ros2/rtsp_capturer.hpp"

// MAX_RETRY_DELAY_S is a private implementation detail (currently 32s); mirrored here
// so the test documents the contract without depending on the private constant.
namespace
{
constexpr int kExpectedMaxRetryDelayS = 32;
}

TEST(NextRetryDelay, DoublesUntilItHitsTheCap) {
  int delay = 1;
  EXPECT_EQ(delay = RTSPCapturer::next_retry_delay(delay), 2);
  EXPECT_EQ(delay = RTSPCapturer::next_retry_delay(delay), 4);
  EXPECT_EQ(delay = RTSPCapturer::next_retry_delay(delay), 8);
  EXPECT_EQ(delay = RTSPCapturer::next_retry_delay(delay), 16);
  EXPECT_EQ(delay = RTSPCapturer::next_retry_delay(delay), kExpectedMaxRetryDelayS);
}

TEST(NextRetryDelay, StaysCappedOnceMaxIsReached) {
  EXPECT_EQ(RTSPCapturer::next_retry_delay(kExpectedMaxRetryDelayS), kExpectedMaxRetryDelayS);
  EXPECT_EQ(
    RTSPCapturer::next_retry_delay(kExpectedMaxRetryDelayS * 4), kExpectedMaxRetryDelayS);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
