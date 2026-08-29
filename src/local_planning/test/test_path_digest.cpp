// Copyright 2026 2026_IFAC contributors
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

#include <gtest/gtest.h>

#include <algorithm>

#include "local_planning/path_digest.hpp"

namespace local_planning
{
namespace
{

f110_msgs::msg::Wpnt makeWaypoint(
  double s, double d, double x, double y, double psi, double kappa, double vx, double ax)
{
  f110_msgs::msg::Wpnt waypoint;
  waypoint.s_m = s;
  waypoint.d_m = d;
  waypoint.x_m = x;
  waypoint.y_m = y;
  waypoint.psi_rad = psi;
  waypoint.kappa_radpm = kappa;
  waypoint.vx_mps = vx;
  waypoint.ax_mps2 = ax;
  return waypoint;
}

f110_msgs::msg::WpntArray knownPath()
{
  f110_msgs::msg::WpntArray path;
  path.wpnts.push_back(makeWaypoint(1.0, -2.0, 3.5, -4.25, 0.5, -0.125, 6.0, -1.5));
  path.wpnts.push_back(makeWaypoint(7.25, 0.75, -8.5, 9.0, -0.25, 0.0625, 4.5, 0.25));
  return path;
}

TEST(PathDigest, IdenticalWaypointSequenceHasIdenticalDigest)
{
  const auto first = knownPath();
  const auto second = first;
  EXPECT_EQ(pathDigest(first), pathDigest(second));
}

TEST(PathDigest, WaypointFieldChangeChangesDigest)
{
  const auto original = knownPath();
  auto changed = original;
  changed.wpnts[0].x_m += 1.0;
  EXPECT_NE(pathDigest(original), pathDigest(changed));
}

TEST(PathDigest, WaypointOrderChangeChangesDigest)
{
  const auto original = knownPath();
  auto reordered = original;
  std::swap(reordered.wpnts[0], reordered.wpnts[1]);
  EXPECT_NE(pathDigest(original), pathDigest(reordered));
}

TEST(PathDigest, WaypointCountChangeChangesDigest)
{
  const auto original = knownPath();
  auto shortened = original;
  shortened.wpnts.pop_back();
  EXPECT_NE(pathDigest(original), pathDigest(shortened));
}

TEST(PathDigest, MatchesLegacyDigestForKnownInput)
{
  EXPECT_EQ(pathDigest(knownPath()), "3fae34c80325cf10");
}

}  // namespace
}  // namespace local_planning
