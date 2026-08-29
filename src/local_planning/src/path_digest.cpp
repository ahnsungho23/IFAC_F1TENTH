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

#include "local_planning/path_digest.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <sstream>

namespace local_planning
{
namespace
{

void hashBytes(std::uint64_t & hash, const void * data, std::size_t size)
{
  const auto * bytes = static_cast<const unsigned char *>(data);
  for (std::size_t index = 0U; index < size; ++index) {
    hash ^= static_cast<std::uint64_t>(bytes[index]);
    hash *= 1099511628211ULL;
  }
}

void hashDouble(std::uint64_t & hash, double value)
{
  std::uint64_t bits = 0U;
  static_assert(sizeof(bits) == sizeof(value), "unexpected double width");
  std::memcpy(&bits, &value, sizeof(bits));
  hashBytes(hash, &bits, sizeof(bits));
}

}  // namespace

std::string pathDigest(const f110_msgs::msg::WpntArray & path)
{
  std::uint64_t hash = 1469598103934665603ULL;
  const std::uint64_t count = static_cast<std::uint64_t>(path.wpnts.size());
  hashBytes(hash, &count, sizeof(count));
  for (const auto & waypoint : path.wpnts) {
    hashDouble(hash, waypoint.s_m);
    hashDouble(hash, waypoint.d_m);
    hashDouble(hash, waypoint.x_m);
    hashDouble(hash, waypoint.y_m);
    hashDouble(hash, waypoint.psi_rad);
    hashDouble(hash, waypoint.kappa_radpm);
    hashDouble(hash, waypoint.vx_mps);
    hashDouble(hash, waypoint.ax_mps2);
  }
  std::ostringstream output;
  output << std::hex << std::setw(16) << std::setfill('0') << hash;
  return output.str();
}

}  // namespace local_planning
