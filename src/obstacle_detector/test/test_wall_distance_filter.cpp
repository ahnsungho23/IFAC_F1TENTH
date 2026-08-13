#include <gtest/gtest.h>

#include <cmath>

#include "obstacle_detector/wall_distance_filter.hpp"

namespace obstacle_detector
{
namespace
{

// 40x40 grid @ 0.1 m, origin (0,0) -> 4 m x 4 m world.
//   - wall: horizontal row gy=10, gx=5..34  (3.0 m long straight line -> WALL)
//   - blob: 3x3 block gx=18..20, gy=25..27  (compact, non-linear -> NOT wall)
//   - stub: 2 cells gx=35..36, gy=30        (linear but shorter than 1.0 m -> NOT wall)
nav_msgs::msg::OccupancyGrid makeMap()
{
    nav_msgs::msg::OccupancyGrid map;
    map.info.resolution = 0.1F;
    map.info.width = 40;
    map.info.height = 40;
    map.info.origin.position.x = 0.0;
    map.info.origin.position.y = 0.0;
    map.data.assign(40U * 40U, 0);
    auto set = [&map](int gx, int gy) {
        map.data[static_cast<std::size_t>(gy) * 40U + static_cast<std::size_t>(gx)] = 100;
    };
    for (int gx = 5; gx <= 34; ++gx)
    {
        set(gx, 10);
    }
    for (int gx = 18; gx <= 20; ++gx)
    {
        for (int gy = 25; gy <= 27; ++gy)
        {
            set(gx, gy);
        }
    }
    set(35, 30);
    set(36, 30);
    return map;
}

WallDistanceFilter::Params testParams()
{
    WallDistanceFilter::Params params;
    params.occupied_thresh = 50;
    params.linear_ratio = 4.0;
    params.min_length_m = 1.0;
    return params;
}

TEST(WallDistanceFilter, KeepsLongLinearComponentAsWall)
{
    WallDistanceFilter filter;
    filter.buildFromMap(makeMap(), testParams());
    ASSERT_TRUE(filter.active());

    // A point on the wall row itself is wall structure.
    EXPECT_DOUBLE_EQ(filter.distanceToWall(1.05, 1.05), 0.0);
    EXPECT_TRUE(filter.isWallPoint(1.05, 1.05, 0.2));
    // The full 3.0 m span is covered, including both ends.
    EXPECT_TRUE(filter.isWallPoint(0.55, 1.05, 0.2));   // gx=5 end
    EXPECT_TRUE(filter.isWallPoint(3.45, 1.05, 0.2));   // gx=34 end
}

TEST(WallDistanceFilter, RejectsCompactBlobAndShortStub)
{
    WallDistanceFilter filter;
    filter.buildFromMap(makeMap(), testParams());
    ASSERT_TRUE(filter.active());

    // The 3x3 blob (eigenvalue ratio ~1) is not a wall: nearby points stay obstacle candidates.
    EXPECT_FALSE(filter.isWallPoint(1.95, 2.65, 0.2));  // blob centre
    EXPECT_FALSE(filter.isWallPoint(1.95, 2.45, 0.2));  // directly against the blob
    // The 2-cell stub is linear but far below wall_min_length_m.
    EXPECT_FALSE(filter.isWallPoint(3.55, 3.05, 0.2));
    EXPECT_FALSE(filter.isWallPoint(3.65, 3.05, 0.2));
}

TEST(WallDistanceFilter, DistanceTransformValuesAreGridExact)
{
    WallDistanceFilter filter;
    filter.buildFromMap(makeMap(), testParams());
    ASSERT_TRUE(filter.active());

    // Orthogonally adjacent cell: one resolution step.
    EXPECT_NEAR(filter.distanceToWall(1.05, 1.15), 0.1, 1.0e-6);
    // Diagonally adjacent to the wall end cell (5,10): sqrt(2) * resolution.
    EXPECT_NEAR(filter.distanceToWall(0.45, 1.15), 0.1 * std::sqrt(2.0), 1.0e-6);
    // Two cells away orthogonally.
    EXPECT_NEAR(filter.distanceToWall(1.05, 1.25), 0.2, 1.0e-6);
}

TEST(WallDistanceFilter, ClassifiesPointsWithinAssociationDistance)
{
    WallDistanceFilter filter;
    filter.buildFromMap(makeMap(), testParams());
    ASSERT_TRUE(filter.active());

    // Within 0.2 m of the wall -> Layer 1 structure, dropped.
    EXPECT_TRUE(filter.isWallPoint(1.05, 1.14, 0.2));   // cell dist 0.1
    EXPECT_TRUE(filter.isWallPoint(0.45, 1.15, 0.2));   // diagonal 0.141
    // At/beyond 0.2 m -> survives as an obstacle candidate.
    EXPECT_FALSE(filter.isWallPoint(1.05, 1.25, 0.2));  // cell dist exactly 0.2
    EXPECT_FALSE(filter.isWallPoint(1.05, 1.45, 0.2));  // 0.4 m away
    // A larger association distance reclaims the same points.
    EXPECT_TRUE(filter.isWallPoint(1.05, 1.25, 0.3));
}

TEST(WallDistanceFilter, InactiveOrOutOfBoundsIsNeverWall)
{
    WallDistanceFilter empty_filter;
    EXPECT_FALSE(empty_filter.active());
    EXPECT_FALSE(empty_filter.isWallPoint(1.0, 1.0, 0.2));
    EXPECT_DOUBLE_EQ(empty_filter.distanceToWall(1.0, 1.0), -1.0);

    WallDistanceFilter filter;
    filter.buildFromMap(makeMap(), testParams());
    ASSERT_TRUE(filter.active());
    EXPECT_FALSE(filter.isWallPoint(-0.5, 1.05, 0.2));  // outside the grid
    EXPECT_DOUBLE_EQ(filter.distanceToWall(-0.5, 1.05), -1.0);

    nav_msgs::msg::OccupancyGrid invalid;  // empty map -> filter stays inactive
    WallDistanceFilter invalid_filter;
    invalid_filter.buildFromMap(invalid, testParams());
    EXPECT_FALSE(invalid_filter.active());
}

}  // namespace
}  // namespace obstacle_detector
