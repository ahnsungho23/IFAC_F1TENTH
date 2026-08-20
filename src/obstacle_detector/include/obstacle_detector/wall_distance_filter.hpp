// ================================================================================================
// WALL DISTANCE FILTER - structural Layer-1 filter (replaces the occupancy-ratio cluster filter)
// ================================================================================================
// The SLAM map and the current scan can diverge (post-collision environment changes, localization
// offset), so a per-cell occupancy test both erases real obstacles standing on mapped cells and
// keeps wall returns that fall slightly off the mapped wall. This filter instead extracts the
// LINEAR wall structures from the occupancy grid once per map:
//   1. Occupied cells (>= occupied_thresh) are grouped into connected components with
//      8-connectivity flood fill — the grid equivalent of DBSCAN (dense point clusters reachable
//      through eps-neighbourhoods, with the grid itself defining the neighbourhood).
//   2. Each component is a WALL when its extent along the major axis >= min_length_m. Compact
//      blobs (obstacles/debris baked into the map) fail the test, so points near them stay
//      obstacle candidates.
//
//      🔴 2026-08-16: 종전에는 여기에 PCA 선형성(lambda_max/lambda_min >= linear_ratio)도
//      요구했다. 그런데 트랙 경계는 **닫힌 고리**라 등방적이고, 선형성 검사를 통과할 수
//      없다. 실측(ifac 시뮬 맵): 점유 셀 100,006개가 성분 2개로 묶이는데 87,080개(87%)를
//      가진 바깥 경계 성분의 ratio가 1.85로 탈락했다. 그 결과 트랙 벽의 87%가 필터에
//      존재하지 않았고, 헤어핀에서 벽이 매 랩 장애물 3개로 잡혔다(유령 하나는 실물
//      장애물만큼 오래 지속). 유령 위치에서 "벽으로 인정된 셀"까지 거리가 2.4~2.7 m로,
//      wall_assoc_distance_m=0.2 필터가 닿을 수조차 없었다.
//
//      선형성은 성분 전체에 대해 물을 수 있는 성질이 아니다(국소 성질이다). 반면 크기는
//      성분 전체의 성질이고, 이 게이트의 원래 의도인 "맵에 구워진 작은 장애물은 지우지
//      말 것"을 그대로 지킨다 — 26 m짜리 경계는 통과하고 0.3 m짜리 상자는 통과하지 못한다.
//   3. A multi-source distance transform (8-neighbour Dijkstra over the grid) gives every cell
//      its distance [m] to the nearest wall cell, making each per-beam query O(1).
// A LiDAR point within the association distance of a wall structure IS Layer 1 (structure) and is
// dropped before clustering.
// ================================================================================================

#ifndef OBSTACLE_DETECTOR__WALL_DISTANCE_FILTER_HPP_
#define OBSTACLE_DETECTOR__WALL_DISTANCE_FILTER_HPP_

#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>

namespace obstacle_detector
{

class WallDistanceFilter
{
  public:
    struct Params
    {
        int occupied_thresh{50};   // occupancy value considered "occupied"
        double min_length_m{1.0};  // [m] required extent along the component's major axis
    };

    // Extract wall components from the map and build the distance transform. An empty/invalid
    // map leaves the filter inactive (isWallPoint always false).
    void buildFromMap(const nav_msgs::msg::OccupancyGrid &map, const Params &params);
    void clear();

    bool active() const { return active_; }

    // Distance [m] from (x_map, y_map) to the nearest wall cell; -1 when inactive or the point
    // falls outside the grid.
    double distanceToWall(double x_map, double y_map) const;
    // Layer-1 membership test: the point lies within assoc_dist [m] of a wall structure.
    bool isWallPoint(double x_map, double y_map, double assoc_dist) const;

  private:
    double origin_x_{0.0};
    double origin_y_{0.0};
    double resolution_{0.0};
    int width_{0};
    int height_{0};
    std::vector<float> dist_to_wall_;  // row-major per-cell distance to the nearest wall cell [m]
    bool active_{false};
};

}  // namespace obstacle_detector

#endif  // OBSTACLE_DETECTOR__WALL_DISTANCE_FILTER_HPP_
