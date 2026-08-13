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
//   2. Each component is tested for linearity with a 2x2 PCA on its cell coordinates: a component
//      is a WALL when the eigenvalue ratio (lambda_max/lambda_min) >= linear_ratio AND its extent
//      along the major axis >= min_length_m. Compact blobs (obstacles/debris baked into the map)
//      fail the test, so points near them stay obstacle candidates.
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
        double linear_ratio{4.0};  // PCA eigenvalue ratio (lmax/lmin) for a component to be a wall
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
