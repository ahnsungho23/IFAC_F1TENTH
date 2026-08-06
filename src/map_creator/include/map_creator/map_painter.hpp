// Copyright 2026 2026_IFAC contributors
// Licensed under the Apache License, Version 2.0.

#ifndef MAP_CREATOR__MAP_PAINTER_HPP_
#define MAP_CREATOR__MAP_PAINTER_HPP_

#include <functional>
#include <string>

#include <opencv2/core.hpp>

#include <f110_msgs/msg/obstacle.hpp>

namespace map_creator
{

struct MapPainterConfig
{
  // A pixel counts as non-drivable when value < threshold (the conservative
  // complement of the generator's free predicate, load_map: free = pixel >= 250).
  int nondrivable_threshold{250};
  int side_samples_min{9};
  double ray_step_fraction{0.25};  // ray-march step = resolution * fraction
  double max_ray_length_m{5.0};    // wall_not_found beyond this lateral distance
};

// Paints obstacle rectangles and blocked-side wall polygons (black, value 0) on
// a working copy of the base map. Every paint session starts from the pristine
// original (immutable-baseline rule). ROS-free; Frenet->Cartesian conversion is
// injected so the painter shares the planner's exact reference geometry.
class MapPainter
{
public:
  // to_cart(s, d, x, y, yaw): planner's toCartesian (wrap-aware).
  using FrenetToCartesian =
    std::function<void (double, double, double &, double &, double &)>;

  explicit MapPainter(MapPainterConfig config = MapPainterConfig());

  bool loadBase(const std::string & base_yaml_path, std::string * error);
  void reset();  // working copy = pristine original

  // block_left=true paints the LEFT side of the obstacle to the left wall
  // (i.e. the planner chose to pass RIGHT). Also paints the obstacle body.
  bool paintObstacle(
    const f110_msgs::msg::Obstacle & obstacle,
    bool block_left,
    double track_length_m,
    const FrenetToCartesian & to_cart,
    std::string * error);

  // Writes <map_basename>.png + <map_basename>.yaml (tmp + rename, atomic).
  bool save(
    const std::string & output_dir, const std::string & map_basename,
    std::string * error) const;

  bool loaded() const {return !original_.empty();}
  double resolution() const {return resolution_;}
  const cv::Mat & working() const {return working_;}

private:
  bool worldToPixel(double x, double y, cv::Point & pixel) const;
  bool pixelNonDrivable(const cv::Point & pixel) const;

  MapPainterConfig config_;
  cv::Mat original_;
  cv::Mat working_;
  double resolution_{0.05};
  double origin_x_{0.0};
  double origin_y_{0.0};
  int negate_{0};
  double occupied_thresh_{0.65};
  double free_thresh_{0.196};
  std::string mode_{"trinary"};
};

}  // namespace map_creator

#endif  // MAP_CREATOR__MAP_PAINTER_HPP_
