// Copyright 2026 2026_IFAC contributors
// Licensed under the Apache License, Version 2.0.

#include "map_creator/map_painter.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace map_creator
{

namespace fs = std::filesystem;

MapPainter::MapPainter(MapPainterConfig config)
: config_(config)
{
}

bool MapPainter::loadBase(const std::string & base_yaml_path, std::string * error)
{
  YAML::Node node;
  try {
    node = YAML::LoadFile(base_yaml_path);
  } catch (const std::exception & e) {
    if (error) {*error = std::string("cannot parse map yaml: ") + e.what();}
    return false;
  }
  if (!node["image"] || !node["resolution"] || !node["origin"]) {
    if (error) {*error = "map yaml missing image/resolution/origin";}
    return false;
  }
  resolution_ = node["resolution"].as<double>();
  origin_x_ = node["origin"][0].as<double>();
  origin_y_ = node["origin"][1].as<double>();
  negate_ = node["negate"] ? node["negate"].as<int>() : 0;
  occupied_thresh_ = node["occupied_thresh"] ? node["occupied_thresh"].as<double>() : 0.65;
  free_thresh_ = node["free_thresh"] ? node["free_thresh"].as<double>() : 0.196;
  mode_ = node["mode"] ? node["mode"].as<std::string>() : "trinary";

  fs::path image_path(node["image"].as<std::string>());
  if (image_path.is_relative()) {
    image_path = fs::path(base_yaml_path).parent_path() / image_path;
  }
  original_ = cv::imread(image_path.string(), cv::IMREAD_GRAYSCALE);
  if (original_.empty()) {
    if (error) {*error = "cannot read map image: " + image_path.string();}
    return false;
  }
  reset();
  return true;
}

void MapPainter::reset()
{
  working_ = original_.clone();
}

bool MapPainter::worldToPixel(double x, double y, cv::Point & pixel) const
{
  const double col = (x - origin_x_) / resolution_;
  const double row_from_bottom = (y - origin_y_) / resolution_;
  const int col_i = static_cast<int>(std::floor(col));
  const int row_i = working_.rows - 1 - static_cast<int>(std::floor(row_from_bottom));
  if (col_i < 0 || col_i >= working_.cols || row_i < 0 || row_i >= working_.rows) {
    return false;
  }
  pixel = cv::Point(col_i, row_i);
  return true;
}

bool MapPainter::pixelNonDrivable(const cv::Point & pixel) const
{
  return working_.at<uint8_t>(pixel) < config_.nondrivable_threshold;
}

bool MapPainter::paintObstacle(
  const f110_msgs::msg::Obstacle & obstacle,
  bool block_left,
  double track_length_m,
  const FrenetToCartesian & to_cart,
  std::string * error)
{
  if (working_.empty()) {
    if (error) {*error = "painter has no base map";}
    return false;
  }

  double span = obstacle.s_end - obstacle.s_start;
  if (track_length_m > 0.0) {
    while (span < 0.0) {span += track_length_m;}
  }
  if (span <= 0.0) {span = 1e-3;}

  const int n_samples = std::max(
    config_.side_samples_min,
    static_cast<int>(std::ceil(span / (resolution_ * 0.5))) + 1);

  const double face_d = block_left ? obstacle.d_left : obstacle.d_right;
  const double direction = block_left ? 1.0 : -1.0;  // +d = left of the raceline
  const double step_d = resolution_ * config_.ray_step_fraction;
  const int max_steps =
    static_cast<int>(std::ceil(config_.max_ray_length_m / step_d));

  std::vector<cv::Point> face_pixels;
  std::vector<cv::Point> wall_pixels;
  face_pixels.reserve(static_cast<std::size_t>(n_samples));
  wall_pixels.reserve(static_cast<std::size_t>(n_samples));

  for (int i = 0; i < n_samples; ++i) {
    const double s =
      obstacle.s_start + span * static_cast<double>(i) / static_cast<double>(n_samples - 1);
    double x = 0.0, y = 0.0, yaw = 0.0;
    to_cart(s, face_d, x, y, yaw);
    cv::Point face_px;
    if (!worldToPixel(x, y, face_px)) {
      if (error) {*error = "obstacle face outside map extent (s=" + std::to_string(s) + ")";}
      return false;
    }
    face_pixels.push_back(face_px);

    bool wall_found = false;
    for (int k = 1; k <= max_steps; ++k) {
      const double d = face_d + direction * step_d * static_cast<double>(k);
      to_cart(s, d, x, y, yaw);
      cv::Point px;
      if (!worldToPixel(x, y, px)) {
        break;  // left the image before hitting a wall -> wall_not_found
      }
      if (pixelNonDrivable(px)) {
        wall_pixels.push_back(px);
        wall_found = true;
        break;
      }
    }
    if (!wall_found) {
      if (error) {
        *error = "wall_not_found at s=" + std::to_string(s) +
          (block_left ? " (left side)" : " (right side)");
      }
      return false;
    }
  }

  // Blocked-side polygon: obstacle face forward + wall trace backward.
  std::vector<cv::Point> polygon = face_pixels;
  polygon.insert(polygon.end(), wall_pixels.rbegin(), wall_pixels.rend());
  const std::vector<std::vector<cv::Point>> polys{polygon};
  cv::fillPoly(working_, polys, cv::Scalar(0));

  // Obstacle body: sampled left edge forward + right edge backward.
  std::vector<cv::Point> body;
  body.reserve(static_cast<std::size_t>(2 * n_samples));
  for (int pass = 0; pass < 2; ++pass) {
    const double edge_d = (pass == 0) ? obstacle.d_left : obstacle.d_right;
    for (int i = 0; i < n_samples; ++i) {
      const int idx = (pass == 0) ? i : (n_samples - 1 - i);
      const double s =
        obstacle.s_start + span * static_cast<double>(idx) / static_cast<double>(n_samples - 1);
      double x = 0.0, y = 0.0, yaw = 0.0;
      to_cart(s, edge_d, x, y, yaw);
      cv::Point px;
      if (worldToPixel(x, y, px)) {
        body.push_back(px);
      }
    }
  }
  if (body.size() >= 3U) {
    const std::vector<std::vector<cv::Point>> body_polys{body};
    cv::fillPoly(working_, body_polys, cv::Scalar(0));
  }
  return true;
}

bool MapPainter::save(
  const std::string & output_dir, const std::string & map_basename,
  std::string * error) const
{
  if (working_.empty()) {
    if (error) {*error = "nothing to save";}
    return false;
  }
  std::error_code ec;
  fs::create_directories(output_dir, ec);

  const fs::path png_path = fs::path(output_dir) / (map_basename + ".png");
  const fs::path png_tmp = fs::path(output_dir) / ("." + map_basename + ".png");
  if (!cv::imwrite(png_tmp.string(), working_)) {
    if (error) {*error = "cannot write " + png_tmp.string();}
    return false;
  }
  fs::rename(png_tmp, png_path, ec);
  if (ec) {
    if (error) {*error = "rename failed: " + ec.message();}
    return false;
  }

  const fs::path yaml_path = fs::path(output_dir) / (map_basename + ".yaml");
  const fs::path yaml_tmp = fs::path(output_dir) / ("." + map_basename + ".yaml.tmp");
  {
    std::ofstream out(yaml_tmp);
    if (!out) {
      if (error) {*error = "cannot write " + yaml_tmp.string();}
      return false;
    }
    out << "image: " << map_basename << ".png\n";
    out << "mode: " << mode_ << "\n";
    out << "resolution: " << resolution_ << "\n";
    out << "origin: [" << origin_x_ << ", " << origin_y_ << ", 0]\n";
    out << "negate: " << negate_ << "\n";
    out << "occupied_thresh: " << occupied_thresh_ << "\n";
    out << "free_thresh: " << free_thresh_ << "\n";
  }
  fs::rename(yaml_tmp, yaml_path, ec);
  if (ec) {
    if (error) {*error = "rename failed: " + ec.message();}
    return false;
  }
  return true;
}

}  // namespace map_creator
