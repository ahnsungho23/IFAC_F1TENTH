// Copyright 2026 2026_IFAC contributors
// Licensed under the Apache License, Version 2.0.

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

#include <opencv2/imgcodecs.hpp>

#include "map_creator/map_painter.hpp"

namespace
{

namespace fs = std::filesystem;

// Synthetic 4 m x 2 m corridor map at 0.05 m/px (80 x 40 px): free (254)
// everywhere except 2-px occupied (0) bands at the top and bottom rows.
// World frame: origin (0,0) at bottom-left; straight reference along +x with
// Frenet s = x, d = y - 1.0 (corridor centre line at y = 1.0).
class PainterFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    dir_ = fs::temp_directory_path() / "map_creator_painter_test";
    fs::create_directories(dir_);
    cv::Mat img(40, 80, CV_8UC1, cv::Scalar(254));
    img.rowRange(0, 2).setTo(0);    // top wall (world y ~ [1.9, 2.0])
    img.rowRange(38, 40).setTo(0);  // bottom wall (world y ~ [0.0, 0.1])
    cv::imwrite((dir_ / "base.png").string(), img);
    std::ofstream yaml(dir_ / "base.yaml");
    yaml << "image: base.png\nmode: trinary\nresolution: 0.05\n"
         << "origin: [0.0, 0.0, 0]\nnegate: 0\n"
         << "occupied_thresh: 0.65\nfree_thresh: 0.196\n";
  }

  static void toCart(double s, double d, double & x, double & y, double & yaw)
  {
    x = s;
    y = 1.0 + d;
    yaw = 0.0;
  }

  f110_msgs::msg::Obstacle makeObstacle() const
  {
    f110_msgs::msg::Obstacle ob;
    ob.id = 7;
    ob.s_start = 1.9;
    ob.s_end = 2.1;
    ob.d_right = -0.1;
    ob.d_left = 0.1;
    ob.s_center = 2.0;
    ob.d_center = 0.0;
    return ob;
  }

  fs::path dir_;
};

}  // namespace

TEST_F(PainterFixture, BlocksRightSideToWallKeepsLeftOpen)
{
  map_creator::MapPainter painter;
  std::string error;
  ASSERT_TRUE(painter.loadBase((dir_ / "base.yaml").string(), &error)) << error;

  const auto ob = makeObstacle();
  // Pass LEFT -> block RIGHT (block_left = false): paint from d=-0.1 downward.
  ASSERT_TRUE(painter.paintObstacle(ob, false, 100.0, &PainterFixture::toCart, &error))
    << error;

  const cv::Mat & img = painter.working();
  auto pixel_at = [&img](double x, double y) {
      const int col = static_cast<int>(x / 0.05);
      const int row = img.rows - 1 - static_cast<int>(y / 0.05);
      return img.at<uint8_t>(row, col);
    };

  EXPECT_EQ(pixel_at(2.0, 1.0), 0);    // obstacle body painted
  EXPECT_EQ(pixel_at(2.0, 0.5), 0);    // blocked corridor between obstacle and bottom wall
  EXPECT_EQ(pixel_at(2.0, 1.5), 254);  // chosen (left) side stays open
  EXPECT_EQ(pixel_at(0.5, 1.0), 254);  // upstream corridor untouched

  ASSERT_TRUE(painter.save(dir_.string(), "obstacle_map", &error)) << error;
  EXPECT_TRUE(fs::exists(dir_ / "obstacle_map.png"));
  EXPECT_TRUE(fs::exists(dir_ / "obstacle_map.yaml"));

  // reset() must restore the pristine base (immutable-baseline rule).
  painter.reset();
  EXPECT_EQ(painter.working().at<uint8_t>(20, 40), 254);
}

TEST_F(PainterFixture, WallNotFoundFailsInsteadOfSilentSuccess)
{
  // Open map with no walls at all: ray march must report wall_not_found.
  cv::Mat img(40, 80, CV_8UC1, cv::Scalar(254));
  cv::imwrite((dir_ / "open.png").string(), img);
  std::ofstream yaml(dir_ / "open.yaml");
  yaml << "image: open.png\nmode: trinary\nresolution: 0.05\n"
       << "origin: [0.0, 0.0, 0]\nnegate: 0\n"
       << "occupied_thresh: 0.65\nfree_thresh: 0.196\n";
  yaml.close();

  map_creator::MapPainter painter;
  std::string error;
  ASSERT_TRUE(painter.loadBase((dir_ / "open.yaml").string(), &error)) << error;
  EXPECT_FALSE(
    painter.paintObstacle(makeObstacle(), false, 100.0, &PainterFixture::toCart, &error));
  EXPECT_NE(error.find("wall_not_found"), std::string::npos) << error;
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
