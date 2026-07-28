#include "particle_filter_cpp/particle_filter.hpp"

#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<particle_filter_cpp::ParticleFilter>());
  rclcpp::shutdown();
  return 0;
}
