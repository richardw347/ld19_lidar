#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "ld19_node.hpp"

auto main(int argc, char* argv[]) -> int
{
  rclcpp::init(argc, argv);
  auto ld19_node = std::make_shared<LD19Node>();
  if (!ld19_node->init_device())
  {
    RCLCPP_ERROR(ld19_node->get_logger(), "Error initialising LD19");
    return -1;
  }
  RCLCPP_INFO(ld19_node->get_logger(), "Starting to spin");
  rclcpp::spin(ld19_node);
  rclcpp::shutdown();
  return 0;
}