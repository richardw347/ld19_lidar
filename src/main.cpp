#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "ld19_node.hpp"

auto main(int argc, char* argv[]) -> int
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  auto ld19_node = std::make_shared<LD19Node>("ld19_node");
  exe.add_node(ld19_node->get_node_base_interface());
  RCLCPP_INFO(ld19_node->get_logger(), "Starting to spin");
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
