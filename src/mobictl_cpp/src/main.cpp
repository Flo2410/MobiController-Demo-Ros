#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "MobiCtl.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto mobi_node = std::make_shared<MobiCtl>();
  rclcpp::shutdown();
  return 0;
}