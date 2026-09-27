#include "stm32_driver/stm32_driver_factory.hpp"

#include "rclcpp/rclcpp.hpp"

#include <exception>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(stm32_driver::make_stm32_driver());
  } catch (const std::exception & error) {
    if (rclcpp::ok()) {
      RCLCPP_FATAL(rclcpp::get_logger("stm32_driver"), "Driver startup failed: %s", error.what());
    }
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
