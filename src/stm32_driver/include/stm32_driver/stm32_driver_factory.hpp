#pragma once

#include <memory>

#include "rclcpp/node.hpp"

namespace stm32_driver
{

std::shared_ptr<rclcpp::Node> make_stm32_driver();

}  // namespace stm32_driver
