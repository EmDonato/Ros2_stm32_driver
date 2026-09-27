#include "stm32_driver/differential_drive_odometry.hpp"

#include <cmath>
#include <stdexcept>

namespace stm32_driver
{

DifferentialDriveOdometry::DifferentialDriveOdometry(double wheel_base)
: wheel_base_(wheel_base)
{
  if (wheel_base_ <= 0.0) {
    throw std::invalid_argument("wheel_base must be greater than zero");
  }
}

void DifferentialDriveOdometry::update(
  double left_velocity, double right_velocity, double dt)
{
  if (dt <= 0.0) {
    return;
  }

  const double linear_velocity = (right_velocity + left_velocity) / 2.0;
  const double angular_velocity = (right_velocity - left_velocity) / wheel_base_;
  const double heading = state_.yaw + angular_velocity * dt * 0.5;

  state_.x += linear_velocity * std::cos(heading) * dt;
  state_.y += linear_velocity * std::sin(heading) * dt;
  state_.yaw = std::atan2(
    std::sin(state_.yaw + angular_velocity * dt),
    std::cos(state_.yaw + angular_velocity * dt));
}

const OdometryState & DifferentialDriveOdometry::state() const
{
  return state_;
}

}  // namespace stm32_driver
