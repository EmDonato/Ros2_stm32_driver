#pragma once

namespace stm32_driver
{

struct OdometryState
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

class DifferentialDriveOdometry
{
public:
  explicit DifferentialDriveOdometry(double wheel_base);

  void update(double left_velocity, double right_velocity, double dt);
  const OdometryState & state() const;

private:
  double wheel_base_;
  OdometryState state_;
};

}  // namespace stm32_driver
