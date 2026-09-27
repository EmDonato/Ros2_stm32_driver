#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "stm32_driver/differential_drive_odometry.hpp"
#include "stm32_driver/packet_parser.hpp"
#include "stm32_driver/parsing_types.hpp"
#include "stm32_driver/serial_port.hpp"
#include "stm32_driver/stm32_driver_factory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/wheel_speeds.hpp"
#include "robot_interfaces/msg/imu_euler.hpp"
#include "robot_interfaces/srv/control_command.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "std_msgs/msg/bool.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>


class Stm32Driver : public rclcpp::Node
{
public:
  Stm32Driver()
  : Node("stm32_driver"),
    verbose_logger_(this->get_logger().get_child("verbose_logger")),
    last_stamp_(0, 0, RCL_ROS_TIME),
    last_enc_(0, 0, RCL_ROS_TIME)
  {
    // ============================================================
    // Parameters
    // ============================================================

    port_ =
      this->declare_parameter<std::string>(
      "port",
      "/dev/ttyACM0");

    baudrate_ =
      this->declare_parameter<int>(
      "baudrate",
      115200);

    frequency_ms_ =
      this->declare_parameter<int>(
      "frequency_ms",
      5);

    cmd_frequency_ms_ =
      this->declare_parameter<int>(
      "cmd_frequency_ms",
      20);

    radius_ =
      this->declare_parameter<double>(
      "radius",
      0.0346);

    wheel_base_ =
      this->declare_parameter<double>(
      "wheel_base",
      0.2);

    cmd_timeout_s_ =
      this->declare_parameter<double>(
      "cmd_timeout_s",
      0.5);

    const auto control_service =
      this->declare_parameter<std::string>(
      "control_service",
      "control/command");

    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");
    imu_parent_frame_id_ = declare_parameter<std::string>("imu_parent_frame_id", "chassis");
    imu_frame_id_ = declare_parameter<std::string>("imu_frame_id", "imu_link");
    encoder_odom_frame_id_ =
      declare_parameter<std::string>("encoder_odom_frame_id", "enc_odom");
    publish_imu_tf_ = declare_parameter<bool>("publish_imu_tf", true);
    imu_x_ = declare_parameter<double>("imu_transform.x", 0.0);
    imu_y_ = declare_parameter<double>("imu_transform.y", 0.0);
    imu_z_ = declare_parameter<double>("imu_transform.z", 0.0);
    imu_roll_ = declare_parameter<double>("imu_transform.roll", 0.0);
    imu_pitch_ = declare_parameter<double>("imu_transform.pitch", 0.0);
    imu_yaw_ = declare_parameter<double>("imu_transform.yaw", 0.0);

    imu_flip_x_ = declare_parameter<bool>("imu_flip_x", true);
    imu_acceleration_scale_ = declare_parameter<double>("imu_acceleration_scale", 9.80665);
    imu_orientation_variance_ = declare_parameter<double>("imu_orientation_variance", -1.0);

    validate_parameters_();

    odometry_ = std::make_unique<stm32_driver::DifferentialDriveOdometry>(wheel_base_);


    // ============================================================
    // Serial
    // ============================================================

    try {
      serial_port_ = std::make_unique<stm32_driver::SerialPort>(port_, baudrate_);

    } catch (const std::exception & error) {

      RCLCPP_FATAL(
        this->get_logger(),
        "Could not configure serial port: %s",
        error.what());

      throw;
    }


    // ============================================================
    // Publishers
    // ============================================================

    imu_pub_ =
      create_publisher<sensor_msgs::msg::Imu>(
      "imu",
      rclcpp::SensorDataQoS());


    enc_pub_ =
      create_publisher<geometry_msgs::msg::TwistStamped>(
      "enc/twist_meas",
      rclcpp::SensorDataQoS());

    odom_pub_ =
      create_publisher<nav_msgs::msg::Odometry>(
      "odom/solo",
      rclcpp::QoS(10));

    imu_debug_pub_ = create_publisher<robot_interfaces::msg::ImuEuler>(
      "debug/imu", rclcpp::QoS(10));

    if (publish_imu_tf_) {
      static_tf_broadcaster_ =
        std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
      publish_imu_transform_();
    }


    joint_pub_ =
      create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states",
      10);


    wheels_pub_ =
      create_publisher<robot_interfaces::msg::WheelSpeeds>(
      "enc/twist_wheels",
      rclcpp::SensorDataQoS());


    mag_pub_ =
      create_publisher<sensor_msgs::msg::MagneticField>(
      "magnetometer",
      rclcpp::SensorDataQoS());


    arm_pub_ =
      create_publisher<std_msgs::msg::Bool>(
      "robot_status/is_armed",
      rclcpp::QoS(
        rclcpp::KeepLast(1))
      .transient_local());


    accepted_pub_ =
      create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel/accepted",
      rclcpp::SensorDataQoS());


    // ============================================================
    // Velocity reference subscriber
    //
    // Latest command wins.
    // ============================================================

    ref_sub_ =
      create_subscription<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel",
      rclcpp::SensorDataQoS(),

      std::bind(
        &Stm32Driver::velocity_callback_,
        this,
        std::placeholders::_1));


    // ============================================================
    // Control service
    // ============================================================

    control_service_ =
      create_service<robot_interfaces::srv::ControlCommand>(
      control_service,

      std::bind(
        &Stm32Driver::handle_control_service_,
        this,
        std::placeholders::_1,
        std::placeholders::_2));


    // ============================================================
    // Serial receive timer
    // ============================================================

    serial_timer_ =
      create_wall_timer(
      std::chrono::milliseconds(
        frequency_ms_),

      std::bind(
        &Stm32Driver::read_serial_,
        this));


    // ============================================================
    // Velocity reference timer
    //
    // Sends the CURRENT reference to STM32 at fixed rate.
    //
    // Default:
    // 20 ms -> 50 Hz
    // ============================================================

    command_timer_ =
      create_wall_timer(
      std::chrono::milliseconds(
        cmd_frequency_ms_),

      std::bind(
        &Stm32Driver::send_reference_,
        this));


    // ============================================================
    // Initial reset
    // ============================================================

    if (!send_command_(
        static_cast<uint8_t>(
          RESET_ID)))
    {
      RCLCPP_WARN(
        this->get_logger(),
        "The initial reset command could not be sent");
    }


    RCLCPP_INFO(
      this->get_logger(),
      "STM32 driver started on %s at %d baud",
      port_.c_str(),
      baudrate_);

    RCLCPP_INFO(
      this->get_logger(),
      "Serial polling: %d ms",
      frequency_ms_);

    RCLCPP_INFO(
      this->get_logger(),
      "Velocity reference rate: %.1f Hz",
      1000.0 /
      static_cast<double>(
        cmd_frequency_ms_));

    RCLCPP_INFO(
      this->get_logger(),
      "cmd_vel timeout: %.3f s",
      cmd_timeout_s_);
  }


  ~Stm32Driver() override
  {
    RCLCPP_INFO(this->get_logger(), "Serial port closed");
  }


private:

  // ============================================================
  // Constants
  // ============================================================

  static constexpr double kTwoPi =
    6.28318530717958647692;


  // ============================================================
  // Configuration
  // ============================================================

  rclcpp::Logger verbose_logger_;

  std::string port_;

  int baudrate_{115200};

  // Serial receive polling period
  int frequency_ms_{5};

  // Velocity TX period to STM32
  int cmd_frequency_ms_{20};

  double radius_{0.0346};

  double wheel_base_{0.2};

  double cmd_timeout_s_{0.5};

  std::unique_ptr<stm32_driver::SerialPort> serial_port_;
  std::unique_ptr<stm32_driver::DifferentialDriveOdometry> odometry_;

  std::string base_frame_id_{"base_link"};
  std::string imu_parent_frame_id_{"chassis"};
  std::string imu_frame_id_{"imu_link"};
  std::string encoder_odom_frame_id_{"enc_odom"};
  bool imu_flip_x_{true};
  double imu_acceleration_scale_{9.80665};
  double imu_orientation_variance_{-1.0};
  bool publish_imu_tf_{true};
  double imu_x_{0.0};
  double imu_y_{0.0};
  double imu_z_{0.0};
  double imu_roll_{0.0};
  double imu_pitch_{0.0};
  double imu_yaw_{0.0};


  // ============================================================
  // Parser
  // ============================================================

  stm32_driver::PacketParser packet_parser_;
  uint8_t message_type_{0};
  std::vector<uint8_t> payload_;


  // ============================================================
  // Robot state
  // ============================================================

  double left_wheel_position_{0.0};

  double right_wheel_position_{0.0};

  bool last_arm_state_{true};


  // ============================================================
  // Velocity reference state
  // ============================================================

  // Last command received from ROS
  float latest_v_{0.0f};

  float latest_w_{0.0f};

  // Have we ever received a valid command?
  bool have_cmd_{false};


  // ============================================================
  // Time state
  // ============================================================

  // Stamp generated upstream with TwistStamped
  rclcpp::Time last_stamp_;

  // Local time of previous encoder message
  rclcpp::Time last_enc_;


  // ============================================================
  // ROS interfaces
  // ============================================================

  rclcpp::Publisher<
    sensor_msgs::msg::Imu>::SharedPtr
  imu_pub_;


  rclcpp::Publisher<
    sensor_msgs::msg::MagneticField>::SharedPtr
  mag_pub_;


  rclcpp::Publisher<
    sensor_msgs::msg::JointState>::SharedPtr
  joint_pub_;


  rclcpp::Publisher<
    geometry_msgs::msg::TwistStamped>::SharedPtr
  enc_pub_;

  rclcpp::Publisher<
    nav_msgs::msg::Odometry>::SharedPtr
  odom_pub_;

  rclcpp::Publisher<robot_interfaces::msg::ImuEuler>::SharedPtr imu_debug_pub_;

  std::unique_ptr<tf2_ros::StaticTransformBroadcaster>
  static_tf_broadcaster_;


  rclcpp::Publisher<
    geometry_msgs::msg::Twist>::SharedPtr
  accepted_pub_;


  rclcpp::Publisher<
    std_msgs::msg::Bool>::SharedPtr
  arm_pub_;


  rclcpp::Publisher<
    robot_interfaces::msg::WheelSpeeds>::SharedPtr
  wheels_pub_;


  rclcpp::Subscription<
    geometry_msgs::msg::TwistStamped>::SharedPtr
  ref_sub_;


  rclcpp::Service<
    robot_interfaces::srv::ControlCommand>::SharedPtr
  control_service_;


  rclcpp::TimerBase::SharedPtr
  serial_timer_;


  rclcpp::TimerBase::SharedPtr
  command_timer_;


  // ============================================================
  // Parameters validation
  // ============================================================

  void validate_parameters_() const
  {
    if (frequency_ms_ <= 0) {

      throw std::invalid_argument(
              "frequency_ms must be greater than zero");
    }


    if (cmd_frequency_ms_ <= 0) {

      throw std::invalid_argument(
              "cmd_frequency_ms must be greater than zero");
    }


    if (radius_ <= 0.0) {

      throw std::invalid_argument(
              "radius must be greater than zero");
    }


    if (wheel_base_ <= 0.0) {

      throw std::invalid_argument(
              "wheel_base must be greater than zero");
    }


    if (!std::isfinite(imu_acceleration_scale_) || imu_acceleration_scale_ <= 0.0 ||
      !std::isfinite(imu_orientation_variance_) ||
      (imu_orientation_variance_ != -1.0 && imu_orientation_variance_ <= 0.0))
    {
      throw std::invalid_argument(
              "IMU scale must be positive; orientation variance must be positive or -1");
    }

    if (cmd_timeout_s_ <= 0.0) {

      throw std::invalid_argument(
              "cmd_timeout_s must be greater than zero");
    }

    if (
      base_frame_id_.empty() || imu_parent_frame_id_.empty() || imu_frame_id_.empty() ||
      encoder_odom_frame_id_.empty())
    {
      throw std::invalid_argument("TF frame IDs must not be empty");
    }

    if (
      base_frame_id_ == imu_frame_id_ || imu_parent_frame_id_ == imu_frame_id_ ||
      encoder_odom_frame_id_ == base_frame_id_)
    {
      throw std::invalid_argument(
              "Odometry/base/IMU frame IDs must be distinct and IMU cannot parent itself");
    }
  }


  void publish_imu_transform_()
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = imu_parent_frame_id_;
    transform.child_frame_id = imu_frame_id_;
    transform.transform.translation.x = imu_x_;
    transform.transform.translation.y = imu_y_;
    transform.transform.translation.z = imu_z_;

    tf2::Quaternion rotation;
    rotation.setRPY(imu_roll_, imu_pitch_, imu_yaw_);
    transform.transform.rotation.x = rotation.x();
    transform.transform.rotation.y = rotation.y();
    transform.transform.rotation.z = rotation.z();
    transform.transform.rotation.w = rotation.w();
    static_tf_broadcaster_->sendTransform(transform);
  }


  // ============================================================
  // Serial receive
  // ============================================================

  void read_serial_()
  {
    try {
      const auto bytes = serial_port_->read_available();
      auto batch = packet_parser_.consume(bytes);
      if (batch.checksum_errors > 0) {
        RCLCPP_WARN(
          this->get_logger(), "%zu serial packet(s) failed checksum validation",
          batch.checksum_errors);
      }
      for (auto & packet : batch.packets) {
        message_type_ = packet.type;
        payload_ = std::move(packet.payload);
        handle_message_();
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(this->get_logger(), "%s", error.what());
    }
  }



  // ============================================================
  // Dispatcher
  // ============================================================

  void handle_message_()
  {
    switch (message_type_) {

      case IMU_ID:

        handle_imu_message_();

        break;


      case ENC_ID:

        handle_encoder_message_();

        break;


      case HB_ID:

        handle_heartbeat_message_();

        break;


      case STRING_ID:

        handle_string_message_();

        break;


      default:

        break;
    }
  }


  // ============================================================
  // IMU
  // ============================================================

  void handle_imu_message_()
  {
    constexpr size_t expected_length =
      sizeof(uint32_t) +
      15 * sizeof(float);


    if (
      payload_.size() !=
      expected_length)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid IMU payload length: %zu",
        payload_.size());

      return;
    }


    float data[15]{};


    std::memcpy(
      data,
      payload_.data() +
      sizeof(uint32_t),
      sizeof(data));


    const rclcpp::Time now =
      this->now();


    sensor_msgs::msg::Imu imu;


    imu.header.stamp =
      now;

    imu.header.frame_id =
      imu_frame_id_;


    tf2::Quaternion orientation;


    orientation.setRPY(
      data[0],
      data[1],
      data[2]);


    // R maps raw sensor vectors into imu_link: (x, y, z) -> (x, -y, -z).
    // Orientation maps body to world, hence q_world_link = q_world_raw * inverse(q_link_raw).
    const double yz_sign = imu_flip_x_ ? -1.0 : 1.0;
    if (imu_flip_x_) {
      orientation *= tf2::Quaternion(-1.0, 0.0, 0.0, 0.0);
    }
    orientation.normalize();
    if (imu_orientation_variance_ < 0.0) {
      imu.orientation_covariance[0] = -1.0;
    } else {
      imu.orientation_covariance[0] = imu_orientation_variance_;
      imu.orientation_covariance[4] = imu_orientation_variance_;
      imu.orientation_covariance[8] = imu_orientation_variance_;
    }

    imu.orientation.x =
      orientation.x();

    imu.orientation.y =
      orientation.y();

    imu.orientation.z =
      orientation.z();

    imu.orientation.w =
      orientation.w();


    imu.linear_acceleration.x =
      data[3] * imu_acceleration_scale_;

    imu.linear_acceleration.y =
      data[4] * imu_acceleration_scale_ * yz_sign;

    imu.linear_acceleration.z =
      data[5] * imu_acceleration_scale_ * yz_sign;


    imu.angular_velocity.x =
      data[6];

    imu.angular_velocity.y =
      data[7] * yz_sign;

    imu.angular_velocity.z =
      data[8] * yz_sign;


    imu.angular_velocity_covariance = {

      1.28e-6, 0, 0,

      0, 1.13e-6, 0,

      0, 0, 1.38e-6
    };


    imu.linear_acceleration_covariance = {

      2.43e-4, 2.81e-5, 1.43e-5,

      2.81e-5, 1.82e-4, 1.52e-5,

      1.43e-5, 1.52e-5, 1.80e-4
    };


    // Covariance follows C_link = scale^2 * R * C_raw * R^T.
    const double signs[3] = {1.0, yz_sign, yz_sign};
    for (size_t row = 0; row < 3; ++row) {
      for (size_t col = 0; col < 3; ++col) {
        imu.linear_acceleration_covariance[row * 3 + col] *=
          imu_acceleration_scale_ * imu_acceleration_scale_ * signs[row] * signs[col];
        imu.angular_velocity_covariance[row * 3 + col] *= signs[row] * signs[col];
      }
    }

    imu_pub_->publish(
      imu);
    // Report the same corrected orientation as /imu, not integrated gyro attitude.
    robot_interfaces::msg::ImuEuler debug;
    debug.header = imu.header;
    tf2::Matrix3x3(orientation).getRPY(debug.roll_rad, debug.pitch_rad, debug.yaw_rad);
    constexpr double rad_to_deg = 180.0 / 3.14159265358979323846;
    debug.roll_deg = debug.roll_rad * rad_to_deg;
    debug.pitch_deg = debug.pitch_rad * rad_to_deg;
    debug.yaw_deg = debug.yaw_rad * rad_to_deg;
    imu_debug_pub_->publish(debug);


    sensor_msgs::msg::MagneticField
      magnetic_field;


    magnetic_field.header.stamp =
      now;

    magnetic_field.header.frame_id =
      imu_frame_id_;


    magnetic_field.magnetic_field.x =
      data[9];

    magnetic_field.magnetic_field.y =
      data[10] * yz_sign;

    magnetic_field.magnetic_field.z =
      data[11] * yz_sign;


    mag_pub_->publish(
      magnetic_field);
  }


  // ============================================================
  // Encoders
  // ============================================================

  void handle_encoder_message_()
  {
    constexpr size_t expected_length =
      sizeof(uint32_t) +
      4 * sizeof(float) +
      2 * sizeof(uint32_t);


    if (
      payload_.size() <
      expected_length)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid encoder payload length: %zu",
        payload_.size());

      return;
    }


    float encoder_data[4]{};


    std::memcpy(
      encoder_data,
      payload_.data() +
      sizeof(uint32_t),
      sizeof(
        encoder_data));


    uint32_t pwm_data[2]{};


    std::memcpy(
      pwm_data,
      payload_.data() +
      sizeof(uint32_t) +
      sizeof(
        encoder_data),
      sizeof(
        pwm_data));


    // ==========================================================
    // Wheel angular velocities
    // RPM -> rad/s
    // ==========================================================

    const double omega_left =
      static_cast<double>(
        encoder_data[2])
      *
      kTwoPi /
      60.0;


    const double omega_right =
      static_cast<double>(
        encoder_data[3])
      *
      kTwoPi /
      60.0;


    // ==========================================================
    // Wheel linear velocities
    // ==========================================================

    const double velocity_left =
      omega_left *
      radius_;


    const double velocity_right =
      omega_right *
      radius_;


    // ==========================================================
    // Integrate wheel positions
    // ==========================================================

    const rclcpp::Time now =
      this->now();

    double dt = 0.0;


    if (
      last_enc_.nanoseconds()
      != 0)
    {
      dt =
        (now -
        last_enc_)
        .seconds();


      if (
        dt > 0.0 &&
        dt < 0.5)
      {
        left_wheel_position_ +=
          omega_left *
          dt;


        right_wheel_position_ +=
          omega_right *
          dt;

        odometry_->update(velocity_left, velocity_right, dt);
      }
    }


    last_enc_ =
      now;


    // ==========================================================
    // WheelSpeeds
    // ==========================================================

    robot_interfaces::msg::WheelSpeeds
      wheel_speeds;


    wheel_speeds.header.stamp =
      now;

    wheel_speeds.header.frame_id =
      base_frame_id_;


    wheel_speeds.speed[0] =
      velocity_left;

    wheel_speeds.speed[1] =
      velocity_right;


    wheel_speeds.pwm[0] =
      pwm_data[0];

    wheel_speeds.pwm[1] =
      pwm_data[1];


    // ==========================================================
    // Differential-drive Twist
    // ==========================================================

    geometry_msgs::msg::TwistStamped
      measured_twist;


    measured_twist.header.stamp =
      now;

    measured_twist.header.frame_id =
      base_frame_id_;


    measured_twist.twist.linear.x =
      (
        velocity_right +
        velocity_left
      ) /
      2.0;


    measured_twist.twist.angular.z =
      (
        velocity_right -
        velocity_left
      ) /
      wheel_base_;

    const auto & odom_state = odometry_->state();
    tf2::Quaternion odom_orientation;
    odom_orientation.setRPY(0.0, 0.0, odom_state.yaw);

    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = now;
    odometry.header.frame_id = encoder_odom_frame_id_;
    odometry.child_frame_id = base_frame_id_;
    odometry.pose.pose.position.x = odom_state.x;
    odometry.pose.pose.position.y = odom_state.y;
    odometry.pose.pose.orientation.x = odom_orientation.x();
    odometry.pose.pose.orientation.y = odom_orientation.y();
    odometry.pose.pose.orientation.z = odom_orientation.z();
    odometry.pose.pose.orientation.w = odom_orientation.w();
    odometry.twist.twist = measured_twist.twist;

    // The driver has no direct measurement of z, roll or pitch. Large
    // variances prevent downstream filters from treating those axes as known.
    odometry.pose.covariance[0] = 0.02;
    odometry.pose.covariance[7] = 0.02;
    odometry.pose.covariance[14] = 1.0e6;
    odometry.pose.covariance[21] = 1.0e6;
    odometry.pose.covariance[28] = 1.0e6;
    odometry.pose.covariance[35] = 0.05;
    odometry.twist.covariance[0] = 0.02;
    odometry.twist.covariance[7] = 1.0e6;
    odometry.twist.covariance[14] = 1.0e6;
    odometry.twist.covariance[21] = 1.0e6;
    odometry.twist.covariance[28] = 1.0e6;
    odometry.twist.covariance[35] = 0.05;


    // ==========================================================
    // Joint states
    // ==========================================================

    sensor_msgs::msg::JointState
      joint_state;


    joint_state.header.stamp =
      now;


    joint_state.name = {

      "left_wheel_joint",

      "right_wheel_joint"
    };


    joint_state.position = {

      left_wheel_position_,

      right_wheel_position_
    };


    joint_state.velocity = {

      omega_left,

      omega_right
    };


    enc_pub_->publish(
      measured_twist);

    odom_pub_->publish(odometry);

    joint_pub_->publish(
      joint_state);


    wheels_pub_->publish(
      wheel_speeds);
  }


  // ============================================================
  // Heartbeat
  // ============================================================

  void handle_heartbeat_message_()
  {
    constexpr size_t expected_length =
      2 *
      sizeof(uint32_t);


    if (
      payload_.size() <
      expected_length)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid heartbeat payload length: %zu",
        payload_.size());

      return;
    }


    uint32_t heartbeat_data[2]{};


    std::memcpy(
      heartbeat_data,
      payload_.data(),
      sizeof(
        heartbeat_data));


    const bool current_arm_state =
      heartbeat_data[1] == 1U;


    if (
      current_arm_state !=
      last_arm_state_)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Robot hardware is %s",
        current_arm_state
        ?
        "armed"
        :
        "disarmed");
    }


    last_arm_state_ =
      current_arm_state;


    std_msgs::msg::Bool
      arm_state;


    arm_state.data =
      current_arm_state;


    arm_pub_->publish(
      arm_state);
  }


  // ============================================================
  // STM32 strings
  // ============================================================

  void handle_string_message_()
  {
    if (payload_.empty()) {

      return;
    }


    const std::string message(

      reinterpret_cast<
        const char *>(
        payload_.data()),

      payload_.size());


    RCLCPP_INFO(
      verbose_logger_,
      "%s",
      message.c_str());
  }


  // ============================================================
  // Serial write
  // ============================================================

  bool write_all_(
    const uint8_t * data,
    size_t length)
  {
    try {
      serial_port_->write_all(data, length);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(this->get_logger(), "%s", error.what());
      return false;
    }
    return true;
  }


  // ============================================================
  // Create and send velocity packet
  // ============================================================

  bool send_velocity_packet_(
    float linear,
    float angular)
  {
    constexpr uint8_t payload_length =
      sizeof(float) *
      2;


    float payload[2] = {

      linear,

      angular
    };


    const auto * payload_bytes =
      reinterpret_cast<
        const uint8_t *>(
        payload);


    uint8_t checksum =
      HDR1 ^
      HDR2 ^
      CMD_VEL_ID ^
      payload_length;


    for (
      uint8_t index = 0;
      index < payload_length;
      ++index)
    {
      checksum ^=
        payload_bytes[index];
    }


    std::vector<uint8_t>
      packet = {

      HDR1,

      HDR2,

      CMD_VEL_ID,

      payload_length
    };


    packet.insert(

      packet.end(),

      payload_bytes,

      payload_bytes +
      payload_length);


    packet.push_back(
      checksum);


    if (
      !write_all_(
        packet.data(),
        packet.size()))
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Velocity reference "
        "was not sent");

      return false;
    }


    return true;
  }


  // ============================================================
  // ROS velocity callback
  //
  // IMPORTANT:
  // this callback DOES NOT send directly to STM32.
  //
  // It only:
  //   1. validates command
  //   2. saves latest reference
  //
  // Sending is done periodically by send_reference_().
  // ============================================================

  void velocity_callback_(
    const geometry_msgs::msg::TwistStamped::SharedPtr message)
  {
    const rclcpp::Time now =
      this->now();


    const rclcpp::Time stamp(

      message->header.stamp,

      this->get_clock()
      ->get_clock_type());


    // ==========================================================
    // Check age
    // ==========================================================

    const double age =
      (
        now -
        stamp
      ).seconds();


    // Too old
    if (
      age >
      cmd_timeout_s_)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Discarding stale command: "
        "%.3f s old",
        age);

      return;
    }


    // Timestamp significantly in future
    if (
      age <
      -0.1)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Discarding command from future: "
        "age %.3f s",
        age);

      return;
    }


    // ==========================================================
    // Out-of-order / duplicate
    // ==========================================================

    if (
      last_stamp_.nanoseconds()
      != 0 &&
      stamp <=
      last_stamp_)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Discarding duplicate or "
        "out-of-order command");

      return;
    }


    // ==========================================================
    // Store latest valid reference
    // ==========================================================

    latest_v_ =
      static_cast<float>(
        message->twist.linear.x);


    latest_w_ =
      static_cast<float>(
        message->twist.angular.z);


    last_stamp_ =
      stamp;
    have_cmd_ =
      true;
  }


  // ============================================================
  // Periodic reference transmission
  //
  // Called at fixed rate.
  //
  // Latest valid command wins and is kept until a new one arrives.
  // Before the first valid command, 0,0 is transmitted.
  // ============================================================

  void send_reference_()
  {
    const float v_ref =
      have_cmd_ ? latest_v_ : 0.0f;

    const float w_ref =
      have_cmd_ ? latest_w_ : 0.0f;


    // ==========================================================
    // Always send current reference to STM32
    // ==========================================================

    if (
      !send_velocity_packet_(
        v_ref,
        w_ref))
    {
      return;
    }


    // ==========================================================
    // Publish exactly what STM32 has been commanded with
    // ==========================================================

    geometry_msgs::msg::Twist
      accepted;


    accepted.linear.x =
      v_ref;


    accepted.angular.z =
      w_ref;


    accepted_pub_->publish(
      accepted);
  }


  // ============================================================
  // Generic STM32 command
  // ============================================================

  bool send_command_(
    uint8_t command)
  {
    constexpr uint8_t payload_length =
      0;


    const uint8_t checksum =
      HDR1 ^
      HDR2 ^
      payload_length ^
      command;


    const uint8_t packet[5] = {

      HDR1,

      HDR2,

      command,

      payload_length,

      checksum
    };


    try {
      serial_port_->flush_output();
    } catch (const std::exception & error) {
      RCLCPP_WARN(
        this->get_logger(),
        "Could not flush serial output: %s",
        error.what());
    }


    if (
      !write_all_(
        packet,
        sizeof(packet)))
    {
      return false;
    }


    RCLCPP_INFO(
      this->get_logger(),
      "Sent command 0x%02X "
      "with checksum 0x%02X",
      command,
      checksum);


    return true;
  }


  // ============================================================
  // Control service
  // ============================================================

  void handle_control_service_(
    const std::shared_ptr<
      robot_interfaces::srv::
      ControlCommand::Request>
    request,

    std::shared_ptr<
      robot_interfaces::srv::
      ControlCommand::Response>
    response)
  {
    if (
      request->command == 0 ||
      request->command == 1)
    {
      response->success =
        send_command_(
        static_cast<uint8_t>(
          ARM_ID));

    } else if (
      request->command == 2)
    {
      response->success =
        send_command_(
        static_cast<uint8_t>(
          RESET_ID));

    } else {

      RCLCPP_WARN(
        this->get_logger(),
        "Invalid control command: %d",
        request->command);


      response->success =
        false;
    }
  }
};


// ================================================================
namespace stm32_driver
{

std::shared_ptr<rclcpp::Node> make_stm32_driver()
{
  return std::make_shared<Stm32Driver>();
}

}  // namespace stm32_driver
