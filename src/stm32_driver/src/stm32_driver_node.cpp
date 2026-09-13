#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "parsing_types.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/wheel_speeds.hpp"
#include "robot_interfaces/srv/control_command.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "std_msgs/msg/bool.hpp"

#include <boost/circular_buffer.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <system_error>
#include <termios.h>
#include <unistd.h>
#include <vector>


class Stm32Driver : public rclcpp::Node
{
public:
  Stm32Driver()
  : Node("stm32_driver"),
    verbose_logger_(this->get_logger().get_child("verbose_logger")),
    last_stamp_(0, 0, RCL_ROS_TIME),
    last_enc_(0, 0, RCL_ROS_TIME),
    last_cmd_rx_(0, 0, RCL_ROS_TIME)
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

    validate_parameters_();


    // ============================================================
    // Serial
    // ============================================================

    try {
      open_serial_();

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
      rclcpp::QoS(
        rclcpp::KeepLast(1))
      .best_effort(),

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
    if (serial_fd_ >= 0) {

      close(serial_fd_);

      RCLCPP_INFO(
        this->get_logger(),
        "Serial port closed");
    }
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

  int serial_fd_{-1};

  pollfd serial_poll_{};

  uint8_t read_buffer_[128]{};

  boost::circular_buffer<uint8_t>
  receive_buffer_{512};

  double radius_{0.0346};

  double wheel_base_{0.2};

  double cmd_timeout_s_{0.5};


  // ============================================================
  // Parser
  // ============================================================

  ParseState parse_state_{
    ParseState::WAIT_HEADER1};

  uint8_t message_type_{0};

  uint8_t message_length_{0};

  uint8_t checksum_{0};

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

  // Prevent repeated watchdog warning
  bool watchdog_active_{false};


  // ============================================================
  // Time state
  // ============================================================

  // Stamp generated upstream with TwistStamped
  rclcpp::Time last_stamp_;

  // Local time of previous encoder message
  rclcpp::Time last_enc_;

  // Local arrival time of latest VALID cmd_vel
  rclcpp::Time last_cmd_rx_;


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


    if (cmd_timeout_s_ <= 0.0) {

      throw std::invalid_argument(
              "cmd_timeout_s must be greater than zero");
    }
  }


  // ============================================================
  // Baudrate
  // ============================================================

  static speed_t termios_baud_(
    int baudrate)
  {
    switch (baudrate) {

      case 9600:
        return B9600;

      case 19200:
        return B19200;

      case 38400:
        return B38400;

      case 57600:
        return B57600;

      case 115200:
        return B115200;

      case 230400:
        return B230400;

      default:

        throw std::invalid_argument(
                "Unsupported baudrate: " +
                std::to_string(
                  baudrate));
    }
  }


  // ============================================================
  // Open serial
  // ============================================================

  void open_serial_()
  {
    serial_fd_ =
      open(
      port_.c_str(),
      O_RDWR |
      O_NOCTTY |
      O_NONBLOCK);


    if (serial_fd_ < 0) {

      throw std::system_error(
              errno,
              std::generic_category(),
              "Failed to open " +
              port_);
    }


    try {

      termios tty{};


      if (tcgetattr(
          serial_fd_,
          &tty) != 0)
      {
        throw std::system_error(
                errno,
                std::generic_category(),
                "tcgetattr failed");
      }


      const speed_t speed =
        termios_baud_(
        baudrate_);


      if (
        cfsetispeed(
          &tty,
          speed) != 0 ||

        cfsetospeed(
          &tty,
          speed) != 0)
      {
        throw std::system_error(
                errno,
                std::generic_category(),
                "Setting baudrate failed");
      }


      tty.c_cflag |=
        CLOCAL | CREAD;

      tty.c_cflag &=
        ~CSIZE;

      tty.c_cflag |=
        CS8;

      tty.c_cflag &=
        ~(PARENB |
        CSTOPB |
        CRTSCTS);


      tty.c_lflag = 0;

      tty.c_iflag = 0;

      tty.c_oflag = 0;

      tty.c_cc[VMIN] = 0;

      tty.c_cc[VTIME] = 0;


      if (
        tcsetattr(
          serial_fd_,
          TCSANOW,
          &tty) != 0)
      {
        throw std::system_error(
                errno,
                std::generic_category(),
                "tcsetattr failed");
      }


      if (
        tcflush(
          serial_fd_,
          TCIOFLUSH) != 0)
      {
        throw std::system_error(
                errno,
                std::generic_category(),
                "tcflush failed");
      }

    } catch (...) {

      close(
        serial_fd_);

      serial_fd_ = -1;

      throw;
    }


    serial_poll_.fd =
      serial_fd_;

    serial_poll_.events =
      POLLIN;
  }


  // ============================================================
  // Serial receive
  // ============================================================

  void read_serial_()
  {
    int poll_result = 0;


    while (
      (poll_result =
      poll(
        &serial_poll_,
        1,
        0)) > 0)
    {
      if (
        (serial_poll_.revents &
        POLLIN) == 0)
      {
        break;
      }


      const ssize_t bytes_read =
        read(
        serial_fd_,
        read_buffer_,
        sizeof(
          read_buffer_));


      if (bytes_read > 0) {

        for (
          ssize_t index = 0;
          index < bytes_read;
          ++index)
        {
          receive_buffer_.push_back(
            read_buffer_[index]);
        }

      } else if (
        bytes_read < 0 &&
        errno != EAGAIN &&
        errno != EWOULDBLOCK &&
        errno != EINTR)
      {
        RCLCPP_ERROR(
          this->get_logger(),
          "Serial read failed: %s",
          std::strerror(
            errno));

        break;
      }
    }


    if (
      poll_result < 0 &&
      errno != EINTR)
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Serial poll failed: %s",
        std::strerror(
          errno));
    }


    parse_();
  }


  // ============================================================
  // Parser
  // ============================================================

  void parse_()
  {
    while (
      !receive_buffer_.empty())
    {
      const uint8_t byte =
        receive_buffer_.front();


      receive_buffer_.pop_front();


      switch (parse_state_) {

        case ParseState::WAIT_HEADER1:

          if (byte == HDR1) {

            checksum_ =
              byte;

            parse_state_ =
              ParseState::WAIT_HEADER2;
          }

          break;


        case ParseState::WAIT_HEADER2:

          if (byte == HDR2) {

            checksum_ ^=
              byte;

            parse_state_ =
              ParseState::WAIT_TYPE;

          } else {

            parse_state_ =
              ParseState::WAIT_HEADER1;
          }

          break;


        case ParseState::WAIT_TYPE:

          message_type_ =
            byte;

          checksum_ ^=
            byte;

          parse_state_ =
            ParseState::WAIT_LEN;

          break;


        case ParseState::WAIT_LEN:

          message_length_ =
            byte;

          payload_.clear();

          checksum_ ^=
            byte;


          parse_state_ =
            message_length_ == 0
            ?
            ParseState::WAIT_CHECKSUM
            :
            ParseState::WAIT_PAYLOAD;

          break;


        case ParseState::WAIT_PAYLOAD:

          payload_.push_back(
            byte);

          checksum_ ^=
            byte;


          if (
            payload_.size() >=
            message_length_)
          {
            parse_state_ =
              ParseState::WAIT_CHECKSUM;
          }

          break;


        case ParseState::WAIT_CHECKSUM:

          if (checksum_ == byte) {

            handle_message_();

          } else {

            RCLCPP_WARN(
              this->get_logger(),
              "Checksum mismatch for type "
              "0x%02X: calculated 0x%02X, "
              "received 0x%02X",
              message_type_,
              checksum_,
              byte);
          }


          parse_state_ =
            ParseState::WAIT_HEADER1;

          break;
      }
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
      "imu_link";


    tf2::Quaternion orientation;


    orientation.setRPY(
      data[0],
      data[1],
      data[2]);


    imu.orientation.x =
      orientation.x();

    imu.orientation.y =
      orientation.y();

    imu.orientation.z =
      orientation.z();

    imu.orientation.w =
      orientation.w();


    imu.linear_acceleration.x =
      data[3];

    imu.linear_acceleration.y =
      data[4];

    imu.linear_acceleration.z =
      data[5];


    imu.angular_velocity.x =
      data[6];

    imu.angular_velocity.y =
      data[7];

    imu.angular_velocity.z =
      data[8];


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


    imu_pub_->publish(
      imu);


    sensor_msgs::msg::MagneticField
      magnetic_field;


    magnetic_field.header.stamp =
      now;

    magnetic_field.header.frame_id =
      "mag_link";


    magnetic_field.magnetic_field.x =
      data[9];

    magnetic_field.magnetic_field.y =
      data[10];

    magnetic_field.magnetic_field.z =
      data[11];


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


    if (
      last_enc_.nanoseconds()
      != 0)
    {
      const double dt =
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
    if (serial_fd_ < 0) {

      RCLCPP_ERROR(
        this->get_logger(),
        "Cannot write because "
        "the serial port is closed");

      return false;
    }


    size_t offset =
      0;


    while (
      offset <
      length)
    {
      const ssize_t written =
        write(
        serial_fd_,
        data + offset,
        length - offset);


      if (written > 0) {

        offset +=
          static_cast<size_t>(
            written);

        continue;
      }


      if (
        written < 0 &&
        errno == EINTR)
      {
        continue;
      }


      if (
        written < 0 &&
        (
          errno == EAGAIN ||
          errno == EWOULDBLOCK
        ))
      {
        pollfd write_poll{

          serial_fd_,

          POLLOUT,

          0
        };


        const int poll_result =
          poll(
          &write_poll,
          1,
          100);


        if (poll_result > 0) {

          continue;
        }


        RCLCPP_ERROR(
          this->get_logger(),
          "Timed out while writing "
          "to the serial port");


        return false;
      }


      RCLCPP_ERROR(
        this->get_logger(),
        "Serial write failed: %s",
        std::strerror(
          errno));


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


    // Local reception timestamp.
    //
    // Watchdog uses LOCAL time,
    // not sender timestamp.
    last_cmd_rx_ =
      now;


    have_cmd_ =
      true;


    // New fresh command:
    // exit watchdog state.
    watchdog_active_ =
      false;
  }


  // ============================================================
  // Periodic reference transmission
  //
  // Called at fixed rate.
  //
  // Default:
  // 20 ms -> 50 Hz
  //
  // This is the actual reference stream seen by STM32.
  // ============================================================

  void send_reference_()
  {
    const rclcpp::Time now =
      this->now();


    float v_ref =
      0.0f;


    float w_ref =
      0.0f;


    // ==========================================================
    // Decide whether latest command is still valid
    // ==========================================================

    if (
      have_cmd_ &&
      last_cmd_rx_.nanoseconds()
      != 0)
    {
      const double elapsed =
        (
          now -
          last_cmd_rx_
        ).seconds();


      if (
        elapsed <=
        cmd_timeout_s_)
      {
        // ================================================
        // Fresh reference
        // ================================================

        v_ref =
          latest_v_;


        w_ref =
          latest_w_;

      } else {

        // ================================================
        // Watchdog
        // ================================================

        v_ref =
          0.0f;


        w_ref =
          0.0f;


        have_cmd_ =
          false;


        if (
          !watchdog_active_)
        {
          RCLCPP_WARN(
            this->get_logger(),
            "cmd_vel timeout after %.3f s "
            "-> forcing STOP",
            elapsed);


          watchdog_active_ =
            true;
        }
      }
    }


    // ==========================================================
    // Always send reference to STM32
    //
    // Fresh command:
    //     latest v,w
    //
    // Timeout / startup:
    //     0,0
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


    if (
      tcflush(
        serial_fd_,
        TCOFLUSH) != 0)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Could not flush serial output: %s",
        std::strerror(
          errno));
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
// Main
// ================================================================

int main(
  int argc,
  char ** argv)
{
  rclcpp::init(
    argc,
    argv);


  try {

    rclcpp::spin(
      std::make_shared<
        Stm32Driver>());

  } catch (
    const std::exception & error)
  {
    if (rclcpp::ok()) {

      RCLCPP_FATAL(
        rclcpp::get_logger(
          "stm32_driver"),
        "Driver startup failed: %s",
        error.what());
    }


    rclcpp::shutdown();

    return 1;
  }


  rclcpp::shutdown();

  return 0;
}