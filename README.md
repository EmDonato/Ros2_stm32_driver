# STM32 ROS 2 driver

This repository contains the ROS 2 Jazzy serial driver for the STM32 Nucleo
F303RE robot controller. The container builds the driver and the shared
`robot_interfaces` package from source, then starts the driver automatically.

## Repository layout

- `src/stm32_driver`: ROS 2 package and executable. The implementation is
  separated into the ROS node, serial transport, packet parser, odometry and
  process entry point.
- `config/params.yaml`: default runtime parameters.
- `Dockerfile`: single-stage source build.
- `compose.yaml`: Raspberry Pi 5 runtime example.

The intended parent repository layout is:

```text
main_folder/
|-- drivers/
|   `-- stm32_driver/
|-- interfaces/
|   `-- src/robot_interfaces/
`-- modules/
```

Compose passes `../../interfaces` to Docker as the named `interfaces` build
context. Override `INTERFACES_CONTEXT` when the repositories are stored in a
different layout.

## Dependencies and device

The ROS package depends on `rclcpp`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`,
`std_msgs`, `tf2`, `tf2_ros`, and the sibling `robot_interfaces` package. Docker
obtains `robot_interfaces` only from the named `interfaces` build context; it
is not vendored in this repository.

Runtime requires the STM32 serial device, `/dev/ttyACM0` by default.

## Build and run

Run from this repository:

```bash
docker compose build
docker compose up
```

The default platform is `linux/arm64`. Use the native PC architecture when
developing on an x86-64 machine:

```bash
ROBOT_PLATFORM=linux/amd64 docker compose build
```

Useful overrides:

```bash
ROS_DOMAIN_ID=7 HOST_UID=1000 HOST_GID=1000 docker compose up
INTERFACES_CONTEXT=/absolute/path/to/interfaces docker compose build
```

The service runs as a non-root `ros` user. The user belongs to the `dialout`,
`video`, and `plugdev` groups, while Compose grants access to host devices.

## Parameters

Defaults are stored in `config/params.yaml` and mounted read-only at runtime.

| Parameter | Default | Description |
| --- | --- | --- |
| `port` | `/dev/ttyACM0` | STM32 serial device. |
| `baudrate` | `115200` | Serial baudrate. Supported values are 9600 through 230400. |
| `frequency_ms` | `5` | Serial polling period in milliseconds. |
| `radius` | `0.0346` | Wheel radius in metres. |
| `wheel_base` | `0.2` | Distance between wheels in metres. |
| `base_frame_id` | `base_link` | Encoder twist and odometry child frame. |
| `imu_parent_frame_id` | `chassis` | Parent of the static IMU mounting transform. |
| `imu_frame_id` | `imu_link` | IMU and magnetometer sensor frame. |
| `encoder_odom_frame_id` | `enc_odom` | Raw encoder odometry frame. |
| `publish_imu_tf` | `true` | Publish static `chassis -> imu_link`. |
| `imu_transform.{x,y,z,roll,pitch,yaw}` | `0.0` | Physical IMU mounting pose in metres and radians. |
| `control_service` | `control/command` | Control service name. |

## ROS interfaces

The node name is `/stm32_driver`.

Published topics:

- `imu` (`sensor_msgs/msg/Imu`)
- `magnetometer` (`sensor_msgs/msg/MagneticField`)
- `enc/twist_meas` (`geometry_msgs/msg/TwistStamped`)
- `odom/solo` (`nav_msgs/msg/Odometry`), encoder-only pose; no TF.
- `debug/imu` (`robot_interfaces/msg/ImuEuler`), corrected IMU Euler angles.
- `enc/twist_wheels` (`robot_interfaces/msg/WheelSpeeds`)
- `/joint_states` (`sensor_msgs/msg/JointState`)
- `robot_status/is_armed` (`std_msgs/msg/Bool`)

The driver subscribes to `/cmd_vel` and provides the
`robot_interfaces/srv/ControlCommand` service at `control/command` by default.
IMU and magnetometer messages use the `imu_link` frame. Encoder twist uses
`base_link`, while raw encoder odometry uses `enc_odom` as its parent. Joint
names remain `left_wheel_joint` and
`right_wheel_joint`.

## TF and diagnostics

Only the physical IMU mounting transform is published by the driver:
`chassis -> imu_link`. The URDF owns `base_link -> chassis`, and the EKF owns
`odom -> base_link`. Ghost robots, diagnostic world alignments, encoder TF and
gyro-integrated attitude TF have been removed.

`odom/solo` replaces `enc/odom`. It contains encoder-only odometry, starts
at zero when the driver starts, uses `enc_odom` as its reference frame and
`base_link` as its child. It publishes no TF and does not compete with EKF
odometry. The EKF still receives `enc/twist_meas`; its input is unchanged.

`debug/imu` contains `roll_rad`, `pitch_rad`, `yaw_rad`, `roll_deg`,
`pitch_deg`, `yaw_deg`, and the same header as `imu`. Angles are derived
from the corrected quaternion published in `imu`, including the configured
raw-axis correction. They are diagnostic values even when
`imu_orientation_variance: -1.0` disables orientation fusion. Euler angles
wrap at +/-pi (180 degrees), and pitch has a singularity at +/-90 degrees.

```bash
ros2 topic echo /debug/imu
ros2 topic echo /odom/solo
ros2 run tf2_ros tf2_echo odom base_link
```

## Migration

The former package and executable names were:

```text
stm32_nucleo_f303re_driver/stm_driver
```

Use the following names now:

```text
stm32_driver/stm32_driver_node
```

Update external launch files and package dependencies accordingly. The shared
message and service definitions in `robot_interfaces` are unchanged.

## Troubleshooting

- Confirm that the controller exists with `ls -l /dev/ttyACM0`.
- If the container cannot open the port, verify that the host user and the
  container user have access to the device's group.
- Set a different `port` in `config/params.yaml` when udev assigns another
  device path.
- The driver exits with an error when the serial port cannot be opened or when
  an unsupported baudrate is configured.

### IMU units and axes

The output frame uses X forward, Y left, Z up. With `imu_flip_x: true`,
raw vectors are mapped as (x, -y, -z), including gyro and magnetometer.
The quaternion is right-multiplied by the inverse mounting rotation.
This assumes raw X forward, Y right, Z down, and firmware RPY in radians
describing sensor-to-world orientation. A negative stationary Z and roll near
pi are consistent with this mounting but do not prove the horizontal axes or
world convention. Firmware is not included here; verify on hardware.

`imu_acceleration_scale: 9.80665` converts raw g to m/s²; set 1.0 for
firmware already producing SI acceleration. Existing acceleration covariance
is treated as raw-unit squared and scaled by the square of this factor;
covariances are also rotated. Gyro input is assumed rad/s.
The inherited covariance constants still require sensor calibration.

`imu_orientation_variance: -1.0` marks orientation unavailable to consumers
until its convention and uncertainty are validated; the corrected quaternion
remains in the message for inspection. Set a measured positive isotropic
variance in rad² to enable orientation for fusion. Zero covariance means
unknown uncertainty, not perfect accuracy (see the
[ROS Imu contract](https://raw.githubusercontent.com/ros/common_msgs/noetic-devel/sensor_msgs/msg/Imu.msg)).

Keep static `imu_transform.roll/pitch/yaw` zero for a robot-aligned
`imu_link`; do not apply the raw-axis correction a second time in TF.
Set the translation to the measured sensor position.

Hardware acceptance:
- Level and stationary: acceleration approximately (0, 0, +9.80665) m/s²,
  gyro near zero, roll/pitch near zero; yaw depends on the firmware reference.
- Accelerate forward, then left: the changes in acceleration must be +X, +Y.
- Turn left about upward Z: angular velocity Z must be positive.
- Tilt about each positive axis using the right-hand rule and check quaternion
  and gyro consistency before enabling orientation fusion.
