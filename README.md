# STM32 ROS 2 driver

This repository contains the ROS 2 Humble serial driver for the STM32 Nucleo
F303RE robot controller. The container builds the driver and the shared
`robot_interfaces` package from source, then starts the driver automatically.

## Repository layout

- `src/stm32_driver`: ROS 2 package and executable.
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

The ROS package depends on `rclcpp`, `geometry_msgs`, `sensor_msgs`,
`std_msgs`, `tf2`, Boost, and the sibling `robot_interfaces` package. Docker
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
| `control_service` | `control/command` | Control service name. |

## ROS interfaces

The node name is `/stm32_driver`.

Published topics:

- `imu` (`sensor_msgs/msg/Imu`)
- `magnetometer` (`sensor_msgs/msg/MagneticField`)
- `enc/twist_meas` (`geometry_msgs/msg/TwistStamped`)
- `enc/twist_wheels` (`robot_interfaces/msg/WheelSpeeds`)
- `/joint_states` (`sensor_msgs/msg/JointState`)
- `robot_status/is_armed` (`std_msgs/msg/Bool`)

The driver subscribes to `/cmd_vel` and provides the
`robot_interfaces/srv/ControlCommand` service at `control/command` by default.
IMU messages use the `imu_link` frame and magnetometer messages use the
`mag_link` frame. Joint names remain `left_wheel_joint` and
`right_wheel_joint`.

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
