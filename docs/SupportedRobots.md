# Supported robots

URI talks to a robot through a **`RobotDriver` plugin**. Each plugin is a
standalone project in its own repository. This page lists the existing
drivers. To support a robot that is not listed, see
[Create a new robot driver](docNewRobotDriver.html).

## Drivers

| Driver | Robots | `robot_interface.driver` | SDK / transport | Commands | Repository |
|---|---|---|---|---|---|
| `rtde_driver` | Universal Robots (UR3e/UR5e/UR10e/...) | `RobotDriverRTDE` | [ur_client_library](https://github.com/UniversalRobots/Universal_Robots_Client_Library) (RTDE) | position, velocity, torque | [isri-aist/rtde_driver](https://github.com/isri-aist/rtde_driver) |
| `kortex_driver` | Kinova Gen3, Gen3 Lite | `RobotDriverKortex` | [Kortex API](https://github.com/Kinovarobotics/kortex) | position, velocity, torque | [isri-aist/kortex_driver](https://github.com/isri-aist/kortex_driver) |
| `xarm_driver` | UFACTORY xArm | `RobotDriverxArm` | xArm C++ SDK | position, velocity | [isri-aist/xarm_driver](https://github.com/isri-aist/xarm_driver) |
| `triorb_driver` | TriOrb omnidirectional mobile base | `RobotDriverTriOrb` | USB serial (`/dev/ttyACM0`) | position, velocity (planar) | [isri-aist/triorb_driver](https://github.com/isri-aist/triorb_driver) |
| `ros2_control_driver` | Any robot running a ros2_control `controller_manager` (UR, Franka, OpenArm, Gazebo/MuJoCo sims, ...) | `RobotDriverROS2Control` | ROS 2 topics | position, velocity, torque (depends on the active controllers) | [isri-aist/ros2_control_driver](https://github.com/isri-aist/ros2_control_driver) |
| `miroki_driver` | Enchanted Tools Mirokai | `RobotDriverMirokai` | ROS 2 topics (`enchanted_msgs`) | position, velocity, torque | [isri-aist/miroki_driver](https://github.com/isri-aist/miroki_driver) (private) |

"Commands" lists the `controller.mode` values the driver implements (see
[Robot manager](docRobotManager.html)). Asking for a mode the driver does not
support makes the driver throw or ignore the command.

To run a robot in simulation, see [Simulation](docSimulation.html).

## Messages

Every driver uses the same two messages, whatever the robot. `uri interface`
fills a `robot_comm::State` from the driver's getters each cycle. It passes
the `robot_comm::Command` it receives to `servoJ`, `speedJ` or `tauJ`,
according to `controller.mode`. The full definitions are in
[Communication](docCommunication.html).

| `State` field | Filled from | Drivers that provide it |
|---|---|---|
| `position` [rad] | `getActualQ()` | all |
| `velocity` [rad/s] | `getActualQd()` | all |
| `torque` [Nm] | `getJointTorques()` | all (`rtde_driver` reports the *target* torque) |
| `bodySensors` (IMU: orientation, angular velocity, linear acceleration) | `getIMUs()` | `miroki_driver` |
| `forceSensors` (F/T: force, torque, in sensor frame) | `getForceSensors()` | none yet |

| `Command` field | Sent to | Used when `controller.mode` is |
|---|---|---|
| `position` [rad] | `servoJ()` | `position` |
| `velocity` [rad/s] | `speedJ()` | `velocity` |
| `torque` [Nm] | `tauJ()` | `torque` |

Sensor data is matched by name to the RobotModule's body sensors and force
sensors. A driver that doesn't override `getIMUs()` or `getForceSensors()`
sends empty lists.

## Using a driver

Every driver is built and installed the same way. Use the **same install
prefix as URI**, because `uri interface` looks for plugins in
`<prefix>/lib/robot_interface`. To keep a driver elsewhere, list its directory in
the `URI_DRIVER_PATH` environment variable (see
[Robot interface](docRobotInterface.html#plugin-lookup)):

```bash
cmake -S . -B build \
  -DCMAKE_PREFIX_PATH=/path/to/install \
  -DCMAKE_INSTALL_PREFIX=/path/to/install
cmake --build build --parallel
cmake --install build
```

Then point a robot entry of `mc_rtc.yaml` at the driver's class name:

```yaml
Robots:
  ur5e:
    module: UR5e
    robot_interface:
      driver: RobotDriverRTDE
      ip: 192.168.1.100
```

Use `uri viewer -c <config>` to check that the plugin loads and reports joint
state before starting `uri manager`.

## Driver notes

### rtde_driver

- Runs the UR controller in headless mode: it sends the URScript program
  itself, so no URCap is needed. Set `UR_SCRIPT_FILE` if
  `external_control.urscript` is not found automatically.
- `sync()` blocks on each RTDE package, so the loop runs at the robot's RTDE
  rate (500 Hz by default).

### kortex_driver

- The Kortex API is downloaded by CMake if it is not found. Pass
  `-DKORTEX_ROOT_DIR=/opt/kortex` to use a local copy.
- Uses the robot's default username and password, which are hardcoded for now.

### xarm_driver

- Joint position control only (`set_mode(1)`, servo mode). `speedJ` and
  `tauJ` throw.

### triorb_driver

- `ip` is the serial device, e.g. `/dev/ttyACM0`.
- State is the planar pose `[x, y, theta]`. Commands are `[vx, vy, wz]` in
  velocity mode. In position mode, the driver turns the pose error into
  velocity commands; it does not use TriOrb's native position commands.
- Resets the odometry origin at startup. Has a watchdog and sends zero
  velocity on shutdown.

### ros2_control_driver

- A plain ROS 2 node, not a hardware plugin. It reads `/joint_states` and
  publishes to `forward_command_controller` topics. `controller_manager` must
  already be running with the right controllers active.
- Takes a YAML file through `robot_interface.config_path` that names the
  topics; `ip` and `port` are ignored. Supporting a new ros2_control robot
  only needs a new config file.
- Requires a sourced ROS 2 workspace at build and run time.

### miroki_driver

- Talks to Mirokai's own onboard controller through its ROS 2 bridge, one
  topic pair per body part. Needs the `enchanted_msgs` package from the
  Mirokai SDK.
- The robot must be in User Control Mode with joints enabled, otherwise
  commands are silently ignored.
- Takes the joint-group mapping (`etc/miroki.yaml`) through
  `robot_interface.config_path`.

## Compatibility

`uri interface` only loads plugins built against the current driver ABI
(`ROBOT_DRIVER_ABI_VERSION` in `RobotDriverTemplate.h`, currently 3). A
driver must use the current `create()` signature and call
`ROBOT_DRIVER_EXPORT_ABI_VERSION()`. Otherwise it is refused with
"built against an incompatible interface".

As of this writing, only `rtde_driver` and `ros2_control_driver` do this.
`kortex_driver`, `xarm_driver` and `triorb_driver` still use the older
`create(name, ip, port)` signature. `miroki_driver` has the new signature but
does not export the ABI version. These drivers need that update before they
load with the current `uri`.
