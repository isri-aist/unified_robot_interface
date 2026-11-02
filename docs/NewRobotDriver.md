# Create a new robot driver

Check [Supported robots](docSupportedRobots.html) first: your robot may
already have a driver. A simulator is written the same way, see
[Simulation](docSimulation.html).

Supporting a new robot in URI means writing a **`RobotDriver` plugin**: a
small shared library that wraps the robot's SDK behind about eight methods.
The rest is already generic: networking, the init handshake, the control loop
and the mc_rtc integration.

A driver:

- is a **standalone CMake project**, living in its own repository,
- depends only on the header-only `unified_robot_interface::robot_driver_api`
  target (**no mc_rtc**),
- is installed into `<prefix>/lib/robot_interface/`, where `uri interface`
  finds it by class name.

Overview of the steps:

1. Scaffold the project with `uri create_new_driver`.
2. Implement the TODOs: connect, `sync()`, getters and commands.
3. Build and install with the same prefix as URI.
4. Test it on its own with `uri viewer`.
5. Reference it from `mc_rtc.yaml` and run `uri`.

## 1. Generate the project

`uri create_new_driver` runs the generator script
`robot_interface/tools/create_new_driver.sh`, which is embedded in `uri` at build time:

```bash
uri create_new_driver Franka ~/devel      # DriverName in PascalCase, optional output dir
```

```text
Created new driver project: /home/me/devel/franka_driver

  franka_driver/
  ├── CMakeLists.txt
  ├── src/
  │   ├── CMakeLists.txt
  │   └── RobotDriverFranka.cpp
  ├── include/franka_driver/
  │   └── RobotDriverFranka.h
  ├── etc/
  │   ├── robot_interface.yaml
  │   └── franka_robot.yaml
  ├── README.md
  └── .gitignore
```

Naming, derived from the argument:

| Item | Value for `Franka` |
|---|---|
| Project / directory | `franka_driver` |
| Class, plugin name and `robot_interface.driver` value | `RobotDriverFranka` |
| Namespace | `franka_driver` |
| Installed library | `lib/robot_interface/libRobotDriverFranka.so` |

The script also runs `git init`; pass `--no-git` to skip it. Run
`uri create_new_driver --help` for all options.

## 2. Implement the driver

The generated project already compiles. It is a skeleton that returns empty
state and ignores commands. Fill in `src/RobotDriverFranka.cpp`:

```cpp
RobotDriverFranka::RobotDriverFranka(const std::string & ip, uint16_t port, const std::string & config_path)
: ip_(ip)
{
  // Open the connection. Throw on failure: the error is sent back to uri
  // as "ERROR: Driver load failed: <what()>" and this robot is skipped.
  robot_ = std::make_unique<franka::Robot>(ip_);
}

void RobotDriverFranka::sync()
{
  // Block until the robot publishes a new state. This call sets the
  // control-loop rate, so never replace it with a sleep().
  state_ = robot_->readOnce();
}

std::vector<double> RobotDriverFranka::getActualQ()
{
  // Radians, in the SAME ORDER as the mc_rtc RobotModule's ref_joint_order.
  return {state_.q.begin(), state_.q.end()};
}

std::vector<double> RobotDriverFranka::getJointTorques()
{
  return {state_.tau_J.begin(), state_.tau_J.end()};
}

void RobotDriverFranka::servoJ(const std::vector<double> & q)
{
  // Called every cycle in position mode, with the latest mc_rtc command
  // (repeated when mc_rtc runs slower than the robot).
  sendJointPositions(q);
}
```

The code above is illustrative. Use your robot's actual SDK calls.

Rules to keep in mind:

- **Joint order**: every vector, in and out, follows the RobotModule's
  reference joint order. Reorder in the driver if the SDK uses another order.
- **Units**: rad, rad/s, Nm, N; quaternions are `(w, x, y, z)`.
- **Empty `getActualQ()`** means "no state yet". Nothing is published that
  cycle, and mc_rtc waits before initializing the robot.
- **Command modes**: implement at least the method for the `controller.mode`
  you will use. The others can throw or be no-ops.
- **Sensors**: override `getIMUs()` / `getForceSensors()` when the robot has
  them. The keys must match the RobotModule's body sensor and force sensor
  names, e.g. `"EEForceSensor"`.

### Constructor arguments and `config_path`

`create()` receives values from the robot's `robot_interface` entry in
`mc_rtc.yaml`:

| `create()` argument | From | Typical use |
|---|---|---|
| `ip`, `port` | `robot_interface.ip` / `.port` (both required by the manager) | Robot address |
| `config_path` | `robot_interface.config_path` (optional) | Path to a driver-specific YAML file, for anything richer than ip/port: topic names, joint mappings, gains... |
| `grippers` | the RobotModule's grippers, added by the manager | Know which joints belong to a gripper, e.g. to drive them through a separate API |

The generated `create()` forwards `ip`, `port` and `config_path` to the
constructor and ignores `grippers`. Extend it if you need grippers. **Never
change `create()`'s signature**: every driver is called through the same
function-pointer type, and a mismatch is undefined behavior, not a compile
error.

### SDK dependencies

Add them in the two CMake files. The generated `add_robot_driver()` call sets
the RUNPATH of the installed `.so`, so its dependencies resolve without
`LD_LIBRARY_PATH`:

```cmake
# CMakeLists.txt
find_package(Franka REQUIRED)

# src/CMakeLists.txt
target_link_libraries(RobotDriverFranka PRIVATE Franka::Franka)
```

If the driver uses ROS 2 (`rclcpp`), also:

- export `extern "C" void LOAD_GLOBAL() {}` from the library, and
- create any `rclcpp` executor or node as a `std::unique_ptr` **in the
  constructor body, after `rclcpp::init()`**, not as a plain member.

Otherwise `rclcpp` fails with `failed to create guard condition: context
argument is null`.

## 3. Build and install

Use the **same install prefix as URI**. `uri interface` looks in the
`lib/robot_interface` directory under the prefix it was built with (and in the
directories of `URI_DRIVER_PATH`, see below):

```bash
cd franka_driver
cmake -S . -B build \
  -DCMAKE_PREFIX_PATH=/path/to/uri/install \
  -DCMAKE_INSTALL_PREFIX=/path/to/uri/install
cmake --build build
cmake --install build
# -- Installing: /path/to/uri/install/lib/robot_interface/libRobotDriverFranka.so
```

Check that the four plugin symbols are exported:

```bash
nm -D --defined-only /path/to/uri/install/lib/robot_interface/libRobotDriverFranka.so \
  | grep -E ' (ROBOT_DRIVER_PLUGIN|create|destroy|robot_driver_abi_version)$'
```

The generated source already contains `ROBOT_DRIVER_EXPORT_ABI_VERSION()`,
which exports `robot_driver_abi_version`. Keep it: without it, the driver is
refused at load time. Rebuild and reinstall the driver whenever URI is updated,
and it fails to load with "built against an incompatible interface". See
[Robot interface, ABI version](docRobotInterface.html).

## 4. Test the driver on its own

Edit `etc/robot_interface.yaml` with the robot's address, then:

```bash
uri viewer -c etc/robot_interface.yaml
```

This loads the plugin exactly as `uri interface` does, then displays the
positions, velocities and torques at 10 Hz. Move the robot by hand, or in
simulation, and check that joint order, signs and units are right before
connecting mc_rtc. The viewer never sends commands.

If you get `Plugin 'RobotDriverFranka' not found`, the log lists the searched
directories. Either your install prefix doesn't match URI's, or the driver's
directory is not in `URI_DRIVER_PATH`. If you get `built against
an incompatible interface`, rebuild and reinstall the driver against the
current URI headers.

To test a driver without installing it into URI's prefix, put its directory
in `URI_DRIVER_PATH` (colon-separated, searched before the prefix):

```bash
URI_DRIVER_PATH=$PWD/build uri viewer -c etc/robot_interface.yaml
```

Below the joint table, the viewer also shows how long `sync()` blocks and the
cycle period, so you can check that the driver runs at the robot's rate.

## 5. Use it from mc_rtc

Add the robot to the manager's `mc_rtc.yaml`:

```yaml
MainRobot: Panda
Enabled: Posture
Timestep: 0.005

Robots:
  panda:
    module: Panda                   # the mc_rtc RobotModule for this robot
    controller:
      mode: position
      time_step: 0.001              # the robot's native control period
    network_interface:
      protocol: zenoh
    robot_interface:
      driver: RobotDriverFranka     # the class name from step 1
      ip: 172.16.0.2
      port: 0
      # config_path: /path/to/franka_driver/etc/franka.yaml
      autostart: true               # robot reachable from this PC
```

```bash
uri manager -c mc_rtc.yaml
```

Expected log:

```text
[RobotInterface] 'panda' loading driver 'RobotDriverFranka' (172.16.0.2:0), config_path: '', 0 gripper(s)
[RobotInterface] 'panda' driver 'RobotDriverFranka' loaded
[RobotManager] mc_rtc running at 200Hz, robot running at 1000Hz
[FMInterfaceTemplate] 'panda' robot initialized
```

For a robot on its own PC, drop `autostart`. Install the driver on that PC
and start `uri interface` there instead. See
[Getting started, example 2](docGettingStarted.html).

## Checklist

- `getActualQ()` returns values in the RobotModule's joint order and
  units, verified with `uri viewer`
- `sync()` blocks at the robot's native rate
- The command method for the chosen `controller.mode` is implemented
- `controller.time_step` matches the robot's rate, and mc_rtc's
  `Timestep` is a multiple of it
- The driver is installed in URI's `<prefix>/lib/robot_interface`
- ROS 2 drivers export `LOAD_GLOBAL`
- The driver source keeps `ROBOT_DRIVER_EXPORT_ABI_VERSION()`

---

[![CNRS-AIST JRL (Joint Robotics Laboratory)](images/jrl-logo.png)](https://unit.aist.go.jp/isri/isri-jrl/en/)[![CNRS](images/cnrs-logo.png)](https://www.cnrs.fr/)[![AIST](images/aist-logo.png)](https://www.aist.go.jp/index_en.html)

Copyright © CNRS-AIST JRL 2026
