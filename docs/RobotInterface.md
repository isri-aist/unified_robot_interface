# Robot interface (`uri interface`)

`uri interface` is the generic robot-side process (implemented by the
`robot_interface::RobotInterface` class). There is one per robot.
It contains no robot-specific code: at runtime it loads the `RobotDriver`
plugin that the manager asks for, then shuttles data between that driver and
`robot_comm`.

It runs either:

- **on a robot-side PC**, started by hand or by a service **before**
  `uri manager` (the manager skips a robot whose init query gets no answer), or
- **on the control PC**, spawned automatically by `uri manager` when the robot has
  `robot_interface.autostart: true`.

## Command line

All `uri` commands are listed in [Command line](docCommandLine.html).

```bash
uri interface -c <config.yaml> [-n <name>]
```

It shows up as `uri:<name>` in `ps`/`top` (truncated to 15 characters).

| Option | Meaning |
|---|---|
| `-c, --config` | YAML file, **required** |
| `-n, --name` | Robot name; overrides `name:` in the file. Must match the key under `Robots:` in the manager's `mc_rtc.yaml`. |

The config file only needs what it takes to reach the manager. Everything
else, including the driver, IP, port and control mode, arrives in the init
query:

```yaml
name: ur5e
network_interface:
  protocol: zenoh
  # configuration: /path/to/zenoh.json5   # if multicast discovery is unavailable
```

## Lifecycle

`RobotInterface::run()` does the following:

1. Connects with a `robot_comm` **client** (`setupClient()`): it publishes
   `{name}/state` and subscribes to `{name}/command`.
2. Registers a queryable on `{name}/init` and logs
   `'<name>' waiting for init query on '<name>/init'`.
3. When the init query arrives (`handleInitQuery`):
   - It parses the YAML and reads `controller.mode`, which defaults to
     `position`.
   - It reads `robot_interface.driver`, `ip` (default `127.0.0.1`), `port`
     (default `0`), `config_path` (default `""`) and the `grippers` map that
     the manager added from the RobotModule.
   - It loads the driver through `PluginLoader<RobotDriver>` and replies `OK`.
     Any failure is replied as `ERROR: <reason>` and the process exits.
   - The handler is idempotent. A repeated query, from the manager's retries,
     just replies `OK` again without reloading the driver.
4. **Hold command**: before the loop starts, it primes a "last command" so the
   robot receives a valid command from the very first cycle, even before
   mc_rtc has produced one:
   - `position`: the current `getActualQ()`, so the robot holds its pose
   - `velocity` / `torque`: zeros
5. **Control loop**, until `SIGINT`/`SIGTERM`:

   ```cpp
   while(!interrupt)
   {
     driver_->sync();     // blocks until the robot has a new state: this paces the loop
     updateSensors();     // getActualQ/Qd, getJointTorques, getIMUs, getForceSensors -> publish State
     updateControl();     // latest Command (or previous one) -> servoJ / speedJ / tauJ
   }
   ```

   There is no timer in `RobotInterface`. The loop runs at whatever rate the
   driver's `sync()` returns, which should be the robot's native control rate.
   If no new command arrived this cycle, the previous one is sent again, so the
   robot always receives a continuous stream even though mc_rtc runs slower.

   A `Command` carries one of `position`, `velocity` or `torque`. The first
   non-empty one is applied, in that order.

## The `RobotDriver` interface

`robot_interface::RobotDriver`, in
`robot_interface/include/robot_interface/RobotDriverTemplate.h`, is the whole
contract between URI and a robot:

| Method | Required | Called | Contract |
|---|---|---|---|
| `sync()` | yes | once per cycle, first | Block until fresh state is available. It paces the loop, so avoid `sleep()`. |
| `getActualQ()` | yes | every cycle | Joint positions [rad], in the RobotModule's reference joint order. An empty vector means "no state yet", and nothing is published. |
| `getActualQd()` | no | every cycle | Joint velocities [rad/s]; default `{}` |
| `getJointTorques()` | yes | every cycle | Joint torques [Nm] |
| `getIMUs()` | no | every cycle | `name -> IMUData`, where the name matches a RobotModule body sensor; default none |
| `getForceSensors()` | no | every cycle | `name -> WrenchData`, where the name matches a RobotModule force sensor; default none |
| `servoJ(q)` | yes | in `position` mode | Send joint positions [rad] |
| `speedJ(qd)` | yes | in `velocity` mode | Send joint velocities [rad/s] |
| `tauJ(tau)` | yes | in `torque` mode | Send joint torques [Nm] |
| `setDataRead()` | no | not called by `RobotInterface` | Optional hook |

The data types (`IMUData`, `WrenchData`, `GripperInfo`) are plain structs.
They depend on neither mc_rtc nor `robot_comm`, so a driver links only the
header-only `unified_robot_interface::robot_driver_api` target.

### Plugin symbols

A driver `.so` exports four C functions. `mc_rtc::ObjectLoader` uses them to
discover and create the driver:

```cpp
// Exports robot_driver_abi_version(), see "ABI version" below.
ROBOT_DRIVER_EXPORT_ABI_VERSION()

extern "C" {
// Lists the class names this library provides (used as robot_interface.driver).
void ROBOT_DRIVER_PLUGIN(std::vector<std::string> & classes);

// Must have EXACTLY this signature: the loader calls every driver through
// one fixed function-pointer type.
robot_interface::RobotDriver * create(const std::string & name,         // class name
                                         const std::string & ip,
                                         const uint16_t & port,
                                         const std::string & config_path,
                                         const std::vector<robot_interface::GripperInfo> & grippers);

void destroy(robot_interface::RobotDriver * ptr);
}
```

Drivers that link ROS 2 (`rclcpp`) must also export
`extern "C" void LOAD_GLOBAL() {}`. With it, the loader opens the library with
`RTLD_GLOBAL`. Without it, `rclcpp` fails with
`failed to create guard condition: context argument is null`.

### ABI version

Drivers are separate projects built against `RobotDriverTemplate.h`. Adding,
removing or reordering a virtual function of `RobotDriver` (or changing
`create()`'s signature) changes the binary layout. A driver built against the
older header would then call the wrong function, typically a segfault on the
first new call.

To turn that into a clear error, the header defines
`ROBOT_DRIVER_ABI_VERSION`, and each driver embeds it with
`ROBOT_DRIVER_EXPORT_ABI_VERSION()`. When `uri interface` or `uri viewer`
scans the plugin directory, it reads that value from every library.
`load()` refuses a driver whose version differs, or that has none:

```text
[PluginLoader] Plugin 'RobotDriverRTDE' (/opt/uri/lib/robot_interface/libRobotDriverRTDE.so) was built against an incompatible interface: ABI version 2 (expected ABI version 3). Rebuild and reinstall it against the current headers.
```

With autostart, the manager receives that message as the init reply and drops
the robot. The fix is always to rebuild and reinstall the driver.

**When you change `RobotDriver`**, increase `ROBOT_DRIVER_ABI_VERSION` in
`RobotDriverTemplate.h` in the same commit.

### Plugin lookup

`uri interface` and `uri viewer` search these directories, in this order:

1. The directories listed in the `URI_DRIVER_PATH` environment variable
   (colon-separated, like `PATH`; empty entries are ignored).
2. `<CMAKE_INSTALL_PREFIX>/lib/robot_interface`, using the prefix
   `robot_interface` was built with (`ROBOT_INTERFACE_INSTALL_PREFIX` in the
   generated `robot_interface/config.h`).

Duplicate directories are dropped. If two directories contain a driver with the
same class name, the first one will be selected. Without `URI_DRIVER_PATH`, only the
install prefix is searched. Paths are used as given (no `~` expansion), and
missing directories are skipped.

This lets you run a driver that is not installed in URI's prefix, for example
one built in its own build tree, under Nix, or mounted into a container:

```bash
URI_DRIVER_PATH=/home/me/franka_driver/build:/opt/drivers uri interface -c etc/robot_interface.yaml
```

When the variable is set, `uri interface` and `uri viewer` log the search
paths at info level. An interface started by the manager (`autostart: true`)
inherits the manager's environment, so set the variable before `uri manager`.
The variable is local to the machine that loads the driver: it is never sent
from the manager to a remote interface.

`uri` carries the `cap_sys_nice` capability, and `URI_DRIVER_PATH` is honoured
regardless. **Whoever controls the variable or the listed directories controls the
code that runs with that capability.** Only list directories that unprivileged
users cannot write to.

When a driver is missing, the loader logs the searched paths and the drivers it
did find:

```text
[PluginLoader] Plugin 'RobotDriverFoo' not found
[PluginLoader] Searched paths:
  - /home/me/franka_driver/build
  - /opt/uri/lib/robot_interface
[PluginLoader] Available plugins:
  - RobotDriverRTDE
  - RobotDriverROS2Control
```

## Example: bring up a robot-side PC

```bash
# 1. Install URI and the driver with the same prefix on the robot PC.
cmake --install build                          # URI
cmake --install my_driver/build                # driver -> <prefix>/lib/robot_interface

# 2. Check that the driver talks to the robot, without any networking.
uri viewer -c viewer.yaml              # see Getting started, example 4

# 3. Start the interface and leave it running.
uri interface -c ur5e_robot.yaml
# [RobotInterface] 'ur5e' waiting for init query on 'ur5e/init'

# 4. On the control PC: uri manager -c mc_rtc.yaml
# [RobotInterface] 'ur5e' loading driver 'RobotDriverRTDE' (192.168.1.6:0), config_path: '', 0 gripper(s)
# [RobotInterface] 'ur5e' initialized, starting control loop
```

To write the driver itself, see
[Create a new robot driver](docNewRobotDriver.html).

---

[![CNRS-AIST JRL (Joint Robotics Laboratory)](images/jrl-logo.png)](https://unit.aist.go.jp/isri/isri-jrl/en/)[![CNRS](images/cnrs-logo.png)](https://www.cnrs.fr/)[![AIST](images/aist-logo.png)](https://www.aist.go.jp/index_en.html)

Copyright © CNRS-AIST JRL 2026
