# Getting started

This page covers building URI, running it in the most common setups, and
regenerating this documentation.

## Build

### mc_rtc and other controllers

URI is not meant to be tied to mc_rtc. The controller that runs in
`uri manager` is a plugin (see
[Controller backend](docRobotManager.html)), and driver plugins do not
depend on mc_rtc at all. Today, though, **mc_rtc is the only backend**, and
the URI libraries still link mc_rtc for logging and plugin loading. So for
now, installing URI means installing it next to mc_rtc. This will change as
other controllers are supported.

There are three ways to install it:

- **with mc-rtc-superbuild**, the simplest if you
  already build mc_rtc with it;
- **manually**, against an existing mc_rtc install;
- **in the devcontainer**, for a ready-made development
  environment.

### With mc-rtc-superbuild

[mc-rtc-superbuild](https://github.com/mc-rtc/mc-rtc-superbuild) builds and
installs mc_rtc and its dependencies. Its `WITH_URI` option adds URI and
the dependencies mc_rtc doesn't provide:

| Project | Version | Notes |
|---|---|---|
| FlatBuffers | v25.12.19 | Ubuntu's package is too old |
| zenoh-c | 1.9.0 | Built with shared memory, for the `zenoh/shm` protocol |
| zenoh-cpp | 1.9.0 | zenoh-c backend only |
| URI | `main` | Tests and Protobuf disabled |

Enable the option when you configure the superbuild. zenoh-c is written in
Rust: if `cargo` is not found, the configure step installs a Rust toolchain
with [rustup](https://rustup.rs) in `~/.cargo`.

```bash
cd mc-rtc-superbuild
cmake --preset relwithdebinfo -DWITH_URI=ON
cmake --build --preset relwithdebinfo
```

Everything is installed in the superbuild's install prefix. Build your
drivers with that same prefix (see below).

### Manual installation

Dependencies:

- [mc_rtc](https://jrl-umi3218.github.io/mc_rtc/)
- `zenoh-c` (built with `-DZENOHC_BUILD_WITH_SHARED_MEMORY=ON`, needs Rust)
  and `zenoh-cpp` (built with `-DZENOHCXX_ZENOHC=ON -DZENOHCXX_ZENOHPICO=OFF`)
- FlatBuffers, a recent version that provides `flatbuffers_generate_headers()`;
  Protobuf is optional (`-DWITH_PROTOBUF=OFF` to skip it)
- GTest, only for `-DBUILD_TESTING=ON`

Then configure, build and install:

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_PREFIX_PATH="/path/to/mc_rtc/install;/path/to/extra/deps" \
  -DCMAKE_INSTALL_PREFIX=/path/to/install
cmake --build build --parallel
cmake --install build
```

### In the devcontainer

The devcontainer in `.devcontainer/` installs all the dependencies, with
mc_rtc from its `head` apt repository, then builds and installs URI on first
start.
See the top-level `README.md`.

### After installing

Whichever way you install, you also need the RobotModule of each robot you
want to control (e.g. `mc_ur5e`), and a driver plugin for it (see
[Supported robots](docSupportedRobots.html)).

The install prefix matters. `uri interface` looks for driver plugins
in `<CMAKE_INSTALL_PREFIX>/lib/robot_interface` (compiled in), unless the
`URI_DRIVER_PATH` environment variable lists other directories.
Otherwise install drivers with the same prefix. See
[Robot interface](docRobotInterface.html#plugin-lookup).

The build and the install both run `setcap cap_sys_nice+eip` on `uri` through
`sudo`, so you may be prompted for your password. `uri` needs this capability
to request `SCHED_DEADLINE`. Because of it, the dynamic loader ignores
`LD_LIBRARY_PATH` when running `uri`. See
[Troubleshooting](docRobotManager.html) if the manager crashes on exit with
CycloneDDS.

This installs one executable, `uri`, with a subcommand per tool (`uri manager`,
`uri interface`, ...). See [Command line](docCommandLine.html) for all of them
and their options.

### Before running `uri manager`

`uri manager` calls `mlockall()` at startup and exits with code `-2` if that fails.
Raise the memlock limit for your user in `/etc/security/limits.conf`, for
example:

```text
youruser  -  memlock  unlimited
```

Log out and back in after changing it. Also source your mc_rtc environment
(e.g. `setup_mc_rtc.sh`) so that `uri` finds mc_rtc's RobotModules and
controllers.

## Example 1: one robot on the control PC (autostart)

This is the simplest setup: the robot, or its simulator, is reachable from the
control PC, so no separate robot-side PC is needed. `uri manager` starts
`uri interface` itself.

`robot_manager/etc/mc_rtc_ur5e_sim.yaml` drives a Gazebo UR5e through
`ros2_control`:

```yaml
MainRobot: UR5e
Enabled: Posture
Timestep: 0.005

Robots:
  ur5e:
    module: UR5e                 # mc_rtc RobotModule
    network_interface:
      protocol: zenoh
    robot_interface:
      driver: RobotDriverROS2Control
      ip: 127.0.0.1              # unused by this driver, but required
      port: 0
      config_path: /path/to/ros2_control_driver/etc/ur5e.yaml
      autostart: true            # uri manager spawns uri interface itself
```

Prerequisites:

- the `ros2_control_driver` plugin, installed with the same prefix as URI
- the UR5e simulation running, with its `controller_manager` up
- `config_path` edited to point to your checkout

Run:

```bash
uri manager -c robot_manager/etc/mc_rtc_ur5e_sim.yaml
```

What happens, in order:

1. `uri` starts a local Zenoh router and spawns
   `uri interface -c /tmp/robot_manager_ur5e_interface.yaml -n ur5e`.
2. It sends the robot's configuration on the `ur5e/init` query. The
   interface loads `RobotDriverROS2Control` and replies `OK`.
3. The first `ur5e/state` message initializes the mc_rtc robot from the real
   joint positions. mc_rtc then starts running the `Posture` controller.

Stop with `Ctrl+C`. `uri manager` sends `SIGTERM` to the spawned `uri interface`, and
`SIGKILL` if it has not exited within 5 s.

## Example 2: robot on its own PC

Use this when the robot is connected to a different machine, for example a
real-time PC next to the arm.

**On the robot PC**, write a `uri interface` config file. It only needs the
robot's name and how to reach the manager:

```yaml
# ur5e_robot.yaml
name: ur5e
network_interface:
  protocol: zenoh
```

Then start it **before** `uri manager`. It waits for the manager's init query:

```bash
uri interface -c ur5e_robot.yaml
# [RobotInterface] 'ur5e' waiting for init query on 'ur5e/init'
```

**On the control PC**, reference the same robot name in `mc_rtc.yaml`, without
`autostart`:

```yaml
MainRobot: UR5e
Enabled: Posture
Timestep: 0.005

Robots:
  ur5e:
    module: UR5e
    controller:
      mode: position
      time_step: 0.001           # the robot's own control period
    network_interface:
      protocol: zenoh
    robot_interface:
      driver: RobotDriverRTDE    # loaded on the robot PC, not here
      ip: 192.168.1.6            # passed to the driver on the robot PC
      port: 0
```

```bash
uri manager -c mc_rtc.yaml
```

The driver name, IP, port and `config_path` all come from the manager's
config. They are sent in the init query, so switching drivers never requires
touching the robot PC's config. The driver `.so` must be installed on the
robot PC, though.

Both Zenoh peers must be able to discover each other. On a single LAN,
Zenoh's default multicast scouting finds them. Otherwise, point both sides at
a Zenoh config file that lists explicit endpoints (`network_interface.configuration`).
See [Communication](docCommunication.html).

## Example 3: several robots, one controller

Add more entries under `Robots:`. Each one gets its own `uri interface` and
its own topics. `base:` copies another robot's entry and overrides keys:

```yaml
Robots:
  ur5e_left:
    module: UR5e
    network_interface: { protocol: zenoh }
    robot_interface: { driver: RobotDriverRTDE, ip: 192.168.1.2, port: 0 }

  ur5e_right:
    base: ur5e_left              # everything from ur5e_left...
    robot_interface:             # ...except the robot address
      driver: RobotDriverRTDE
      ip: 192.168.1.3
      port: 0
```

Start one `uri interface -c <file> -n ur5e_left` and one with
`-n ur5e_right`, or set `autostart: true` on both. Then run `uri manager`.

## Example 4: test a driver without the manager

`uri viewer` loads a driver plugin directly and prints its joint state
in the terminal. It needs neither mc_rtc's controller nor Zenoh, so it is the
quickest way to check that a new driver connects and reads sensible values:

```yaml
# viewer.yaml
robot_interface:
  driver: RobotDriverRTDE
  ip: 192.168.1.6
  port: 0
  # config_path: /path/to/driver_config.yaml
```

```bash
uri viewer -c viewer.yaml --rate 10
```

It also accepts the manager's `mc_rtc.yaml` directly and uses that robot's
`robot_interface` section. Pick the robot with `-n` when there are several:

```bash
uri viewer -c robot_manager/etc/mc_rtc_ur5e_sim.yaml
uri viewer -c robot_manager/etc/mc_rtc_callm.yaml -n <robot>
```

```text
Robot State Viewer — RobotDriverRTDE @ 192.168.1.6
------------------------------------------------------------
Joint │  Position (rad)  │  Velocity (rad/s)  │  Torque (Nm)
──────┼──────────────────┼────────────────────┼─────────────
  0   │      -0.0012   │       0.0000   │       0.1523
  ...
```

It only reads state. It never calls `servoJ`/`speedJ`/`tauJ`.

## Generate this documentation

The documentation is generated by [hdoc](https://hdoc.io) from
`build/compile_commands.json` and the Markdown pages in `docs/`, as configured
in `.hdoc.toml`. On every push to `main`, the `Documentation` workflow
(`.github/workflows/docs.yaml`) regenerates it and publishes it to the
`gh-pages` branch. To generate it locally:

```bash
cmake -S . -B build          # compile_commands.json must exist
scripts/gen-hdoc-docs.sh
xdg-open hdoc-output/index.html
```

On the first run, the script builds hdoc 1.4.1 against LLVM/Clang 14 and
caches it in `~/.cache/hdoc-build`, which takes a few minutes. Install the
toolchain with:

```bash
sudo apt-get install llvm-14-dev libclang-14-dev clang-14
```

To add a page, create `docs/<Name>.md` and add it to `[pages] paths` in
`.hdoc.toml`. It is published as `doc<Name>.html`, and its file name is the
sidebar title. Images go in `docs/images/`; the script copies that folder
into the output.

---

[![CNRS-AIST JRL (Joint Robotics Laboratory)](images/jrl-logo.png)](https://unit.aist.go.jp/isri/isri-jrl/en/)[![CNRS](images/cnrs-logo.png)](https://www.cnrs.fr/)[![AIST](images/aist-logo.png)](https://www.aist.go.jp/index_en.html)

Copyright © CNRS-AIST JRL 2026
