#!/usr/bin/env bash
#
# Embedded in the `uri` binary and run as `uri create_new_driver` (see
# robot_manager/src/CMakeLists.txt); it can also be run directly from the source tree.
#
# Usage:
#   uri create_new_driver <DriverName> [output-dir]
#
# Example:
#   uri create_new_driver Franka
#     -> creates ./franka_driver/ with class franka_driver::RobotDriverFranka
#
#   uri create_new_driver Franka /path/to/somewhere
#     -> creates /path/to/somewhere/franka_driver/
#
set -euo pipefail

usage()
{
  cat <<EOF
Usage: $(basename "$0") [--no-git] <DriverName> [output-dir]

Generates a standalone robot_interface driver project implementing
robot_interface::RobotDriver (see robot_interface/include/robot_interface/RobotDriverTemplate.h),
modeled after rtde_driver/.

  <DriverName>   PascalCase identifier for the robot/driver, e.g. "Franka", "Kinova".
                 The generated class is named RobotDriver<DriverName>.
  [output-dir]   Directory in which to create the new project (default: current directory).
  --no-git       Skip initializing a git repository in the generated project.

Example:
  $(basename "$0") Franka
    -> ./franka_driver/{CMakeLists.txt, src/, include/franka_driver/, etc/, README.md, .gitignore}
       plus a freshly initialized git repository.
EOF
}

GIT_INIT=1
ARGS=()
for arg in "$@"; do
  case "$arg" in
    -h|--help)
      usage
      exit 0
      ;;
    --no-git)
      GIT_INIT=0
      ;;
    *)
      ARGS+=("$arg")
      ;;
  esac
done
set -- "${ARGS[@]+"${ARGS[@]}"}"

if [ $# -lt 1 ]; then
  usage
  exit 1
fi

DRIVER_NAME="$1"

if ! [[ "$DRIVER_NAME" =~ ^[A-Za-z][A-Za-z0-9]*$ ]]; then
  echo "Error: <DriverName> must be a valid PascalCase identifier (letters/digits, starting with a letter)." >&2
  exit 1
fi
# Capitalize the first letter so RobotDriver<Name> is well-formed even if the user typed lowercase.
DRIVER_NAME="$(tr '[:lower:]' '[:upper:]' <<< "${DRIVER_NAME:0:1}")${DRIVER_NAME:1}"

OUT_DIR="${2:-$(pwd)}"

# PascalCase -> snake_case (e.g. "URRobot" -> "ur_robot", "Franka" -> "franka")
SNAKE="$(sed -E 's/([a-z0-9])([A-Z])/\1_\2/g; s/([A-Z]+)([A-Z][a-z])/\1_\2/g' <<< "$DRIVER_NAME" | tr '[:upper:]' '[:lower:]')"

PROJECT="${SNAKE}_driver"
CLASS="RobotDriver${DRIVER_NAME}"
TARGET_DIR="${OUT_DIR}/${PROJECT}"

if [ -e "$TARGET_DIR" ]; then
  echo "Error: $TARGET_DIR already exists, refusing to overwrite." >&2
  exit 1
fi

render()
{
  sed -e "s/__PROJECT__/${PROJECT}/g" -e "s/__CLASS__/${CLASS}/g" -e "s/__NAME__/${DRIVER_NAME}/g" -e "s/__SNAKE__/${SNAKE}/g"
}

mkdir -p "$TARGET_DIR"/include/"$PROJECT" "$TARGET_DIR"/src "$TARGET_DIR"/etc

# ── <project>/CMakeLists.txt ──────────────────────────────────────────────────
render > "$TARGET_DIR/CMakeLists.txt" <<'TEMPLATE'
cmake_minimum_required(VERSION 3.22)

set(PROJECT_NAME __PROJECT__)
set(PROJECT_VERSION 1.0.0)
project(
  ${PROJECT_NAME}
  LANGUAGES CXX
  VERSION ${PROJECT_VERSION}
)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# CMAKE_INSTALL_LIBDIR/BINDIR (used by src/CMakeLists.txt's install())
include(GNUInstallDirs)

# Dependencies
find_package(unified_robot_interface REQUIRED)
# TODO: find_package() your robot's SDK/client library here, e.g.:
# find_package(my_robot_sdk REQUIRED)

add_subdirectory(src)

install(DIRECTORY include/ DESTINATION include)
TEMPLATE

# ── <project>/src/CMakeLists.txt ──────────────────────────────────────────────
render > "$TARGET_DIR/src/CMakeLists.txt" <<'TEMPLATE'
add_library(__CLASS__ SHARED RobotDriver__NAME__.cpp)

target_include_directories(
  __CLASS__
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
         $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  __CLASS__ PUBLIC unified_robot_interface::robot_driver_api
)
# TODO: link your robot's SDK/client library here, e.g.:
# target_link_libraries(__CLASS__ PRIVATE my_robot_sdk::my_robot_sdk)

add_robot_driver(__CLASS__)

# Plain lib/ (not CMAKE_INSTALL_LIBDIR, which may be multiarch): RobotInterface
# only searches <prefix>/lib/robot_interface for driver plugins.
install(
  TARGETS __CLASS__
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION lib/robot_interface
  ARCHIVE DESTINATION lib/robot_interface
)
TEMPLATE

# ── <project>/include/<project>/RobotDriver<Name>.h ──────────────────────────
render > "$TARGET_DIR/include/$PROJECT/RobotDriver${DRIVER_NAME}.h" <<'TEMPLATE'
#pragma once

#include <robot_interface/RobotDriverTemplate.h>
#include <robot_interface/driver/GripperInfo.h>

#include <memory>
#include <string>
#include <vector>

namespace __PROJECT__
{

class __CLASS__ : public robot_interface::RobotDriver
{
public:
  // config_path: optional path to a driver-specific config file, forwarded
  // as-is from the `robot_interface.config_path` key in mc_rtc.yaml. Ignore
  // it (as below) if this driver only needs ip/port.
  __CLASS__(const std::string & ip, uint16_t port = 0, const std::string & config_path = "");

  ~__CLASS__() override;

  // Called once per control cycle, before any of the getters below.
  // TODO: read/receive the latest state from the robot here.
  void sync() override;

  // Optional: called once the interface has consumed this cycle's state.
  // void setDataRead() override {}

  // TODO: return the current joint positions [rad].
  std::vector<double> getActualQ() override;

  // Optional: return the current joint velocities [rad/s]. Remove this
  // override if the robot does not report velocities (defaults to {}).
  std::vector<double> getActualQd() override;

  // TODO: return the current joint torques [Nm].
  std::vector<double> getJointTorques() override;

  // TODO: send a joint position command [rad].
  void servoJ(const std::vector<double> & q) override;

  // TODO: send a joint velocity command [rad/s].
  void speedJ(const std::vector<double> & alpha) override;

  // TODO: send a joint torque command [Nm].
  void tauJ(const std::vector<double> & tau) override;

private:
  std::string ip_;
  // TODO: add your driver/connection handle(s) here.
};

} // namespace __PROJECT__

// ── robot_interface plugin symbols ────────────────────────────────────────────
#include <robot_interface/driver/api.h>

extern "C"
{
  ROBOT_DRIVER_DLLAPI void ROBOT_DRIVER_PLUGIN(std::vector<std::string> & classes);

  // grippers: gripper name -> joint names, resolved from the RobotModule by
  // RobotManager and forwarded via the init query. Ignore it (as below) if
  // this robot has no grippers.
  ROBOT_DRIVER_DLLAPI robot_interface::RobotDriver * create(
      const std::string & name, const std::string & ip, const uint16_t & port, const std::string & config_path,
      const std::vector<robot_interface::GripperInfo> & grippers);

  ROBOT_DRIVER_DLLAPI void destroy(robot_interface::RobotDriver * ptr);
}
TEMPLATE

# ── <project>/src/RobotDriver<Name>.cpp ───────────────────────────────────────
render > "$TARGET_DIR/src/RobotDriver${DRIVER_NAME}.cpp" <<'TEMPLATE'
#include <__PROJECT__/RobotDriver__NAME__.h>

#include <fmt/core.h>

namespace __PROJECT__
{

__CLASS__::__CLASS__(const std::string & ip, uint16_t /*port*/, const std::string & /*config_path*/) : ip_(ip)
{
  fmt::print("[__CLASS__] Connecting to robot at {}\n", ip_);

  // TODO: open the connection to the robot / hardware SDK here.
  // Throw on failure -- RobotDriverLoader will propagate the exception to the caller.
}

__CLASS__::~__CLASS__()
{
  // TODO: close the connection / stop the control loop here.
}

void __CLASS__::sync()
{
  // TODO: block or poll until a new state update is available from the robot.
  // This paces the control loop, so prefer a blocking read over a sleep().
}

std::vector<double> __CLASS__::getActualQ()
{
  // TODO: return the joint positions read during sync().
  return {};
}

std::vector<double> __CLASS__::getActualQd()
{
  // TODO: return the joint velocities read during sync(), or delete this
  // override (and its declaration in the header) if unsupported.
  return {};
}

std::vector<double> __CLASS__::getJointTorques()
{
  // TODO: return the joint torques read during sync().
  return {};
}

void __CLASS__::servoJ(const std::vector<double> & q)
{
  // TODO: send a joint position command to the robot.
}

void __CLASS__::speedJ(const std::vector<double> & alpha)
{
  // TODO: send a joint velocity command to the robot.
}

void __CLASS__::tauJ(const std::vector<double> & tau)
{
  // TODO: send a joint torque command to the robot.
}

} // namespace __PROJECT__

// Lets robot_interface refuse this plugin (instead of crashing) once the
// RobotDriver interface changes and the plugin needs a rebuild.
ROBOT_DRIVER_EXPORT_ABI_VERSION()

extern "C"
{
  void ROBOT_DRIVER_PLUGIN(std::vector<std::string> & classes)
  {
    classes.push_back("__CLASS__");
  }

  robot_interface::RobotDriver * create(const std::string & /*name*/, const std::string & ip,
                                           const uint16_t & port, const std::string & config_path,
                                           const std::vector<robot_interface::GripperInfo> & /*grippers*/)
  {
    return new __PROJECT__::__CLASS__(ip, port, config_path);
  }

  void destroy(robot_interface::RobotDriver * ptr)
  {
    delete ptr;
  }
}
TEMPLATE

# ── <project>/etc/robot_interface.yaml ────────────────────────────────────────
render > "$TARGET_DIR/etc/robot_interface.yaml" <<'TEMPLATE'

robot_interface:
  driver: __CLASS__
  ip: 192.168.1.100
# port: 0   # optional -- uncomment and set if your driver uses it
# config_path: /path/to/driver_config.yaml   # optional -- driver-specific config file

network_interface:
  protocol: zenoh
TEMPLATE

# ── <project>/etc/<snake>_robot.yaml ──────────────────────────────────────────
render > "$TARGET_DIR/etc/${SNAKE}_robot.yaml" <<'TEMPLATE'
name: __SNAKE__

network_interface:
  protocol: zenoh

robot_interface:
  driver: __CLASS__
  ip: 192.168.1.100
  # port: 0   # optional
TEMPLATE

# ── <project>/README.md ───────────────────────────────────────────────────────
render > "$TARGET_DIR/README.md" <<'TEMPLATE'
# __PROJECT__

A `robot_interface` driver plugin for TODO: name your robot/controller.

It implements the `robot_interface::RobotDriver` interface
(see `robot_interface/include/robot_interface/RobotDriverTemplate.h`) so it can be
loaded by `robot_interface` at runtime as a shared library -- no mc_rtc dependency
required.

## Dependencies

| Dependency | Where to get it |
|---|---|
| `unified_robot_interface` | Built from the `unified_robot_interface` repository |
| TODO | TODO: add your robot's SDK/client library here |

## Build

```bash
cd __PROJECT__
mkdir build && cd build
cmake .. \
  -DCMAKE_PREFIX_PATH="/path/to/unified_robot_interface/install" \
  -DCMAKE_INSTALL_PREFIX=/path/to/unified_robot_interface/install
cmake --build .
cmake --install .
```

The plugin is installed to `lib/robot_interface/lib__CLASS__.so`. To use it
without installing it into URI's prefix, set `URI_DRIVER_PATH` to the directory
containing the `.so` (e.g. `build/`) when running `uri interface` or `uri viewer`.

## Implementing the driver

Fill in the `TODO`s in `src/RobotDriver__NAME__.cpp` (and `CMakeLists.txt` /
`src/CMakeLists.txt` for extra dependencies):

- Constructor: open the connection to the robot.
- `sync()`: block/poll for a new state update -- this paces the control loop.
- `getActualQ()` / `getActualQd()` / `getJointTorques()`: report the latest state.
- `servoJ()` / `speedJ()` / `tauJ()`: send position/velocity/torque commands.

Test it without the manager using `uri viewer -c etc/robot_interface.yaml`
(installed with `unified_robot_interface`), which loads the plugin and prints
its joint state.

## Plugin API

The shared library exports four C symbols consumed by `robot_interface`'s plugin loader:

```cpp
void ROBOT_DRIVER_PLUGIN(std::vector<std::string> & classes);  // registers "__CLASS__"
robot_interface::RobotDriver * create(const std::string & name,
                                         const std::string & ip,
                                         const uint16_t & port,
                                         const std::string & config_path,
                                         const std::vector<robot_interface::GripperInfo> & grippers);
void destroy(robot_interface::RobotDriver * ptr);
unsigned int robot_driver_abi_version();  // from ROBOT_DRIVER_EXPORT_ABI_VERSION()
```

`robot_interface` refuses to load a plugin whose ABI version differs from the
headers it was built with. After updating `unified_robot_interface`, rebuild
and reinstall this driver if loading fails with "incompatible interface".

Configure it in `robot_manager/etc/mc_rtc.yaml` under the `robot_interface` key of the
relevant robot entry:

```yaml
Robots:
  __SNAKE__:
    module: TODO_module_name
    robot_interface:
      driver: __CLASS__
      ip: 192.168.1.100
      # port: 0
      # config_path: /path/to/driver_config.yaml
```
TEMPLATE

# ── <project>/.gitignore ───────────────────────────────────────────────────────
render > "$TARGET_DIR/.gitignore" <<'TEMPLATE'
build/
install/
compile_commands.json
TEMPLATE

GIT_STATUS="skipped (--no-git)"
if [ "$GIT_INIT" -eq 1 ]; then
  if command -v git >/dev/null 2>&1; then
    git -C "$TARGET_DIR" init -q
    GIT_STATUS="initialized (run 'git add -A && git commit' in $PROJECT/ when ready)"
  else
    GIT_STATUS="skipped (git not found on PATH)"
  fi
fi

echo "Created new driver project: $TARGET_DIR"
echo ""
echo "  $PROJECT/"
echo "  ├── CMakeLists.txt"
echo "  ├── src/"
echo "  │   ├── CMakeLists.txt"
echo "  │   └── RobotDriver${DRIVER_NAME}.cpp"
echo "  ├── include/$PROJECT/"
echo "  │   └── RobotDriver${DRIVER_NAME}.h"
echo "  ├── etc/"
echo "  │   ├── robot_interface.yaml"
echo "  │   └── ${SNAKE}_robot.yaml"
echo "  ├── README.md"
echo "  └── .gitignore"
echo ""
echo "Git repository: $GIT_STATUS"
echo ""
echo "Next steps:"
echo "  1. Fill in the TODOs in src/RobotDriver${DRIVER_NAME}.cpp (and add any SDK"
echo "     find_package()/target_link_libraries() calls in CMakeLists.txt)."
echo "  2. Build & install against your unified_robot_interface install prefix (see README.md)."
echo "  3. Point a robot's robot_interface.driver at \"$CLASS\" in mc_rtc.yaml."
