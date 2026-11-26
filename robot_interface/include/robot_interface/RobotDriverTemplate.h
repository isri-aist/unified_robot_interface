#pragma once

#include <robot_interface/driver/IMUData.h>
#include <robot_interface/driver/WrenchData.h>
#include <robot_interface/driver/api.h>

#include <memory>
#include <vector>

/// ABI version of the RobotDriver interface below (and of the plugin's create()
/// signature). Bump it whenever either changes: adding, removing or reordering
/// a virtual function changes the vtable layout, so a driver built against an
/// older header would otherwise crash (calling through a missing vtable slot)
/// instead of failing to load.
#define ROBOT_DRIVER_ABI_VERSION 4

/// Symbol exported by ROBOT_DRIVER_EXPORT_ABI_VERSION(), checked by
/// PluginLoader before a driver is created.
#define ROBOT_DRIVER_ABI_SYMBOL "robot_driver_abi_version"

/// Put once in each driver plugin's .cpp, next to create()/destroy(). A plugin
/// without it is refused as having been built before ABI versioning.
#define ROBOT_DRIVER_EXPORT_ABI_VERSION()                                   \
  extern "C" ROBOT_DRIVER_DLLEXPORT unsigned int robot_driver_abi_version() \
  {                                                                         \
    return ROBOT_DRIVER_ABI_VERSION;                                        \
  }

namespace robot_interface
{
/// Interface a robot driver plugin implements to connect a robot to URI.
///
/// `uri interface` calls sync() once per cycle, then reads the state with the
/// getters and sends the latest command with servoJ(), speedJ() or tauJ(),
/// depending on the robot's `controller.mode`. Joint vectors follow the
/// RobotModule's reference joint order. See docs/NewRobotDriver.md.
struct ROBOT_DRIVER_DLLAPI RobotDriver
{
  virtual ~RobotDriver() = default;

  /// Wait for the next state update from the robot. This paces the control
  /// loop, so prefer a blocking read over a sleep.
  virtual void sync() = 0;

  /// Called once the state of this cycle has been read. Optional.
  virtual void setDataRead() {}

  /// Joint positions [rad].
  virtual std::vector<double> getActualQ() = 0;

  /// Joint velocities [rad/s]. Empty if the robot does not report them.
  virtual std::vector<double> getActualQd()
  {
    return {};
  }

  /// Joint torques [Nm].
  virtual std::vector<double> getJointTorques() = 0;

  /// Send a joint position command [rad] (`controller.mode: position`).
  virtual void servoJ(const std::vector<double> & q) = 0;

  /// Send a joint velocity command [rad/s] (`controller.mode: velocity`).
  virtual void speedJ(const std::vector<double> & alpha) = 0;

  /// Send a joint torque command [Nm] (`controller.mode: torque`).
  virtual void tauJ(const std::vector<double> & tau) = 0;

  /// IMU readings keyed by the RobotModule's body sensor name (e.g.
  /// "Accelerometer"). Empty by default.
  virtual IMUMap getIMUs()
  {
    return {};
  }

  /// Force/torque readings keyed by the RobotModule's force sensor name
  /// (e.g. "EEForceSensor"), in the sensor frame. Empty by default.
  virtual WrenchMap getForceSensors()
  {
    return {};
  }

  /// Bring the robot to a safe state. Called at most once, in torque mode, when
  /// the control loop ends: manager lost, Ctrl+C or an exception. No command
  /// follows, and the driver is destroyed right after. Must return promptly;
  /// may throw if the robot could not be stopped. Override it unless the
  /// destructor already leaves the robot safe. Does nothing by default.
  virtual void stop() {}
};

typedef std::shared_ptr<RobotDriver> RobotDriverPtr;

} // namespace robot_interface
