#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <mc_rtc/Configuration.h>

#include <robot_comm/CommunicationFactory.h>
#include <robot_comm/serialization/MessageTraits.h>
#include <robot_interface/PluginLoader.h>
#include <robot_interface/RobotDriverTemplate.h>
#include <robot_interface/driver/GripperInfo.h>

namespace robot_interface
{

/// Robot-side process of one robot (`uri interface`).
///
/// Lifecycle:
///   1. Connects via robot_comm (client role).
///   2. Registers a queryable on "{name}/init".
///   3. Blocks until RobotManager sends an init query (robot config as YAML).
///   4. Loads the requested RobotDriver plugin.
///   5. Replies "OK" or "ERROR: <reason>" to the init query.
///   6. Runs the control loop: publishes the driver state on "{name}/state"
///      and applies the latest command from "{name}/command".
///   7. Destroys the driver once the loop ends, after its stop() in torque mode.
class RobotInterface
{
public:
  /// Creates a driver from its class name and the arguments of a plugin's create().
  using DriverFactory = std::function<std::shared_ptr<RobotDriver>(const std::string & driver_name,
                                                                   const std::string & ip,
                                                                   uint16_t port,
                                                                   const std::string & config_path,
                                                                   const std::vector<GripperInfo> & grippers)>;

  /// \param name Robot name, used as the prefix of its topics.
  /// \param comm_config The `network_interface` configuration.
  /// \param driver_factory Creates the driver; defaults to loading its plugin.
  /// \param driver_paths Extra directories searched for the driver plugin by
  /// the default factory, before URI_DRIVER_PATH and the install prefix (see
  /// driverSearchPaths).
  RobotInterface(const std::string & name,
                 const mc_rtc::Configuration & comm_config,
                 DriverFactory driver_factory = {},
                 const std::vector<std::string> & driver_paths = {});

  /// Handle the init handshake, then run the control loop until interrupt is set
  /// or, in torque mode, the manager is lost. Returns false in the latter case.
  bool run(const std::atomic<bool> & interrupt);

  /// Robot name.
  const std::string & name() const
  {
    return name_;
  }

private:
  // Synchronous query handler registered on "{name}/init".
  // Payload: serialized config string (YAML). Returns "OK" or "ERROR: <reason>".
  robot_comm::ByteBuffer handleInitQuery(const robot_comm::ByteBuffer & payload);

  void loadDriver(const std::string & driver_name,
                  const mc_rtc::Configuration & driver_config,
                  const mc_rtc::Configuration & grippers_config);

  void updateSensors();
  void updateControl();

  // Command that keeps the robot still in control_mode_: current position or
  // zero velocity. nullopt if the driver reports no position yet, and in torque
  // mode: zero torque does not hold a robot.
  std::optional<robot_comm::Command> makeHoldCommand();

  // Stop the robot (torque mode) and destroy the driver.
  void shutdownDriver() noexcept;

  // Why the link to the manager counts as lost, or empty while it is fine.
  std::string linkLossReason();

  std::string name_;
  mc_rtc::Configuration comm_config_;
  std::unique_ptr<robot_comm::Communication> comm_;

  // Local to this machine, never received from the manager (ADR 0002).
  bool has_extra_driver_paths_;
  std::vector<std::string> driver_search_paths_;
  PluginLoader<RobotDriver> driver_loader_;
  DriverFactory driver_factory_;
  std::shared_ptr<RobotDriver> driver_;

  // Last received command — re-applied every RTDE cycle to give the robot a
  // continuous stream even when the manager sends at a slower rate than RTDE.
  std::optional<robot_comm::Command> last_cmd_;
  std::chrono::steady_clock::time_point last_cmd_arrival_;

  // Connection watchdog (network_interface.connection_timeout_ms): when the
  // manager goes away, last_cmd_ is replaced by a hold command until new
  // commands arrive. In torque mode, the control loop ends instead.
  std::chrono::milliseconds connection_timeout_;
  bool link_lost_ = false;
  bool stop_requested_ = false;

  // Signalled when the init query has been handled and driver is ready.
  std::mutex init_mutex_;
  std::condition_variable init_cv_;
  bool initialized_ = false;
  std::string init_error_;

  // TODO: wip passing control mode to interface
  std::string control_mode_{};

  // Round-trip latency stats (network_interface.latency_stats: true).
  void recordLatency(const robot_comm::Command & cmd, std::chrono::steady_clock::time_point arrival);
  bool latency_stats_ = false;
  uint64_t last_echo_stamp_ = 0;
  uint64_t last_echo_hold_ = 0;
  size_t rtt_count_ = 0;
  double rtt_sum_us_ = 0.0;
  double rtt_min_us_ = 0.0;
  double rtt_max_us_ = 0.0;
  std::chrono::steady_clock::time_point last_latency_report_;
};

/// Entry point of `uri interface`: loads the config file at config_path, then
/// runs a RobotInterface until interrupt is set. A non-empty name overrides the
/// config's 'name'. The config's optional 'driver_paths' list is searched for
/// the driver before URI_DRIVER_PATH. Returns the process exit code: 0, or 2
/// if a torque-mode robot was stopped because the manager was lost.
int runRobotInterface(const std::string & config_path, std::string name, const std::atomic<bool> & interrupt);

} // namespace robot_interface
