#pragma once

#include <robot_comm/CommunicationFactory.h>
#include <robot_controller/ControllerLoader.h>
#include <robot_interface/RobotInterfaceFactory.h>

#include <mc_control/mc_global_controller.h>
#include <mc_rtc/SignalSlot.h>
#include <zenoh.hxx>

#include <sys/types.h>

#include <atomic>
#include <condition_variable>
#include <memory>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace robot_manager
{

/// Run the manager returned by init() until the controller stops or
/// interrupt is set.
void run(void * data, const std::atomic<bool> & interrupt);

/// Create a RobotManager from an mc_rtc.yaml file. Returns an owning pointer
/// to it.
void * init(const std::string & mc_config_path, uint64_t & cycle_ns, const std::atomic<bool> & interrupt);

/// Core of `uri manager`: runs the controller and one proxy per robot.
///
/// Reads the `Robots:` section of mc_rtc.yaml, starts the co-located
/// interfaces (`autostart`), sends each robot its init query, then exchanges
/// states and commands with every robot. See docs/RobotManager.md.
class RobotManager
{
public:
  RobotManager();

  /// Load `mc_config_path` and initialize every robot. Returns once all
  /// robots are initialized or interrupt is set.
  RobotManager(const std::string & mc_config_path, const std::atomic<bool> & interrupt);

  ~RobotManager();

  RobotManager(const RobotManager &) = delete;
  RobotManager & operator=(const RobotManager &) = delete;
  RobotManager(RobotManager &&) = delete;
  RobotManager & operator=(RobotManager &&) = delete;

  /// The controller backend.
  [[nodiscard]] robot_controller::Controller & controller()
  {
    return *controller_;
  }

  /// Robot name -> manager-side proxy.
  [[nodiscard]] const auto & interfaces()
  {
    return interfaces_;
  }

  /// Wake the main thread for the next control step.
  void notify()
  {
    cv_.notify_one();
  }

private:
  void init(const std::atomic<bool> & interrupt);

  robot_controller::ControllerLoader controller_loader_{};
  robot_controller::ControllerLoader::ControllerPtr controller_{};

  // TODO: make zenoh router optional
  void launchZenohRouter();
  std::unique_ptr<zenoh::Session> zenoh_router_;

  std::unordered_map<std::string, std::unique_ptr<RobotInterfaceBase>> interfaces_{};

  /* Co-located robot_interface processes (robot_interface.autostart: true) */
  static bool autostartEnabled(const mc_rtc::Configuration & robot_config);
  pid_t spawnRobotInterface(const std::string & robot_name, const mc_rtc::Configuration & robot_config);
  void stopSpawnedInterfaces();
  std::unordered_map<std::string, pid_t> spawned_interfaces_{};
  // Top-level DriverPaths, written into each autostarted interface's config.
  std::vector<std::string> driver_paths_{};

  /* Process configuration */
  void processGConfig(mc_control::MCGlobalController::GlobalConfiguration & gconfig);
  mc_control::MCGlobalController::GlobalConfiguration gconfig_;

  /* Start */
  // Start with main thread
  std::unique_ptr<std::thread> main_thread_;
  std::mutex start_mutex_;
  std::condition_variable start_cv_;
  bool start_control_{false};

  /* Multi-threading */
  std::condition_variable cv_;
  std::vector<std::thread> threads_;
  void mainThread(size_t step_size, const std::atomic<bool> & interrupt);

  /* Replace tools */
  struct ReplaceRequest
  {
    std::string old_robot_name;
    std::string new_robot_name;
  };
  mc_rtc::Slot<std::string, std::string> replace_slot_;
  std::queue<ReplaceRequest> replace_queue_;
  bool signal_received_ = false;
  mutable std::mutex replace_mutex_;

  // TODO idea here is to have a thread for each communication we are lauching for each robot
  // Would be nice to consider the case where robot are sharing the same timestep and protocol
  // Each thread is running at each robot dt.
  // Warning should be set in case the dt outreach protocol capacities
};

} // namespace robot_manager
