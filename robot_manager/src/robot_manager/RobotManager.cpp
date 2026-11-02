#include <robot_manager/RobotManager.h>

#include <mc_rbdyn/RobotLoader.h>

#include <fmt/core.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <spawn.h>
#include <stdexcept>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

extern char ** environ;

namespace robot_manager
{

namespace
{

std::filesystem::path selfExe()
{
  std::error_code ec;
  auto exe = std::filesystem::read_symlink("/proc/self/exe", ec);
  if(ec) return {};
  return exe;
}

} // namespace

RobotManager::RobotManager() : gconfig_(mc_rtc::Configuration{}) {};

RobotManager::RobotManager(const std::string & mc_config_path, const std::atomic<bool> & interrupt)
: gconfig_(mc_control::MCGlobalController::GlobalConfiguration(mc_config_path))
{
  processGConfig(gconfig_);
  fmt::print("{}\n", gconfig_.config("Robots").dump(true, true));

  const std::string backend = gconfig_.config("Controller", std::string{"mc_rtc"});
  controller_ = controller_loader_.create(backend, gconfig_.config.dump());

  // // Connect to the signal
  // auto & mc_controller = controller_->controller();
  // replace_slot_ = mc_controller.replaceRobot.connect(
  //     [&](const std::string & old_robot_name, const std::string & new_robot_name)
  //     {
  //       std::lock_guard<std::mutex> lock(replace_mutex_);
  //       replace_queue_.push({old_robot_name, new_robot_name});
  //       fmt::print("[mc_rtde] Signal caught: Request switching from {} to {}\n", old_robot_name,
  //       new_robot_name);
  //     });

  init(interrupt);
};

RobotManager::~RobotManager()
{
  if(controller_) controller_->stop();

  cv_.notify_all();

  {
    std::lock_guard<std::mutex> lock(start_mutex_);
    start_control_ = true;
  }
  start_cv_.notify_all();

  if(main_thread_ && main_thread_->joinable())
  {
    main_thread_->join();
  }

  for(auto & t : threads_)
  {
    if(t.joinable())
    {
      t.join();
    }
  }

  stopSpawnedInterfaces();

  fmt::print("RobotManager shutdown complete.\n");
}

bool RobotManager::autostartEnabled(const mc_rtc::Configuration & robot_config)
{
  return robot_config.has("robot_interface") && robot_config("robot_interface").has("autostart")
         && static_cast<bool>(robot_config("robot_interface")("autostart"));
}

pid_t RobotManager::spawnRobotInterface(const std::string & robot_name, const mc_rtc::Configuration & robot_config)
{
  // posix_spawn below passes the parent environment, so the interface inherits URI_DRIVER_PATH.
  mc_rtc::Configuration iface_config;
  iface_config.add("name", robot_name);
  iface_config.add("network_interface", robot_config("network_interface"));
  iface_config.add("robot_interface", robot_config("robot_interface"));
  iface_config.add("controller", robot_config("controller"));

  const auto config_path = std::filesystem::temp_directory_path() / ("robot_manager_" + robot_name + "_interface.yaml");
  iface_config.save(config_path.string());

  auto bin_path = selfExe();
  if(bin_path.empty() || !std::filesystem::exists(bin_path))
  {
    bin_path = "uri"; // fall back to PATH lookup
  }

  fmt::print("[RobotManager] Spawning co-located robot_interface for '{}': {} interface -c {} -n {}\n", robot_name,
             bin_path.string(), config_path.string(), robot_name);

  // posix_spawn (rather than fork()+exec()) avoids duplicating this process's
  // other threads (GUI server, ROS, ...) into the child: fork() in a
  // multi-threaded process only carries over the calling thread, so any lock
  // held by another thread at that instant is inherited pre-locked with no
  // owner left to release it. mlockall(MCL_FUTURE), active here since
  // main.cpp, makes plain fork() doubly unsafe (see fork(2)/mlockall(2)).
  const std::string bin_path_str = bin_path.string();
  const std::string config_path_str = config_path.string();
  std::array<char *, 7> argv{const_cast<char *>(bin_path_str.c_str()),
                             const_cast<char *>("interface"),
                             const_cast<char *>("-c"),
                             const_cast<char *>(config_path_str.c_str()),
                             const_cast<char *>("-n"),
                             const_cast<char *>(robot_name.c_str()),
                             nullptr};

  pid_t pid = 0;
  int err = posix_spawnp(&pid, bin_path_str.c_str(), nullptr, nullptr, argv.data(), environ);
  if(err != 0)
  {
    fmt::print("[RobotManager][error] posix_spawn failed while spawning robot_interface for '{}': {}\n", robot_name,
               std::strerror(err));
    return -1;
  }

  spawned_interfaces_[robot_name] = pid;
  return pid;
}

void RobotManager::stopSpawnedInterfaces()
{
  for(const auto & [robot_name, pid] : spawned_interfaces_)
  {
    if(pid <= 0)
    {
      continue;
    }

    fmt::print("[RobotManager] Stopping robot_interface '{}' (pid {})\n", robot_name, pid);

    if(::kill(pid, SIGTERM) != 0 && errno != ESRCH)
    {
      fmt::print(stderr,
                 "[RobotManager][warning] Failed to signal "
                 "robot_interface '{}' (pid {}): {}\n",
                 robot_name, pid, std::strerror(errno));
    }
  }

  for(const auto & [robot_name, pid] : spawned_interfaces_)
  {
    if(pid <= 0)
    {
      continue;
    }

    int status = 0;
    bool reaped = false;

    for(int i = 0; i < 50; ++i)
    {
      const pid_t result = ::waitpid(pid, &status, WNOHANG);

      if(result == pid)
      {
        reaped = true;
        break;
      }

      if(result < 0)
      {
        if(errno == EINTR)
        {
          continue;
        }

        if(errno == ECHILD)
        {
          reaped = true;
        }
        else
        {
          fmt::print(stderr,
                     "[RobotManager][warning] waitpid failed for "
                     "robot_interface '{}' (pid {}): {}\n",
                     robot_name, pid, std::strerror(errno));
        }

        break;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds{100});
    }

    if(reaped)
    {
      continue;
    }

    fmt::print(stderr,
               "[RobotManager][warning] robot_interface '{}' "
               "(pid {}) did not exit after SIGTERM, sending SIGKILL\n",
               robot_name, pid);

    if(::kill(pid, SIGKILL) != 0 && errno != ESRCH)
    {
      fmt::print(stderr,
                 "[RobotManager][warning] Failed to kill "
                 "robot_interface '{}' (pid {}): {}\n",
                 robot_name, pid, std::strerror(errno));
    }

    while(::waitpid(pid, &status, 0) < 0)
    {
      if(errno == EINTR)
      {
        continue;
      }

      if(errno != ECHILD)
      {
        fmt::print(stderr,
                   "[RobotManager][warning] Final waitpid failed for "
                   "robot_interface '{}' (pid {}): {}\n",
                   robot_name, pid, std::strerror(errno));
      }

      break;
    }
  }

  spawned_interfaces_.clear();
}

void RobotManager::init(const std::atomic<bool> & interrupt)
{
  fmt::print("manager init start\n");

  mc_rtc::Configuration robots_config = gconfig_.config("Robots");

  /* Start Zenoh Router if necessary */
  if(robots_config.dump().find("zenoh") != std::string::npos)
  {
    launchZenohRouter();
  }

  /* Set up robot interface and communication*/
  for(auto & robot_name : robots_config.keys())
  {
    fmt::print("manager init robot {}\n", robot_name);

    if(interfaces_.count(robot_name) != 0)
    {
      fmt::print("[RobotManger][error] Skip already exists robot interface {}\n", robot_name);
      continue;
    }

    mc_rtc::Configuration robot_config{robots_config(robot_name)};

    if(robot_config.has("module"))
    {
      const std::string module_name = robot_config("module");
      try
      {
        auto rm = mc_rbdyn::RobotLoader::get_robot_module(module_name);
        if(!rm->grippers().empty())
        {
          mc_rtc::Configuration grippers_config = robot_config.add("grippers");
          for(const auto & gripper : rm->grippers())
          {
            grippers_config.add(gripper.name, gripper.joints);
          }
        }
      }
      catch(const std::exception & e)
      {
        fmt::print("[RobotManager][warning] Could not load module '{}' for robot '{}' to check for grippers: {}\n",
                   module_name, robot_name, e.what());
      }
    }

    if(autostartEnabled(robot_config))
    {
      spawnRobotInterface(robot_name, robot_config);
    }

    std::unique_ptr<RobotInterfaceBase> interface = RobotInterfaceFactory::makeInterface(robot_name, robot_config);
    if(!interface)
    {
      continue;
    }

    interfaces_.try_emplace(robot_name, std::move(interface));
  }

  /* Send init query to each robot_interface process (query/reply: waits for driver load confirmation). */
  // TODO: adding wait time like this might be a bad fix. Need a better robot-is-ready detection
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  std::vector<std::string> failed_robots;
  for(auto & [robot_name, interface] : interfaces_)
  {
    // Driver constructors may block for a while.
    constexpr auto init_query_timeout = std::chrono::seconds(60);
    const std::string init_topic = robot_name + "/init";
    auto config_payload = interface->communication().serializer()->serialize(interface->config().dump());

    // The first reply may be lost while Zenoh discovers a freshly spawned peer.
    constexpr int max_attempts = 5;
    std::optional<robot_comm::ByteBuffer> reply;
    for(int attempt = 0; attempt < max_attempts && !reply; ++attempt)
    {
      if(attempt > 0)
      {
        fmt::print("[robot_manager] Retrying init query to '{}' (attempt {}/{})…\n", robot_name, attempt + 1,
                   max_attempts);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
      fmt::print("[robot_manager] Querying robot_interface '{}' (waiting up to {} s)…\n", robot_name,
                 init_query_timeout.count());
      reply = interface->communication().query(init_topic, config_payload, init_query_timeout);
    }

    if(!reply)
    {
      fmt::print("[RobotManager][error] Init query to robot_interface '{}' timed out after {} attempts — is the "
                 "process running? Skipping this robot.\n",
                 robot_name, max_attempts);
      failed_robots.push_back(robot_name);
      continue;
    }

    const std::string reply_str(reply->begin(), reply->end());
    if(reply_str != "OK")
    {
      fmt::print("[RobotManager][error] Robot '{}' init failed: {} — skipping.\n", robot_name, reply_str);
      failed_robots.push_back(robot_name);
      continue;
    }

    fmt::print("[RobotManager] Robot '{}' driver loaded successfully", robot_name);
  }

  for(const auto & name : failed_robots)
  {
    interfaces_.erase(name);
  }

  if(interfaces_.empty())
  {
    throw std::runtime_error(
        fmt::format("[RobotManager][error] No robot_interface responded — cannot start controller.\n"));
  }

  /* Check timestep compatifibility between mc_rtc and robot */
  double controller_s = controller_->timeStep();
  size_t max_step_size{0};
  for(auto & [robot_name, interface] : interfaces_)
  {
    double cycle_s = interface->dt();
    auto cycle_ns = static_cast<size_t>(cycle_s * 1e9);
    auto controller_ns = static_cast<size_t>(controller_s * 1e9);
    if(controller_ns < cycle_ns)
    {
      throw std::runtime_error(fmt::format("[RobotManager][error] mc_rtc cannot run faster than the robot's control "
                                           "frequency (RobotTimeStep= {}s, Timestep={}s)\n",
                                           cycle_s, controller_s));
    }

    if(controller_ns % cycle_ns != 0)
    {
      throw std::runtime_error(fmt::format("[RobotManager][error] mc_rtc timestep must be a multiple of the robot's "
                                           "control loop frequency (RobotTimeStep= {}s, Timestep={}s)\n",
                                           cycle_s, controller_s));
    }

    size_t step_size = controller_ns / cycle_ns;
    size_t freq = std::ceil(1 / controller_s);
    size_t robot_freq = std::ceil(1 / cycle_s);
    fmt::print("[RobotManager] mc_rtc running at {}Hz, robot running at {}Hz\n", freq, robot_freq);

    if(max_step_size < step_size)
    {
      max_step_size = step_size;
    }
  }

  fmt::print("[RobotManager] mc_rtc will compute commands every {} robot control step\n", max_step_size);

  /* Initialize all real robots */
  controller_->initializeRobots();

  /* Init threads */
  controller_->start();

  for(auto & [robot_name, interface] : interfaces_)
  {
    auto * interface_ptr = interface.get();
    threads_.emplace_back([&, this, interface_ptr]()
                          { interface_ptr->controlThread(*controller_, start_mutex_, start_cv_, start_control_); });
  }

  main_thread_ = std::make_unique<std::thread>(&RobotManager::mainThread, this, max_step_size, std::ref(interrupt));

  fmt::print("manager init done\n");
}

void RobotManager::launchZenohRouter()
{
  if(zenoh_router_) return;

  // Installed with uri (robot_manager/etc/zenoh_router.json5), unless the config names another file
  const std::string zenoh_config_path =
      gconfig_.config("ZenohRouter", mc_rtc::Configuration{})("configuration", std::string{URI_ZENOH_ROUTER_CONFIG});
  if(!std::filesystem::exists(zenoh_config_path))
  {
    throw std::runtime_error(fmt::format("[RobotManager] Zenoh router config not found: {}. Install uri, or set "
                                         "ZenohRouter: {{configuration: <file>}} in the manager config",
                                         zenoh_config_path));
  }
  zenoh::Config config = zenoh::Config::from_file(zenoh_config_path);
  fmt::print("[robot_manager] Launching Zenoh router from {}\n", zenoh_config_path);

  zenoh_router_ = std::make_unique<zenoh::Session>(zenoh::Session::open(std::move(config)));

  // Let the router fully start before clients try to connect
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
}

void RobotManager::processGConfig(mc_control::MCGlobalController::GlobalConfiguration & gconfig)
{
  if(!gconfig.config.has("Robots"))
  {
    throw std::runtime_error(
        fmt::format("No `Robots` section in the configuration, see etc/mc_rtc.yaml for an example\n"));
  }

  fmt::print("manager processGConfig 1\n");

  mc_rtc::Configuration robots_config = gconfig.config("Robots");
  for(auto & robot_name : robots_config.keys())
  {
    mc_rtc::Configuration robot_config{gconfig.config("Robots")(robot_name)};

    // Copy config setting from a previous robot
    if(robot_config.has("base"))
    {
      mc_rtc::Configuration base_config{};
      base_config.load(robots_config(robot_config("base")));
      base_config.load(robot_config);
      robot_config.load(base_config);
    }

    if(!robot_config.has("module"))
    {
      throw std::runtime_error(fmt::format("Missing 'module' configuration for robot '{}'\n", robot_name));
    }

    // Defaults to the mc_rtc Timestep.
    if(!robot_config.has("controller"))
    {
      robot_config.add("controller");
    }
    if(!robot_config("controller").has("mode"))
    {
      robot_config("controller").add("mode", std::string{"position"});
    }
    if(!robot_config("controller").has("time_step"))
    {
      robot_config("controller").add("time_step", gconfig.timestep);
    }

    // robot_interface section carries the driver name and robot-side ip/port.
    if(robot_config.has("robot_interface"))
    {
      if(!robot_config("robot_interface").has("driver"))
      {
        throw std::runtime_error(
            fmt::format("Missing 'robot_interface.driver' configuration for robot '{}'\n", robot_name));
      }
      if(!robot_config("robot_interface").has("ip"))
      {
        throw std::runtime_error(
            fmt::format("Missing 'robot_interface.ip' configuration for robot '{}'\n", robot_name));
      }
      if(!robot_config("robot_interface").has("port"))
      {
        throw std::runtime_error(
            fmt::format("Missing 'robot_interface.port' configuration for robot '{}'\n", robot_name));
      }
    }
    else
    {
      throw std::runtime_error(fmt::format("Missing 'robot_interface' configuration for robot '{}'\n", robot_name));
    }

    // TODO: is "network_interface" only include "protocol"?
    if(!robot_config.has("network_interface"))
    {
      robot_config.add("network_interface");
    }
    if(!robot_config("network_interface").has("protocol"))
    {
      robot_config("network_interface").add("protocol", std::string{"zenoh/shm"});
    }
  }

  fmt::print("manager processGConfig done\n");
}

void RobotManager::mainThread(size_t step_size, const std::atomic<bool> & interrupt)
{
  size_t step = 0;

  double cycle_s = 0.005; // fallback: 200Hz
  if(!interfaces_.empty()) cycle_s = interfaces_.begin()->second->dt();

  using Clock = std::chrono::steady_clock;
  using Duration = std::chrono::duration<double>;
  auto next_wake = Clock::now();

  while(controller_->isRunning())
  {
    // Sleep until the next scheduled cycle tick.
    next_wake += std::chrono::duration_cast<Clock::duration>(Duration(cycle_s));
    std::this_thread::sleep_until(next_wake);

    if(interrupt)
    {
      controller_->stop();
      return;
    }

    for(auto & [robot_name, interface] : interfaces_) interface->updateSensors(*controller_);

    // A lost robot leaves the controller running on stale state: stop. The
    // other robots' interfaces hold their robot once commands stop.
    for(auto & [robot_name, interface] : interfaces_)
    {
      if(const auto error = interface->connectionError(); !error.empty())
      {
        fmt::print(stderr, "[RobotManager][error] Lost connection to robot '{}': {}. Stopping the controller.\n",
                   robot_name, error);
        controller_->stop();
        return;
      }
    }

    // Gate controller.run() until every interface has fed its initial sensor
    // values and called gc.init(), otherwise the Posture task would command
    // the robot from a default (potentially far) configuration.
    bool all_ready =
        std::all_of(interfaces_.begin(), interfaces_.end(), [](const auto & kv) { return kv.second->isInitialized(); });

    if(all_ready && step % step_size == 0) controller_->run();

    {
      std::lock_guard<std::mutex> lock(start_mutex_);
      start_control_ = true;
    }
    start_cv_.notify_all();

    for(auto & [robot_name, interface] : interfaces_) interface->updateControl(*controller_);
    step++;
  }
}

} // namespace robot_manager
