#include <robot_interface/RobotInterface.h>

#include <robot_comm/CommunicationZenoh.h>
#include <robot_comm/serialization/MessageTraits.h>

#include <mc_rtc/logging.h>
#include <cstdlib>
#include <fmt/ranges.h>
#include <robot_interface/config.h>
#include <robot_interface/driver/DriverSearchPaths.h>
#include <robot_interface/driver/GripperInfo.h>

#include <chrono>
#include <sys/prctl.h>

namespace robot_interface
{

RobotInterface::RobotInterface(const std::string & name, const mc_rtc::Configuration & comm_config)
: name_(name), comm_config_(comm_config), driver_loader_("ROBOT_DRIVER_PLUGIN",
                                                         driverSearchPaths(),
                                                         true,
                                                         PluginAbi{ROBOT_DRIVER_ABI_SYMBOL, ROBOT_DRIVER_ABI_VERSION})
{
  comm_ = robot_comm::CommunicationFactory::makeCommunication(name_, comm_config_);
  comm_->setupClient();
  latency_stats_ = comm_config_("latency_stats", false);
  connection_timeout_ = std::chrono::milliseconds(comm_config_("connection_timeout_ms", 100));
}

void RobotInterface::run(const std::atomic<bool> & interrupt)
{
  // Register queryable so RobotManager can bootstrap us.
  const std::string init_topic = name_ + "/init";
  comm_->handleQuery(init_topic, [this](const robot_comm::ByteBuffer & payload) -> robot_comm::ByteBuffer
                     { return handleInitQuery(payload); });

  mc_rtc::log::info("[RobotInterface] '{}' waiting for init query on '{}'", name_, init_topic);

  // Block until the init query is handled (or we are interrupted).
  // Use wait_for so the interrupt flag is checked even if init_cv_ is never
  // notified (e.g. when no manager is running and Ctrl+C is pressed).
  {
    std::unique_lock<std::mutex> lock(init_mutex_);
    while(!initialized_ && init_error_.empty() && !interrupt.load())
      init_cv_.wait_for(lock, std::chrono::milliseconds(100));
  }

  if(interrupt.load())
  {
    mc_rtc::log::warning("[RobotInterface] '{}' interrupted before init", name_);
    return;
  }

  if(!init_error_.empty())
  {
    mc_rtc::log::error("[RobotInterface] '{}' init failed: {}", name_, init_error_);
    return;
  }

  mc_rtc::log::success("[RobotInterface] '{}' initialized, starting control loop", name_);

  // Prime the last command with the robot's current position so the driver's
  // reverse interface gets servoJ right away and the URScript doesn't time out
  // while waiting for the first command from the manager.
  driver_->sync();

  last_cmd_ = makeHoldCommand();

  // Control loop: run at driver's natural rate (spin freely, driver sync() paces us).
  while(!interrupt.load())
  {
    driver_->sync();
    // Shutdown may have been requested while sync() was blocking.
    if(interrupt.load())
    {
      break;
    }

    updateSensors();
    updateControl();
  }

  mc_rtc::log::info("[RobotInterface] '{}' control loop stopped", name_);
}

robot_comm::ByteBuffer RobotInterface::handleInitQuery(const robot_comm::ByteBuffer & payload)
{
  // RobotManager may retry the init query: don't load the driver twice.
  {
    std::lock_guard<std::mutex> lock(init_mutex_);
    if(initialized_)
    {
      static const std::string ok = "OK";
      return robot_comm::ByteBuffer(ok.begin(), ok.end());
    }
  }

  // Deserialize config string from payload.
  auto config_opt =
      comm_->serializer()->deserialize<std::string>(robot_comm::MessageType::CONFIG, payload.data(), payload.size());

  if(!config_opt)
  {
    std::string err = "Failed to deserialize init config";
    mc_rtc::log::error("[RobotInterface] '{}' init query: {}", name_, err);
    std::lock_guard<std::mutex> lock(init_mutex_);
    init_error_ = err;
    init_cv_.notify_one();
    return robot_comm::ByteBuffer(err.begin(), err.end());
  }

  mc_rtc::Configuration robot_config;
  try
  {
    robot_config.loadData(*config_opt);
    control_mode_ = robot_config("controller")("mode", std::string{"position"});
  }
  catch(const std::exception & e)
  {
    std::string err = std::string{"Invalid config YAML: "} + e.what();
    mc_rtc::log::error("[RobotInterface] '{}' init query: {}", name_, err);
    std::lock_guard<std::mutex> lock(init_mutex_);
    init_error_ = err;
    init_cv_.notify_one();
    return robot_comm::ByteBuffer(err.begin(), err.end());
  }

  const std::string driver_name = robot_config("robot_interface")("driver", std::string{});
  if(driver_name.empty())
  {
    std::string err = "Missing 'robot_interface.driver' in config";
    mc_rtc::log::error("[RobotInterface] '{}' init query: {}", name_, err);
    std::lock_guard<std::mutex> lock(init_mutex_);
    init_error_ = err;
    init_cv_.notify_one();
    return robot_comm::ByteBuffer(err.begin(), err.end());
  }

  try
  {
    loadDriver(driver_name, robot_config("robot_interface"), robot_config("grippers", mc_rtc::Configuration{}));
  }
  catch(const std::exception & e)
  {
    std::string err = std::string("Driver load failed: ") + e.what();
    mc_rtc::log::error("[RobotInterface] '{}' {}", name_, err);
    std::lock_guard<std::mutex> lock(init_mutex_);
    init_error_ = err;
    init_cv_.notify_one();
    return robot_comm::ByteBuffer(err.begin(), err.end());
  }

  {
    std::lock_guard<std::mutex> lock(init_mutex_);
    initialized_ = true;
  }
  init_cv_.notify_one();

  static const std::string ok = "OK";
  return robot_comm::ByteBuffer(ok.begin(), ok.end());
}

void RobotInterface::loadDriver(const std::string & driver_name,
                                const mc_rtc::Configuration & driver_config,
                                const mc_rtc::Configuration & grippers_config)
{
  const std::string ip = driver_config("ip", std::string{"127.0.0.1"});
  const uint16_t port = driver_config("port", uint16_t{0});
  const std::string config_path = driver_config("config_path", std::string{});

  std::vector<GripperInfo> grippers;
  for(const auto & gripper_name : grippers_config.keys())
  {
    GripperInfo info;
    info.name = gripper_name;
    info.joints = grippers_config(gripper_name);
    grippers.push_back(std::move(info));
  }

  mc_rtc::log::info("[RobotInterface] '{}' loading driver '{}' ({}:{}), config_path: '{}', {} gripper(s)", name_,
                    driver_name, ip, port, config_path, grippers.size());

  if(std::getenv(DRIVER_PATH_ENV))
  {
    mc_rtc::log::info("[RobotInterface] '{}' {} is set, driver search paths: {}", name_, DRIVER_PATH_ENV,
                      fmt::join(driverSearchPaths(), ":"));
  }

  driver_ = driver_loader_.load(driver_name, ip, port, config_path, grippers);

  mc_rtc::log::success("[RobotInterface] '{}' driver '{}' loaded", name_, driver_name);
}

void RobotInterface::updateSensors()
{
  if(!driver_) return;

  robot_comm::State state;
  state.position = driver_->getActualQ();
  state.velocity = driver_->getActualQd();
  state.torque = driver_->getJointTorques();

  for(const auto & [sensorName, imu] : driver_->getIMUs())
  {
    robot_comm::BodySensorData bs;
    bs.name = sensorName;
    bs.orientation = imu.orientation;
    bs.angularVelocity = imu.angularVelocity;
    bs.linearAcceleration = imu.linearAcceleration;
    state.bodySensors.push_back(std::move(bs));
  }

  for(const auto & [sensorName, wrench] : driver_->getForceSensors())
  {
    robot_comm::ForceSensorData fs;
    fs.name = sensorName;
    fs.force = wrench.force;
    fs.torque = wrench.torque;
    state.forceSensors.push_back(std::move(fs));
  }

  if(state.position.empty()) return;

  state.stamp = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
  comm_->send(comm_->encode(state));
}

void RobotInterface::updateControl()
{
  std::chrono::steady_clock::time_point arrival;
  auto rx = comm_->receive(&arrival);
  // receive() keeps returning the latest command: only a new arrival counts
  if(rx && !rx->empty() && arrival != last_cmd_arrival_)
  {
    last_cmd_arrival_ = arrival;
    auto cmd_opt =
        comm_->serializer()->deserialize<robot_comm::Command>(robot_comm::MessageType::COMMAND, rx->data(), rx->size());
    if(cmd_opt)
    {
      if(latency_stats_) recordLatency(*cmd_opt, arrival);
      if(link_lost_)
      {
        mc_rtc::log::success("[RobotInterface] '{}' receiving commands again, resuming control", name_);
        link_lost_ = false;
      }
      last_cmd_ = std::move(cmd_opt);
    }
  }
  else if(!link_lost_)
  {
    if(const auto reason = linkLossReason(); !reason.empty())
    {
      mc_rtc::log::error("[RobotInterface] '{}' lost the manager ({}), holding the robot", name_, reason);
      link_lost_ = true;
      last_cmd_ = makeHoldCommand();
    }
  }

  if(!last_cmd_) return;

  if(!last_cmd_->position.empty())
    driver_->servoJ(last_cmd_->position);
  else if(!last_cmd_->velocity.empty())
    driver_->speedJ(last_cmd_->velocity);
  else if(!last_cmd_->torque.empty())
    driver_->tauJ(last_cmd_->torque);
}

std::optional<robot_comm::Command> RobotInterface::makeHoldCommand()
{
  const auto q = driver_->getActualQ();
  robot_comm::Command hold;

  if(control_mode_ == "position")
  {
    if(q.empty()) return std::nullopt;
    hold.position = q;
  }
  else if(control_mode_ == "velocity")
  {
    hold.velocity.assign(q.size(), 0.0);
  }
  else if(control_mode_ == "torque")
  {
    hold.torque.assign(q.size(), 0.0);
  }
  else
  {
    throw std::runtime_error("[RobotInterface] Unsupported controller mode: " + control_mode_);
  }

  return hold;
}

std::string RobotInterface::linkLossReason()
{
  if(!comm_->peerAlive()) return "manager process gone";

  // Armed once the first command arrived; until then the robot holds anyway.
  const auto age = comm_->rxAge();
  if(age && *age > connection_timeout_)
  {
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(*age).count();
    return "no command for " + std::to_string(ms) + " ms";
  }

  return {};
}

void RobotInterface::recordLatency(const robot_comm::Command & cmd, std::chrono::steady_clock::time_point arrival)
{
  // Count each echo once.
  if(cmd.stateStamp != 0 && (cmd.stateStamp != last_echo_stamp_ || cmd.stateHold != last_echo_hold_))
  {
    last_echo_stamp_ = cmd.stateStamp;
    last_echo_hold_ = cmd.stateHold;
    const auto arrival_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(arrival.time_since_epoch()).count();
    const double rtt_us =
        static_cast<double>(arrival_ns - static_cast<int64_t>(cmd.stateStamp) - static_cast<int64_t>(cmd.stateHold))
        * 1e-3;
    if(rtt_count_ == 0 || rtt_us < rtt_min_us_) rtt_min_us_ = rtt_us;
    if(rtt_count_ == 0 || rtt_us > rtt_max_us_) rtt_max_us_ = rtt_us;
    rtt_sum_us_ += rtt_us;
    ++rtt_count_;
  }

  const auto now = std::chrono::steady_clock::now();
  if(now - last_latency_report_ < std::chrono::seconds(1)) return;
  last_latency_report_ = now;
  if(rtt_count_ == 0) return;

  const double avg_us = rtt_sum_us_ / static_cast<double>(rtt_count_);
  mc_rtc::log::info("[RobotInterface] '{}' robot_comm round trip ({} samples): min {:.1f} us | avg {:.1f} us | max "
                    "{:.1f} us (one-way ~ {:.1f} us)",
                    name_, rtt_count_, rtt_min_us_, avg_us, rtt_max_us_, avg_us / 2.0);
  rtt_count_ = 0;
  rtt_sum_us_ = 0.0;
}

int runRobotInterface(const std::string & config_path, std::string name, const std::atomic<bool> & interrupt)
{
  mc_rtc::Configuration config(config_path);

  if(name.empty())
  {
    if(config.has("name"))
      name = static_cast<std::string>(config("name"));
    else
      mc_rtc::log::error_and_throw("'name' must be provided via --name or in the config file");
  }

  // Shown as "uri:<name>" in ps/top.
  const std::string thread_name = "uri:" + name;
  prctl(PR_SET_NAME, thread_name.c_str(), 0, 0, 0);

  mc_rtc::Configuration comm_config = config("network_interface");

  mc_rtc::log::info("[robot_interface] Starting '{}' from config '{}'", name, config_path);

  RobotInterface iface(name, comm_config);
  iface.run(interrupt);

  mc_rtc::log::info("[robot_interface] '{}' exited cleanly", name);
  return 0;
}

} // namespace robot_interface
