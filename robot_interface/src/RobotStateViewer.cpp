#include <robot_interface/RobotStateViewer.h>

#include <cstdlib>
#include <fmt/ranges.h>
#include <robot_interface/PluginLoader.h>
#include <robot_interface/RobotDriverTemplate.h>
#include <robot_interface/config.h>
#include <robot_interface/driver/DriverSearchPaths.h>
#include <robot_interface/driver/GripperInfo.h>

#include <mc_rtc/Configuration.h>
#include <mc_rtc/logging.h>

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <vector>

namespace robot_interface
{

namespace
{

// Min/avg/max of a duration series (in microseconds) over one display window.
struct TimingStats
{
  double min_us = std::numeric_limits<double>::infinity();
  double max_us = 0.0;
  double sum_us = 0.0;
  size_t count = 0;

  void add(double us)
  {
    min_us = std::min(min_us, us);
    max_us = std::max(max_us, us);
    sum_us += us;
    ++count;
  }

  double avg() const
  {
    return count ? sum_us / static_cast<double>(count) : 0.0;
  }
};

std::string formatStats(const char * label, const TimingStats & s)
{
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(1) << "  " << std::left << std::setw(14) << label << std::right;
  if(s.count == 0)
  {
    ss << "   n/a";
    return ss.str();
  }
  ss << " min " << std::setw(9) << s.min_us << " us │ avg " << std::setw(9) << s.avg() << " us │ max " << std::setw(9)
     << s.max_us << " us";
  return ss.str();
}

std::string formatRow(int idx, double pos, double vel, double torque)
{
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(4);
  ss << "  " << idx << "   │ " << std::setw(12) << pos << "   │ " << std::setw(12) << vel << "   │ " << std::setw(12)
     << torque;
  return ss.str();
}

void printState(const std::string & driver_name,
                const std::string & ip,
                const std::vector<double> & q,
                const std::vector<double> & qd,
                const std::vector<double> & tau,
                const TimingStats & sync_stats,
                const TimingStats & period_stats)
{
  // Move cursor to top-left and clear screen.
  std::cout << "\033[H\033[2J";

  std::cout << "Robot State Viewer — " << driver_name << " @ " << ip << "\n";
  std::cout << std::string(60, '-') << "\n";
  std::cout << "Joint │  Position (rad)  │  Velocity (rad/s)  │  Torque (Nm)\n";
  std::cout << "──────┼──────────────────┼────────────────────┼─────────────\n";

  const size_t n = q.size();
  for(size_t i = 0; i < n; ++i)
  {
    double vel = (i < qd.size()) ? qd[i] : 0.0;
    double tor = (i < tau.size()) ? tau[i] : 0.0;
    std::cout << formatRow(static_cast<int>(i), q[i], vel, tor) << "\n";
  }

  std::cout << std::string(60, '-') << "\n";
  const double rate_hz = period_stats.avg() > 0.0 ? 1e6 / period_stats.avg() : 0.0;
  std::cout << "Latency (" << sync_stats.count << " cycles, " << std::fixed << std::setprecision(1) << rate_hz
            << " Hz)\n";
  std::cout << formatStats("sync()", sync_stats) << "\n";
  std::cout << formatStats("cycle period", period_stats) << "\n";
  std::cout << std::string(60, '-') << "\n";
  std::cout << "Press Ctrl+C to exit.\n";
  std::cout.flush();
}

} // namespace

int runRobotStateViewer(const std::string & config_path,
                        const std::string & name,
                        double display_rate_hz,
                        const std::atomic<bool> & interrupt)
{
  const auto display_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / display_rate_hz));

  mc_rtc::Configuration config(config_path);

  if(!config.has("robot_interface") && config.has("Robots"))
  {
    auto robots = config("Robots");
    const auto robot_names = robots.keys();
    std::string robot_name = name;
    if(robot_name.empty())
    {
      if(robot_names.size() != 1)
      {
        mc_rtc::log::error_and_throw("'{}' defines {} robots under 'Robots', select one with --name", config_path,
                                     robot_names.size());
      }
      robot_name = robot_names.front();
    }
    if(!robots.has(robot_name))
    {
      mc_rtc::log::error_and_throw("No robot named '{}' under 'Robots' in '{}'", robot_name, config_path);
    }
    config = robots(robot_name);
  }
  if(!config.has("robot_interface"))
  {
    mc_rtc::log::error_and_throw("No 'robot_interface' section in '{}' (expected at top level or under 'Robots')",
                                 config_path);
  }

  const std::string driver_name = config("robot_interface")("driver", std::string{});
  if(driver_name.empty())
  {
    mc_rtc::log::error_and_throw("'driver' key missing from config '{}'", config_path);
  }
  const std::string ip = config("robot_interface")("ip", std::string{"127.0.0.1"});
  const uint16_t port = config("robot_interface")("port", uint16_t{0});
  const std::string driver_config_path = config("robot_interface")("config_path", std::string{});

  mc_rtc::log::info("[robot_state_viewer] Loading driver '{}' ({}:{})", driver_name, ip, port);

  if(std::getenv(robot_interface::DRIVER_PATH_ENV))
  {
    mc_rtc::log::info("[robot_state_viewer] {} is set, driver search paths: {}", robot_interface::DRIVER_PATH_ENV,
                      fmt::join(robot_interface::driverSearchPaths(), ":"));
  }

  robot_interface::PluginLoader<robot_interface::RobotDriver> loader(
      "ROBOT_DRIVER_PLUGIN", robot_interface::driverSearchPaths(), false,
      robot_interface::PluginAbi{ROBOT_DRIVER_ABI_SYMBOL, ROBOT_DRIVER_ABI_VERSION});
  std::shared_ptr<robot_interface::RobotDriver> driver;

  try
  {
    // Must match create()'s full signature (see RobotInterface::loadDriver):
    // the loader dispatches through one fixed function-pointer type.
    const std::vector<robot_interface::GripperInfo> grippers;
    driver = loader.load(driver_name, ip, port, driver_config_path, grippers);
  }
  catch(const std::exception & e)
  {
    mc_rtc::log::error("[robot_state_viewer] Failed to load driver: {}", e.what());
    return 1;
  }

  mc_rtc::log::info("[robot_state_viewer] Connected. Starting state display at {:.1f} Hz.", display_rate_hz);

  auto last_display = std::chrono::steady_clock::now() - display_period;
  std::chrono::steady_clock::time_point last_sync_end;
  bool has_last_sync = false;
  TimingStats sync_stats;
  TimingStats period_stats;

  using us = std::chrono::duration<double, std::micro>;

  while(!interrupt)
  {
    const auto sync_start = std::chrono::steady_clock::now();
    driver->sync();
    const auto now = std::chrono::steady_clock::now();

    sync_stats.add(us(now - sync_start).count());
    if(has_last_sync)
    {
      period_stats.add(us(now - last_sync_end).count());
    }
    last_sync_end = now;
    has_last_sync = true;

    if(now - last_display >= display_period)
    {
      last_display = now;
      printState(driver_name, ip, driver->getActualQ(), driver->getActualQd(), driver->getJointTorques(), sync_stats,
                 period_stats);
      sync_stats = {};
      period_stats = {};
    }
  }

  std::cout << "\n[robot_state_viewer] Exiting.\n";
  return 0;
}

} // namespace robot_interface
