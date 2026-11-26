#pragma once

#include <atomic>
#include <string>

namespace robot_interface
{

/// Entry point of `uri viewer`: loads the driver plugin named in the config's
/// robot_interface section directly (no manager, no communication stack) and
/// prints its live joint state at display_rate_hz until interrupt is set.
/// config_path is either a robot interface config or the manager's mc_rtc.yaml;
/// for the latter, name selects the robot under 'Robots' (optional when there
/// is only one). 'driver_paths' of an interface config, or top-level
/// 'DriverPaths' of an mc_rtc.yaml, is searched for the driver before
/// URI_DRIVER_PATH. Returns the process exit code.
int runRobotStateViewer(const std::string & config_path,
                        const std::string & name,
                        double display_rate_hz,
                        const std::atomic<bool> & interrupt);

} // namespace robot_interface
