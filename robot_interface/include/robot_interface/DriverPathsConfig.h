#pragma once

#include <mc_rtc/Configuration.h>

#include <string>
#include <vector>

namespace robot_interface
{

/// Read the optional list of driver directories stored at `key` in `config`
/// (`driver_paths` in an interface config, `DriverPaths` in mc_rtc.yaml).
/// Returns an empty list when the key is absent or null. Throws std::runtime_error
/// naming `source` when the value is not a list of strings, rather than
/// silently ignoring it.
std::vector<std::string> driverPathsFromConfig(const mc_rtc::Configuration & config,
                                               const std::string & key,
                                               const std::string & source);

} // namespace robot_interface
