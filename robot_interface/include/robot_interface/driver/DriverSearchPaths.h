#pragma once

#include <string>
#include <string_view>
#include <vector>

namespace robot_interface
{

/// Name of the environment variable listing extra driver directories.
inline constexpr const char * DRIVER_PATH_ENV = "URI_DRIVER_PATH";

/// Split a ':'-separated list, dropping empty entries.
std::vector<std::string> splitSearchPath(std::string_view list);

/// Directories to search for driver plugins, in priority order:
/// `extra`, then the entries of URI_DRIVER_PATH, then ROBOT_INTERFACE_INSTALL_PREFIX.
/// Duplicates are removed, keeping the first occurrence. Paths are used as given
/// (no `~` expansion); missing directories are skipped by the plugin loader.
std::vector<std::string> driverSearchPaths(const std::vector<std::string> & extra = {});

} // namespace robot_interface
