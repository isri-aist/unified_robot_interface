#pragma once

#include <mc_rtc/loader.h>
#include <robot_interface/RobotDriverTemplate.h>

namespace mc_robot_interface
{
namespace details
{
template<typename... Args>
struct are_strings : std::true_type
{
};

template<typename T>
struct are_strings<T> : std::is_same<std::string, T>
{
};

template<typename T, typename... Args>
struct are_strings<T, Args...> : std::integral_constant<bool, are_strings<T>::value && are_strings<Args...>::value>
{
};

static_assert(are_strings<>::value, "OK");
static_assert(are_strings<std::string>::value, "OK");
static_assert(are_strings<std::string, std::string>::value, "OK");
static_assert(are_strings<std::string, std::string, std::string>::value, "OK");
static_assert(!are_strings<const char *>::value, "OK");
static_assert(!are_strings<std::string, std::string, bool>::value, "OK");

// Construct a string from a provided argument if it's not one, otherwise just return the string
template<typename T>
typename std::conditional<std::is_same<std::string, T>::value, const std::string &, std::string>::type to_string(
    const T & value)
{
  static_assert(!std::is_integral<T>::value,
                "Passing integral value here would create empty strings of the provided size");
  return value;
}

} // namespace details

/// Static loader for RobotDriver plugins. RobotInterface uses
/// PluginLoader<RobotDriver> instead.
struct MC_ROBOT_DRIVER_DLLAPI RobotDriverLoader
{
public:
  /// Create the driver registered as `name`, forwarding `args` to its create().
  template<typename... Args>
  static mc_robot_interface::RobotDriverPtr get_robot_driver(const std::string & name, const Args &... args)
  {
    if(!details::are_strings<Args...>::value)
    {
      return get_robot_driver(name, details::to_string(args...));
    }
    std::unique_lock<std::recursive_mutex> guard(mtx);
    init();
    mc_robot_interface::RobotDriverPtr rd = nullptr;
    rd = get_robot_module_from_lib(name, args...);
    return rd;
  }

  /// Names of all driver plugins found.
  static std::vector<std::string> available_interfaces();

private:
  static void init(bool skip_default_path = false);

  template<typename... Args>
  static mc_robot_interface::RobotDriverPtr get_robot_module_from_lib(const std::string & name, const Args &... args)
  {
    if(!robot_driver_loader_->has_object(name))
    {
      mc_rtc::log::error("Cannot load the requested robot: {}\nIt is neither a valid alias nor a known exported robot",
                         name);
      mc_rtc::log::info("Available Interfaces:");
      for(const auto & r : available_interfaces())
      {
        mc_rtc::log::info("- {}", r);
      }
      mc_rtc::log::error_and_throw<mc_rtc::LoaderException>("Cannot load the requested interface: {}", name);
    }
    mc_robot_interface::RobotDriverPtr ri = robot_driver_loader_->create_object(name, args...);
    if(!ri)
    {
      mc_rtc::log::error_and_throw("Failed to load {}", name);
    }

    return ri;
  }

  static std::unique_ptr<mc_rtc::ObjectLoader<mc_robot_interface::RobotDriver>> robot_driver_loader_;
  static bool verbose_;

  static std::recursive_mutex mtx;
};

} // namespace mc_robot_interface
