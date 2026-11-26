#include <robot_interface/DriverPathsConfig.h>

#include <fmt/format.h>

#include <stdexcept>

namespace robot_interface
{

std::vector<std::string> driverPathsFromConfig(const mc_rtc::Configuration & config,
                                               const std::string & key,
                                               const std::string & source)
{
  if(!config.has(key))
  {
    return {};
  }
  const auto value = config(key);
  // `driver_paths:` with every entry commented out parses as null: treat it as
  // an empty list. mc_rtc::Configuration has no isNull().
  if(value.dump() == "null")
  {
    return {};
  }
  if(!value.isArray())
  {
    throw std::runtime_error(fmt::format("'{}' in '{}' must be a list of directories", key, source));
  }
  std::vector<std::string> out;
  out.reserve(value.size());
  for(size_t i = 0; i < value.size(); ++i)
  {
    if(!value[i].isString())
    {
      throw std::runtime_error(fmt::format("'{}' in '{}': entry {} is not a string", key, source, i));
    }
    out.push_back(static_cast<std::string>(value[i]));
  }
  return out;
}

} // namespace robot_interface
