#include <robot_interface/driver/DriverSearchPaths.h>

#include <robot_interface/config.h>

#include <algorithm>
#include <cstdlib>

namespace robot_interface
{

std::vector<std::string> splitSearchPath(std::string_view list)
{
  std::vector<std::string> out;
  while(!list.empty())
  {
    const auto pos = list.find(':');
    const auto entry = list.substr(0, pos);
    if(!entry.empty())
    {
      out.emplace_back(entry);
    }
    if(pos == std::string_view::npos)
    {
      break;
    }
    list.remove_prefix(pos + 1);
  }
  return out;
}

std::vector<std::string> driverSearchPaths(const std::vector<std::string> & extra)
{
  std::vector<std::string> out;
  const auto add = [&out](const std::string & p)
  {
    if(std::find(out.begin(), out.end(), p) == out.end())
    {
      out.push_back(p);
    }
  };
  for(const auto & p : extra)
  {
    add(p);
  }
  // Deliberately getenv, not secure_getenv: user paths are honoured even though
  // `uri` carries cap_sys_nice.
  if(const char * env = std::getenv(DRIVER_PATH_ENV))
  {
    for(const auto & p : splitSearchPath(env))
    {
      add(p);
    }
  }
  add(ROBOT_INTERFACE_INSTALL_PREFIX);
  return out;
}

} // namespace robot_interface
