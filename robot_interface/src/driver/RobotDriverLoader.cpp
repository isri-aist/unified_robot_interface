#include <mc_rtc/loader.h>
#include <mutex>
#include <robot_interface/driver/RobotDriverLoader.h>

#include <robot_interface/config.h>

namespace robot_interface
{

std::unique_ptr<mc_rtc::ObjectLoader<mc_robot_interface::RobotDriver>> RobotDriverLoader::robot_driver_loader_;
bool RobotDriverLoader::verbose_ = false;
std::recursive_mutex RobotDriverLoader::mtx;

void RobotDriverLoader::init(bool skip_default_path)
{
  if(!robot_driver_loader_)
  {
    try
    {
      std::vector<std::string> default_path = {};
      if(!skip_default_path)
      {
        default_path.push_back(ROBOT_INTERFACE_INSTALL_PREFIX);
      }
      robot_driver_loader_.reset(new mc_rtc::ObjectLoader<RobotDriver>("ROBOT_DRIVER_PLUGIN", default_path, verbose_));
      // TODO consider aliases if needed
    }
    catch(const mc_rtc::LoaderException & e)
    {
      mc_rtc::log::error("Failed to initialize RobotDriver : {}", e.what());
      throw(e);
    }
  }
}

std::vector<std::string> RobotDriverLoader::available_interfaces()
{
  std::lock_guard<std::recursive_mutex> guard{mtx};
  init();
  auto ret = robot_driver_loader_->objects();
  return ret;
}
} // namespace robot_interface
