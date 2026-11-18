// Minimal driver plugin used by test_driver_search_paths to prove that a
// driver living outside the install prefix can be found and loaded.
#include <robot_interface/RobotDriverTemplate.h>
#include <robot_interface/driver/GripperInfo.h>
#include <robot_interface/driver/api.h>

#include <string>
#include <vector>

namespace
{
struct TestDriver : robot_interface::RobotDriver
{
  void sync() override {}
  std::vector<double> getActualQ() override
  {
    return {1.0, 2.0, 3.0};
  }
  std::vector<double> getJointTorques() override
  {
    return {};
  }
  void servoJ(const std::vector<double> &) override {}
  void speedJ(const std::vector<double> &) override {}
  void tauJ(const std::vector<double> &) override {}
};
} // namespace

ROBOT_DRIVER_EXPORT_ABI_VERSION()

extern "C"
{
  ROBOT_DRIVER_DLLAPI void ROBOT_DRIVER_PLUGIN(std::vector<std::string> & classes)
  {
    classes.push_back("SearchPathTestDriver");
  }

  ROBOT_DRIVER_DLLAPI robot_interface::RobotDriver * create(const std::string &,
                                                            const std::string &,
                                                            const uint16_t &,
                                                            const std::string &,
                                                            const std::vector<robot_interface::GripperInfo> &)
  {
    return new TestDriver();
  }

  ROBOT_DRIVER_DLLAPI void destroy(robot_interface::RobotDriver * ptr)
  {
    delete ptr;
  }
}
