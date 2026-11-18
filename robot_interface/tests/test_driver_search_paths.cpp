#include <robot_interface/PluginLoader.h>
#include <robot_interface/RobotDriverTemplate.h>
#include <robot_interface/config.h>
#include <robot_interface/driver/DriverSearchPaths.h>
#include <robot_interface/driver/GripperInfo.h>

#include <gtest/gtest.h>

#include <cstdlib>

using namespace robot_interface;

namespace
{
const std::string prefix = ROBOT_INTERFACE_INSTALL_PREFIX;

struct EnvGuard
{
  EnvGuard()
  {
    unsetenv(DRIVER_PATH_ENV);
  }
  ~EnvGuard()
  {
    unsetenv(DRIVER_PATH_ENV);
  }
  void set(const std::string & v)
  {
    setenv(DRIVER_PATH_ENV, v.c_str(), 1);
  }
};
} // namespace

using Paths = std::vector<std::string>;

TEST(DriverSearchPaths, SplitDropsEmptyEntries)
{
  EXPECT_EQ(splitSearchPath(""), Paths{});
  EXPECT_EQ(splitSearchPath("a"), (Paths{"a"}));
  EXPECT_EQ(splitSearchPath("a:b"), (Paths{"a", "b"}));
  EXPECT_EQ(splitSearchPath("a::b:"), (Paths{"a", "b"}));
  EXPECT_EQ(splitSearchPath(":::"), Paths{});
}

TEST(DriverSearchPaths, UnsetKeepsOnlyInstallPrefix)
{
  EnvGuard env;
  EXPECT_EQ(driverSearchPaths(), (Paths{prefix}));
}

TEST(DriverSearchPaths, EmptyVariableKeepsOnlyInstallPrefix)
{
  EnvGuard env;
  env.set("");
  EXPECT_EQ(driverSearchPaths(), (Paths{prefix}));
}

TEST(DriverSearchPaths, EnvDirectoriesComeBeforePrefix)
{
  EnvGuard env;
  env.set("/x::/y:");
  EXPECT_EQ(driverSearchPaths(), (Paths{"/x", "/y", prefix}));
}

TEST(DriverSearchPaths, DuplicatesRemovedKeepingFirst)
{
  EnvGuard env;
  env.set("/x:/y:/x:" + prefix);
  EXPECT_EQ(driverSearchPaths(), (Paths{"/x", "/y", prefix}));
}

TEST(DriverSearchPaths, ExtraComesFirst)
{
  EnvGuard env;
  env.set("/z");
  EXPECT_EQ(driverSearchPaths({"/e", "/z"}), (Paths{"/e", "/z", prefix}));
}

TEST(DriverSearchPaths, DriverOutsideInstallPrefixLoadsThroughEnv)
{
  EnvGuard env;
  env.set(TEST_DRIVER_DIR);
  PluginLoader<RobotDriver> loader("ROBOT_DRIVER_PLUGIN", driverSearchPaths(), false,
                                   PluginAbi{ROBOT_DRIVER_ABI_SYMBOL, ROBOT_DRIVER_ABI_VERSION});
  const std::vector<GripperInfo> grippers;
  auto driver = loader.load("SearchPathTestDriver", std::string{"127.0.0.1"}, uint16_t{0}, std::string{}, grippers);
  ASSERT_NE(driver, nullptr);
  EXPECT_EQ(driver->getActualQ(), (std::vector<double>{1.0, 2.0, 3.0}));
}
