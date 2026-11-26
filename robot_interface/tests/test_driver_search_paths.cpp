#include <robot_interface/DriverPathsConfig.h>
#include <robot_interface/PluginLoader.h>
#include <robot_interface/RobotDriverTemplate.h>
#include <robot_interface/RobotStateViewer.h>
#include <robot_interface/config.h>
#include <robot_interface/driver/DriverSearchPaths.h>
#include <robot_interface/driver/GripperInfo.h>

#include <gtest/gtest.h>

#include <atomic>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>

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

// YAML file in the test temp dir, removed on destruction.
struct TempYaml
{
  explicit TempYaml(const std::string & name, const std::string & content)
  : path(std::filesystem::path(::testing::TempDir()) / name)
  {
    std::ofstream(path) << content;
  }
  ~TempYaml()
  {
    std::filesystem::remove(path);
  }
  std::filesystem::path path;
};

// getActualQ() of the test driver loaded from `paths`; the loader (which owns
// the library) outlives the driver. Throws if the driver is not found.
std::vector<double> loadTestDriverQ(const std::vector<std::string> & paths)
{
  PluginLoader<RobotDriver> loader("ROBOT_DRIVER_PLUGIN", paths, false,
                                   PluginAbi{ROBOT_DRIVER_ABI_SYMBOL, ROBOT_DRIVER_ABI_VERSION});
  const std::vector<GripperInfo> grippers;
  auto driver = loader.load("SearchPathTestDriver", std::string{"127.0.0.1"}, uint16_t{0}, std::string{}, grippers);
  if(!driver)
  {
    throw std::runtime_error("SearchPathTestDriver not loaded");
  }
  return driver->getActualQ();
}

// Runs `uri viewer` once: with interrupt already set it returns 0 right after
// the driver loads, 1 if the driver is not found.
int runViewerOnce(const std::string & config_path)
{
  const std::atomic<bool> interrupt{true};
  return runRobotStateViewer(config_path, "", 10.0, interrupt);
}

const std::string viewer_robot_interface = "  driver: SearchPathTestDriver\n"
                                           "  ip: 127.0.0.1\n"
                                           "  port: 0\n";
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

TEST(DriverSearchPaths, AllSourcesInOrder)
{
  EnvGuard env;
  env.set("/z");
  EXPECT_EQ(driverSearchPaths({"/x", "/y"}), (Paths{"/x", "/y", "/z", prefix}));
}

TEST(DriverSearchPaths, DuplicatesAcrossSourcesRemovedKeepingFirst)
{
  EnvGuard env;
  env.set("/y:/x:" + prefix);
  EXPECT_EQ(driverSearchPaths({"/x", "/y", "/x", prefix}), (Paths{"/x", "/y", prefix}));
}

TEST(DriverSearchPaths, DriverOutsideInstallPrefixLoadsThroughExtra)
{
  EnvGuard env;
  EXPECT_EQ(loadTestDriverQ(driverSearchPaths({TEST_DRIVER_DIR})), (std::vector<double>{1.0, 2.0, 3.0}));
}

TEST(DriverSearchPaths, EarlierDirectoryWinsForSameClassName)
{
  // Both directories hold a "SearchPathTestDriver"; copy A reports q0 = 1,
  // copy B q0 = 10. Swapping the sources swaps the winner, so the order decides.
  EnvGuard env;
  env.set(TEST_DRIVER_DIR);
  EXPECT_EQ(loadTestDriverQ(driverSearchPaths({TEST_DRIVER_B_DIR})), (std::vector<double>{10.0, 2.0, 3.0}));
  env.set(TEST_DRIVER_B_DIR);
  EXPECT_EQ(loadTestDriverQ(driverSearchPaths({TEST_DRIVER_DIR})), (std::vector<double>{1.0, 2.0, 3.0}));
}

TEST(DriverPathsConfig, AbsentKeyGivesNoPaths)
{
  EXPECT_EQ(driverPathsFromConfig(mc_rtc::Configuration::fromYAMLData("name: r"), "driver_paths", "test"), Paths{});
}

TEST(DriverPathsConfig, NullKeyGivesNoPaths)
{
  const auto config = mc_rtc::Configuration::fromYAMLData("driver_paths:\n  # - /opt/drivers\nname: r");
  EXPECT_EQ(driverPathsFromConfig(config, "driver_paths", "test"), Paths{});
}

TEST(DriverPathsConfig, ListIsReadInOrder)
{
  const auto config = mc_rtc::Configuration::fromYAMLData("driver_paths: [/b, /a]");
  EXPECT_EQ(driverPathsFromConfig(config, "driver_paths", "test"), (Paths{"/b", "/a"}));
}

TEST(DriverPathsConfig, NonListIsRejected)
{
  const auto config = mc_rtc::Configuration::fromYAMLData("driver_paths: /opt/drivers");
  EXPECT_THROW(driverPathsFromConfig(config, "driver_paths", "test"), std::runtime_error);
}

TEST(DriverPathsConfig, NonStringEntryIsRejected)
{
  const auto config = mc_rtc::Configuration::fromYAMLData("driver_paths: [/a, {b: 1}]");
  EXPECT_THROW(driverPathsFromConfig(config, "driver_paths", "test"), std::runtime_error);
}

TEST(DriverSearchPathsViewer, DriverNotFoundWithoutExtraPaths)
{
  EnvGuard env;
  TempYaml yaml("viewer_no_paths.yaml", "robot_interface:\n" + viewer_robot_interface);
  EXPECT_EQ(runViewerOnce(yaml.path.string()), 1);
}

TEST(DriverSearchPathsViewer, LoadsThroughInterfaceConfigDriverPaths)
{
  EnvGuard env;
  TempYaml yaml("viewer_iface.yaml",
                std::string{"driver_paths: ["} + TEST_DRIVER_DIR + "]\nrobot_interface:\n" + viewer_robot_interface);
  EXPECT_EQ(runViewerOnce(yaml.path.string()), 0);
}

TEST(DriverSearchPathsViewer, LoadsThroughManagerDriverPaths)
{
  EnvGuard env;
  std::string robot_interface;
  for(const auto & line : {"driver: SearchPathTestDriver", "ip: 127.0.0.1", "port: 0"})
  {
    robot_interface += std::string{"      "} + line + "\n";
  }
  TempYaml yaml("viewer_manager.yaml", std::string{"DriverPaths: ["} + TEST_DRIVER_DIR
                                           + "]\nRobots:\n  r:\n    robot_interface:\n" + robot_interface);
  EXPECT_EQ(runViewerOnce(yaml.path.string()), 0);
}

TEST(DriverSearchPathsViewer, InvalidDriverPathsIsAnError)
{
  EnvGuard env;
  TempYaml yaml("viewer_invalid.yaml",
                std::string{"driver_paths: "} + TEST_DRIVER_DIR + "\nrobot_interface:\n" + viewer_robot_interface);
  EXPECT_THROW(runViewerOnce(yaml.path.string()), std::runtime_error);
}
