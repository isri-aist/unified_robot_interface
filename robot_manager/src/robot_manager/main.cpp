#include "CreateNewDriverScript.h"
#include <robot_interface/RobotInterface.h>
#include <robot_interface/RobotStateViewer.h>
#include <robot_manager/RobotManager.h>

#include <CLI/CLI.hpp>

#include <fmt/core.h>
// Resolve redefinition conflict with pthread.h
#define sched_param linux_sched_param // NOLINT(readability-identifier-naming)
#include <linux/sched.h>
#include <linux/sched/types.h>
#undef sched_param

#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <sys/mman.h>
#include <sys/types.h>
#include <syscall.h>
#include <unistd.h>
#include <vector>

int schedSetattr(pid_t pid, const struct sched_attr * attr, unsigned int flags)
{
  return static_cast<int>(syscall(__NR_sched_setattr, pid, attr, flags)); // NOLINT(cppcoreguidelines-pro-type-vararg)
}

namespace
{
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
// Use atomic to prevent cached reading
std::atomic<bool> interrupt{false};
} // namespace

void signalHandler(int s)
{
  fmt::print("[warning] Caught signal {}\n", s);
  interrupt = true;
}

int main(int argc, char * argv[])
{
  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);

  CLI::App app{"uri: unified robot interface"};
  app.require_subcommand(1);

  std::string manager_config_path;
  auto * manager_cmd = app.add_subcommand("manager", "Run the robot manager: mc_rtc controller + one proxy per robot");
  manager_cmd->add_option("-c,--config", manager_config_path, "Path to mc_rtc configuration file")->required();
  manager_cmd->footer("see etc/mc_rtc.yaml for example configuration");

  std::string interface_config_path;
  std::string interface_name;
  auto * interface_cmd =
      app.add_subcommand("interface", "Run the robot-side interface of one robot (loads its driver plugin)");
  interface_cmd->add_option("-c,--config", interface_config_path, "Path to robot interface config file (YAML)")
      ->required();
  interface_cmd->add_option("-n,--name", interface_name, "Robot name (overrides 'name' in config)");

  std::string viewer_config_path;
  std::string viewer_name;
  double viewer_rate_hz = 10.0;
  auto * viewer_cmd = app.add_subcommand(
      "viewer", "Load a driver plugin directly and display its live joint state (no manager, no network)");
  viewer_cmd
      ->add_option("-c,--config", viewer_config_path,
                   "Path to robot interface config file, or the manager's mc_rtc.yaml (YAML)")
      ->required();
  viewer_cmd->add_option("-n,--name", viewer_name,
                         "Robot to view under 'Robots' when --config is an mc_rtc.yaml with several robots");
  viewer_cmd->add_option("-r,--rate", viewer_rate_hz, "Display refresh rate in Hz")->default_val(10.0);

  std::string driver_name;
  std::string driver_output_dir;
  bool driver_no_git = false;
  auto * create_driver_cmd = app.add_subcommand("create_new_driver", "Scaffold a new robot driver plugin project");
  create_driver_cmd
      ->add_option("name", driver_name,
                   "PascalCase driver name, e.g. Franka (generates <name>_driver/ with class RobotDriver<Name>)")
      ->required();
  create_driver_cmd->add_option("folder", driver_output_dir,
                                "Directory in which to create the project (default: current directory)");
  create_driver_cmd->add_flag("--no-git", driver_no_git, "Skip initializing a git repository in the project");

  CLI11_PARSE(app, argc, argv);

  if(*create_driver_cmd)
  {
    std::vector<const char *> args{"bash", "-c", CREATE_NEW_DRIVER_SCRIPT, "uri create_new_driver"};
    if(driver_no_git)
    {
      args.push_back("--no-git");
    }
    args.push_back(driver_name.c_str());
    if(!driver_output_dir.empty())
    {
      args.push_back(driver_output_dir.c_str());
    }
    args.push_back(nullptr);
    execvp("bash", const_cast<char * const *>(args.data())); // NOLINT(cppcoreguidelines-pro-type-const-cast)
    fmt::print(stderr, "[error] Failed to run bash: {}\n", std::strerror(errno));
    return 1;
  }

  // Report bad configs etc. as an error message rather than an abort.
  try
  {
    if(*interface_cmd)
    {
      return robot_interface::runRobotInterface(interface_config_path, interface_name, interrupt);
    }
    if(*viewer_cmd)
    {
      return robot_interface::runRobotStateViewer(viewer_config_path, viewer_name, viewer_rate_hz, interrupt);
    }
  }
  catch(const std::exception & e)
  {
    fmt::print(stderr, "[error] {}\n", e.what());
    return 1;
  }

  /* Lock Memory */
  if(mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
  {
    fmt::print("[error] mlockall failed: {}\n", std::strerror(errno));
    if(errno == ENOMEM)
    {
      fmt::print("Check /etc/security/limits.conf for memlock limits.\n");
    }
    return -2;
  }

  uint64_t cycle_ns{1000UL * 1000UL}; // 1 ms default cycle
  const char * mc_rt_freq = getenv("MC_RT_FREQ");
  if(mc_rt_freq != nullptr)
  {
    cycle_ns = static_cast<uint64_t>(atoi(mc_rt_freq)) * 1000UL * 1000UL;
  }

  /* Initialize callback (non real-time yet) */
  void * raw = nullptr;
  try
  {
    raw = robot_manager::init(manager_config_path, cycle_ns, interrupt);
  }
  catch(const std::exception & e)
  {
    fmt::print(stderr, "[error] {}\n", e.what());
  }
  if(raw == nullptr)
  {
    fmt::print("[error] Initialization failed\n");
    return -2;
  }

  // Automatically free data if schedSetattr fails
  std::unique_ptr<robot_manager::RobotManager> robot_manager{static_cast<robot_manager::RobotManager *>(raw)};

  /* Time reservation */
  struct sched_attr attr = {};
  memset(&attr, 0, sizeof(attr));
  attr.size = sizeof(attr);
  attr.sched_policy = SCHED_DEADLINE;
  attr.sched_runtime = attr.sched_deadline = attr.sched_period = cycle_ns; // nanoseconds

  fmt::print("Running thread at {}ms per cycle\n", double(cycle_ns) / 1e6);

  /* Set scheduler policy for the main thread */
  if(schedSetattr(0, &attr, 0) < 0)
  {
    // Keep running (e.g. under gdb, which drops file capabilities), but make
    // the loss of real-time scheduling obvious.
    const int err = errno;
    fmt::print(stderr, "[error] SCHED_DEADLINE setup failed: {}. Running WITHOUT real-time scheduling.\n",
               std::strerror(err));
    if(err == EPERM)
    {
      fmt::print(stderr, "        uri needs the cap_sys_nice capability: sudo setcap cap_sys_nice+eip $(which uri)\n"
                         "        (check with: getcap $(which uri); in Docker also pass --cap-add=sys_nice)\n");
    }
    else if(err == EBUSY)
    {
      fmt::print(stderr, "        Not enough SCHED_DEADLINE bandwidth for a {}ms runtime per period.\n",
                 double(cycle_ns) / 1e6);
    }
  }

  /* Run */
  robot_manager::run(robot_manager.get(), interrupt);

  return 0;
}
