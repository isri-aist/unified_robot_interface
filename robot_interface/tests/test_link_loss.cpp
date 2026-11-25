#include <robot_interface/RobotInterface.h>

#include <gtest/gtest.h>

#include <robot_comm/CommunicationFactory.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace robot_comm;
using namespace robot_interface;
using namespace std::chrono_literals;

namespace
{

using Joints = std::vector<double>;

const Joints actual_q = {0.1, 0.2, 0.3};
const Joints zeros = {0.0, 0.0, 0.0};
const Joints first_command = {1.0, 2.0, 3.0};
const Joints second_command = {4.0, 5.0, 6.0};

// One call made to the fake driver.
struct Call
{
  std::string name;
  Joints values;
};

// What the fake driver saw; outlives it.
class Probe
{
public:
  std::atomic<bool> throw_in_sync{false};
  std::atomic<int> syncs{0};

  void record(const std::string & name, const Joints & values = {})
  {
    std::lock_guard<std::mutex> lock(mutex_);
    calls_.push_back({name, values});
  }

  std::vector<Call> calls() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return calls_;
  }

  long count(const std::string & name) const
  {
    const auto seen = calls();
    return std::count_if(seen.begin(), seen.end(), [&](const Call & call) { return call.name == name; });
  }

  bool lastIs(const std::string & name, const Joints & values) const
  {
    const auto seen = calls();
    return !seen.empty() && seen.back().name == name && seen.back().values == values;
  }

private:
  mutable std::mutex mutex_;
  std::vector<Call> calls_;
};

// Driver that keeps the default stop().
class FakeDriver : public RobotDriver
{
public:
  explicit FakeDriver(std::shared_ptr<Probe> probe) : probe_(std::move(probe)) {}

  ~FakeDriver() override
  {
    probe_->record("destroy");
  }

  void sync() override
  {
    std::this_thread::sleep_for(1ms);
    ++probe_->syncs;
    if(probe_->throw_in_sync) throw std::runtime_error("sync failed");
  }

  Joints getActualQ() override
  {
    return actual_q;
  }

  Joints getJointTorques() override
  {
    return zeros;
  }

  void servoJ(const Joints & q) override
  {
    probe_->record("servoJ", q);
  }

  void speedJ(const Joints & alpha) override
  {
    probe_->record("speedJ", alpha);
  }

  void tauJ(const Joints & tau) override
  {
    probe_->record("tauJ", tau);
  }

protected:
  std::shared_ptr<Probe> probe_;
};

class StoppableDriver : public FakeDriver
{
public:
  using FakeDriver::FakeDriver;

  void stop() override
  {
    probe_->record("stop");
  }
};

class FailingStopDriver : public FakeDriver
{
public:
  using FakeDriver::FakeDriver;

  void stop() override
  {
    probe_->record("stop");
    throw std::runtime_error("stop failed");
  }
};

// Poll `condition` until it holds or `timeout` expires.
bool waitFor(const std::function<bool()> & condition, std::chrono::milliseconds timeout = 3s)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while(std::chrono::steady_clock::now() < deadline)
  {
    if(condition()) return true;
    std::this_thread::sleep_for(5ms);
  }
  return condition();
}

mc_rtc::Configuration networkConfig(int connection_timeout_ms)
{
  mc_rtc::Configuration config;
  config.add("protocol", std::string{"zenoh"});
  config.add("connection_timeout_ms", connection_timeout_ms);
  return config;
}

// A RobotInterface running in its own thread, and the manager side of its link.
class Rig
{
public:
  template<typename Driver>
  static std::unique_ptr<Rig> make(const std::string & mode, int connection_timeout_ms = 50)
  {
    auto rig = std::unique_ptr<Rig>(new Rig());
    // Every test needs its own topics: ctest may run them in parallel
    const auto name = std::string{"link_loss_"} + testing::UnitTest::GetInstance()->current_test_info()->name();
    const auto network = networkConfig(connection_timeout_ms);

    rig->interface_ = std::make_unique<RobotInterface>(
        name, network,
        [probe = rig->probe](const std::string &, const std::string &, uint16_t, const std::string &,
                             const std::vector<GripperInfo> &) { return std::make_shared<Driver>(probe); });
    rig->result = std::async(std::launch::async, [raw = rig.get()] { return raw->interface_->run(raw->interrupt); });

    rig->manager = CommunicationFactory::makeCommunication(name, network);
    rig->manager->setupServer();

    mc_rtc::Configuration robot_config;
    robot_config.add("controller").add("mode", mode);
    robot_config.add("robot_interface").add("driver", std::string{"FakeDriver"});
    const auto payload = rig->manager->serializer()->serialize(robot_config.dump());

    // The first queries may be lost while Zenoh discovers the peer
    const bool initialized = waitFor(
        [&]
        {
          const auto reply = rig->manager->query(name + "/init", payload, 500ms);
          return reply && std::string(reply->begin(), reply->end()) == "OK";
        });
    if(!initialized) throw std::runtime_error("RobotInterface did not initialize");
    return rig;
  }

  ~Rig()
  {
    interrupt = true;
    if(result.valid()) result.wait();
  }

  // Send `command` until the driver received it.
  bool sendUntilApplied(const Command & command, const std::string & call, const Joints & values)
  {
    return waitFor(
        [&]
        {
          manager->send(manager->encode(command));
          const auto seen = probe->calls();
          return std::any_of(seen.begin(), seen.end(),
                             [&](const Call & c) { return c.name == call && c.values == values; });
        });
  }

  bool finished(std::chrono::milliseconds timeout = 3s)
  {
    return result.wait_for(timeout) == std::future_status::ready;
  }

  std::shared_ptr<Probe> probe = std::make_shared<Probe>();
  std::atomic<bool> interrupt{false};
  std::unique_ptr<Communication> manager;
  std::future<bool> result;

private:
  Rig() = default;

  std::unique_ptr<RobotInterface> interface_;
};

Command torqueCommand()
{
  Command command;
  command.torque = first_command;
  return command;
}

// The only torque ever applied is the manager's, and the calls end with `ending`.
void expectStopped(const Probe & probe, const std::vector<std::string> & ending)
{
  const auto calls = probe.calls();
  ASSERT_GE(calls.size(), ending.size());
  const auto first = calls.size() - ending.size();
  for(size_t i = 0; i < ending.size(); ++i) EXPECT_EQ(calls[first + i].name, ending[i]);
  for(size_t i = 0; i < first; ++i)
  {
    EXPECT_EQ(calls[i].name, "tauJ");
    EXPECT_EQ(calls[i].values, first_command);
  }
}

} // namespace

// ─── Torque mode ─────────────────────────────────────────────────────────────

TEST(LinkLoss, TorqueStopsWhenManagerGoesSilent)
{
  auto rig = Rig::make<StoppableDriver>("torque");
  ASSERT_TRUE(rig->sendUntilApplied(torqueCommand(), "tauJ", first_command));

  ASSERT_TRUE(rig->finished());
  EXPECT_FALSE(rig->result.get());
  expectStopped(*rig->probe, {"stop", "destroy"});
}

TEST(LinkLoss, TorqueStopsWhenManagerIsGone)
{
  // Watchdog out of the picture: only liveliness can notice
  auto rig = Rig::make<StoppableDriver>("torque", 60000);
  ASSERT_TRUE(rig->sendUntilApplied(torqueCommand(), "tauJ", first_command));

  rig->manager.reset();

  ASSERT_TRUE(rig->finished());
  EXPECT_FALSE(rig->result.get());
  expectStopped(*rig->probe, {"stop", "destroy"});
}

TEST(LinkLoss, TorqueSendsNothingBeforeFirstCommand)
{
  auto rig = Rig::make<StoppableDriver>("torque");
  ASSERT_TRUE(waitFor([&] { return rig->probe->syncs > 100; }));

  EXPECT_TRUE(rig->probe->calls().empty());
}

TEST(LinkLoss, TorqueWithDefaultStopDestroysDriver)
{
  auto rig = Rig::make<FakeDriver>("torque");
  ASSERT_TRUE(rig->sendUntilApplied(torqueCommand(), "tauJ", first_command));

  ASSERT_TRUE(rig->finished());
  EXPECT_FALSE(rig->result.get());
  expectStopped(*rig->probe, {"destroy"});
}

TEST(LinkLoss, TorqueDestroysDriverWhenStopThrows)
{
  auto rig = Rig::make<FailingStopDriver>("torque");
  ASSERT_TRUE(rig->sendUntilApplied(torqueCommand(), "tauJ", first_command));

  ASSERT_TRUE(rig->finished());
  EXPECT_FALSE(rig->result.get());
  expectStopped(*rig->probe, {"stop", "destroy"});
}

TEST(LinkLoss, TorqueStopsWhenSyncThrows)
{
  auto rig = Rig::make<StoppableDriver>("torque", 60000);
  ASSERT_TRUE(rig->sendUntilApplied(torqueCommand(), "tauJ", first_command));

  rig->probe->throw_in_sync = true;

  ASSERT_TRUE(rig->finished());
  EXPECT_THROW(rig->result.get(), std::runtime_error);
  expectStopped(*rig->probe, {"stop", "destroy"});
}

TEST(LinkLoss, TorqueStopsOnInterrupt)
{
  auto rig = Rig::make<StoppableDriver>("torque", 60000);
  ASSERT_TRUE(rig->sendUntilApplied(torqueCommand(), "tauJ", first_command));

  rig->interrupt = true;

  ASSERT_TRUE(rig->finished());
  EXPECT_TRUE(rig->result.get());
  expectStopped(*rig->probe, {"stop", "destroy"});
}

// ─── Position and velocity modes ─────────────────────────────────────────────

TEST(LinkLoss, PositionHoldsAndResumes)
{
  auto rig = Rig::make<StoppableDriver>("position");
  Command command;
  command.position = first_command;
  ASSERT_TRUE(rig->sendUntilApplied(command, "servoJ", first_command));

  EXPECT_TRUE(waitFor([&] { return rig->probe->lastIs("servoJ", actual_q); })) << "No hold after the link loss";

  command.position = second_command;
  EXPECT_TRUE(rig->sendUntilApplied(command, "servoJ", second_command)) << "Did not resume";

  EXPECT_FALSE(rig->finished(0ms));
  rig->interrupt = true;
  ASSERT_TRUE(rig->finished());
  EXPECT_TRUE(rig->result.get());
  EXPECT_EQ(rig->probe->count("stop"), 0);
  EXPECT_EQ(rig->probe->count("destroy"), 1);
}

TEST(LinkLoss, VelocityHoldsAndResumes)
{
  auto rig = Rig::make<StoppableDriver>("velocity");
  Command command;
  command.velocity = first_command;
  ASSERT_TRUE(rig->sendUntilApplied(command, "speedJ", first_command));

  EXPECT_TRUE(waitFor([&] { return rig->probe->lastIs("speedJ", zeros); })) << "No hold after the link loss";

  command.velocity = second_command;
  EXPECT_TRUE(rig->sendUntilApplied(command, "speedJ", second_command)) << "Did not resume";

  EXPECT_FALSE(rig->finished(0ms));
  rig->interrupt = true;
  ASSERT_TRUE(rig->finished());
  EXPECT_TRUE(rig->result.get());
  EXPECT_EQ(rig->probe->count("stop"), 0);
  EXPECT_EQ(rig->probe->count("destroy"), 1);
}
