#include <mc_control/GlobalPlugin.h>
#include <mc_tasks/MetaTaskLoader.h>

#include "ObserverbasedImpedanceTask.h"
#include "ObserverbasedAdmittanceTask.h"

namespace observer_tasks
{

class ObserverTasksPlugin : public mc_control::GlobalPlugin
{
public:
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override
  {
    mc_rtc::log::info("ObserverTasksPlugin init");
  }

  void reset(mc_control::MCGlobalController & controller) override
  {
    mc_rtc::log::info("ObserverTasksPlugin reset");
  }

  void before(mc_control::MCGlobalController & controller) override {}

  void after(mc_control::MCGlobalController & controller) override {}

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override
  {
    mc_control::GlobalPlugin::GlobalPluginConfiguration out;
    out.should_run_before = false;
    out.should_run_after = false;
    out.should_always_run = true;
    return out;
  }

  ~ObserverTasksPlugin() override = default;
};

} // namespace observer_tasks

EXPORT_MC_RTC_PLUGIN("ObserverTasksPlugin", observer_tasks::ObserverTasksPlugin)

// Register the tasks with the MetaTaskLoader
namespace
{
// Register ObserverbasedImpedanceTask
static auto registeredImpedance = mc_tasks::MetaTaskLoader::register_load_function(
    "ObserverbasedImpedance",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
    {
      using Allocator = Eigen::aligned_allocator<observer_tasks::ObserverbasedImpedanceTask>;
      const auto robotIndex = robotIndexFromConfig(config, solver.robots(), "ObserverbasedImpedance");
      const auto & robot = solver.robots().robot(robotIndex);
      const auto & frame = [&]() -> const mc_rbdyn::RobotFrame &
      {
        if(config.has("surface"))
        {
          mc_rtc::log::deprecated("ObserverbasedImpedance", "surface", "frame");
          return robot.frame(config("surface"));
        }
        return robot.frame(config("frame"));
      }();

      auto t =
          std::allocate_shared<observer_tasks::ObserverbasedImpedanceTask>(Allocator{}, frame, solver.controller());
      t->reset();
      t->load(solver, config);
      return t;
    });

// Register ObserverbasedAdmittanceTask
static auto registeredAdmittance = mc_tasks::MetaTaskLoader::register_load_function(
    "ObserverbasedAdmittance",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
    {
      auto frame = [&]() -> std::string
      {
        if(config.has("surface"))
        {
          mc_rtc::log::deprecated("ObserverbasedAdmittanceTaskLoader", "surface", "frame");
          return config("surface");
        }
        return config("frame");
      }();
      auto rIndex = robotIndexFromConfig(config, solver.robots(), "ObserverbasedAdmittance");
      auto t = std::make_shared<observer_tasks::ObserverbasedAdmittanceTask>(
          solver.robots().robot(rIndex).frame(frame), solver.controller(), config("stiffness", 5.0),
          config("weight", 100.0));
      t->reset();
      t->load(solver, config);
      return t;
    });
} // namespace