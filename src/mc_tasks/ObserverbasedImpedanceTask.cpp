#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/ObserverbasedImpedanceTask.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include "mc_rbdyn/ForceSensor.h"
#include "mc_rbdyn/RobotModule.h"
#include "mc_rtc/Configuration.h"
#include "mc_rtc/gui/plot/types.h"
#include "mc_rtc/log/Logger.h"
#include "mc_rtc/logging.h"

using Color = mc_rtc::gui::Color;
namespace mc_tasks
{
namespace force
{

ObserverbasedImpedanceTask::ObserverbasedImpedanceTask(const std::string & surfaceName,
                                                       const mc_rbdyn::Robots & robots,
                                                       mc_control::MCController * controller,
                                                       unsigned robotIndex,
                                                       double stiffness,
                                                       double weight)
: ObserverbasedImpedanceTask(robots.robot(robotIndex).frame(surfaceName), controller, stiffness, weight)
{
}

ObserverbasedImpedanceTask::ObserverbasedImpedanceTask(const mc_rbdyn::RobotFrame & frame,
                                                       mc_control::MCController * controller,
                                                       double stiffness,
                                                       double weight)
: ImpedanceTask(frame, stiffness, weight), controller_(controller)
{
  type_ = "ObserverbasedImpedanceTask";
  name_ = "ObserverbasedImpedance_" + robots.robot(rIndex).name() + "_" + frame.name();
  mc_rtc::log::info("[ObserverbasedImpedanceTask] Initialized!");
}

void ObserverbasedImpedanceTask::update(mc_solver::QPSolver & solver)
{
  double dt = solver.dt();
  // 1. Filter the estimated wrench
  if(frame_->hasForceSensor()) { surfaceWrench_ = frame_->forceSensor().wrenchWithoutGravity(robots.robot(rIndex)); }
  else { surfaceWrench_ = sva::ForceVecd::Zero(); }

  // choose the wrench to use
  if(usingWrench_ == "Contact")
  {
    getestimatedContactWrench(surface());
    measuredWrench_ = transformContactWrench(estimatedContactWrench_, surface(), frame_->forceSensor().name());
    estimationError_ = surfaceWrench_ - estimatedContactWrench_;
  }
  else if(usingWrench_ == "External")
  {
    getestimatedExternalWrench();
    measuredWrench_ = transformExternalWrench(estimatedExternalWrench_centroid_, surface());
    estimationError_ = surfaceWrench_ - measuredWrench_;
  }
  else if(usingWrench_ == "None") { measuredWrench_ = targetWrench_; } // should be fixed
  else if(usingWrench_ == "Sensor") { measuredWrench_ = frame_->wrench(); }
  else
  {
    mc_rtc::log::error_and_throw("[ObserverbasedImpedanceTask] usingWrench_ is not defined correctly: {}",
                                 usingWrench_);
  }
  wrenchError_ = measuredWrench_ - targetWrench_;

  lowPass_.update(measuredWrench_);
  filteredMeasuredWrench_ = lowPass_.eval();

  sva::MotionVecd deltaCompVelWPrev = deltaCompVelW_;
  sva::PTransformd T_0_s(surfacePose().rotation());
  deltaCompVelW_ = T_0_s.invMul(sva::MotionVecd(gains().D().vector().cwiseInverse().cwiseProduct(
      -gains().K().vector().cwiseProduct((T_0_s * sva::transformVelocity(deltaCompPoseW_)).vector())
      + gains().wrench().vector().cwiseProduct((filteredMeasuredWrench_ - targetWrench_).vector()))));
  deltaCompAccelW_ = (deltaCompVelW_ - deltaCompVelWPrev) / dt;

  if(deltaCompAccelW_.linear().norm() > deltaCompAccelLinLimit_)
  {
    mc_rtc::log::warning("[ObserverbasedImpedanceTask] Linear deltaCompAccel limited from {} to {}",
                         deltaCompAccelW_.linear().norm(), deltaCompAccelLinLimit_);
    deltaCompAccelW_.linear().normalize();
    deltaCompAccelW_.linear() *= deltaCompAccelLinLimit_;
  }
  if(deltaCompAccelW_.angular().norm() > deltaCompAccelAngLimit_)
  {
    mc_rtc::log::warning("[ObserverbasedImpedanceTask] Angular deltaCompAccel limited from {} to {}",
                         deltaCompAccelW_.angular().norm(), deltaCompAccelAngLimit_);
    deltaCompAccelW_.angular().normalize();
    deltaCompAccelW_.angular() *= deltaCompAccelAngLimit_;
  }

  // 3. Compute the compliance pose and velocity by time integral
  // 3.1 Integrate velocity to pose
  sva::PTransformd T_0_deltaC(deltaCompPoseW_.rotation());
  // Represent the compliance velocity and acceleration in the deltaCompliance frame and scale by dt
  sva::MotionVecd mvDeltaCompVelIntegralC = T_0_deltaC * (dt * deltaCompVelW_);
  // sva::MotionVecd mvDeltaCompVelIntegralC = T_0_deltaC * (dt * (deltaCompVelW_ + 0.5 * dt * deltaCompAccelW_));
  // Convert the angular velocity to the rotation matrix through AngleAxis representation
  Eigen::AngleAxisd aaDeltaCompVelIntegralC(Eigen::Quaterniond::Identity());
  if(mvDeltaCompVelIntegralC.angular().norm() > 1e-6)
  {
    aaDeltaCompVelIntegralC =
        Eigen::AngleAxisd(mvDeltaCompVelIntegralC.angular().norm(), mvDeltaCompVelIntegralC.angular().normalized());
  }
  sva::PTransformd deltaCompVelIntegral(
      // Rotation matrix is transposed because sva::PTransformd uses the left-handed coordinates
      aaDeltaCompVelIntegralC.toRotationMatrix().transpose(), mvDeltaCompVelIntegralC.linear());
  // Since deltaCompVelIntegral is multiplied by deltaCompPoseW_, it must be represented in the deltaCompliance frame
  deltaCompPoseW_ = deltaCompVelIntegral * deltaCompPoseW_;
  // 3.2 Integrate acceleration to velocity
  // deltaCompVelW_ += dt * deltaCompAccelW_;

  if(deltaCompVelW_.linear().norm() > deltaCompVelLinLimit_)
  {
    mc_rtc::log::warning("[ObserverbasedImpedanceTask] Linear deltaCompVel limited from {} to {}",
                         deltaCompVelW_.linear().norm(), deltaCompVelLinLimit_);
    deltaCompVelW_.linear().normalize();
    deltaCompVelW_.linear() *= deltaCompVelLinLimit_;
  }
  if(deltaCompVelW_.angular().norm() > deltaCompVelAngLimit_)
  {
    mc_rtc::log::warning("[ObserverbasedImpedanceTask] Angular deltaCompVel limited from {} to {}",
                         deltaCompVelW_.angular().norm(), deltaCompVelLinLimit_);
    deltaCompVelW_.angular().normalize();
    deltaCompVelW_.angular() *= deltaCompVelAngLimit_;
  }

  if(deltaCompPoseW_.translation().norm() > deltaCompPoseLinLimit_)
  {
    mc_rtc::log::warning("[ObserverbasedImpedanceTask] Linear deltaCompPose limited from {} to {}",
                         deltaCompPoseW_.translation().norm(), deltaCompPoseLinLimit_);
    deltaCompPoseW_.translation().normalize();
    deltaCompPoseW_.translation() *= deltaCompPoseLinLimit_;
  }
  Eigen::AngleAxisd aaDeltaCompRot(deltaCompPoseW_.rotation());
  if(aaDeltaCompRot.angle() > deltaCompPoseAngLimit_)
  {
    mc_rtc::log::warning("[ObserverbasedImpedanceTask] Angular deltaCompPose limited from {} to {}",
                         aaDeltaCompRot.angle(), deltaCompPoseAngLimit_);
    aaDeltaCompRot.angle() = deltaCompPoseAngLimit_;
    deltaCompPoseW_.rotation() = aaDeltaCompRot.toRotationMatrix();
  }

  // 4. Update deltaCompPoseW_ in hold mode (See the hold method documentation for more information)
  if(hold_)
  {
    // Transform to target pose frame (see compliancePose implementation)
    sva::PTransformd T_0_d(targetPoseW_.rotation());
    // The previous compliancePose() is stored in mc_tasks::TransformTask::target()
    deltaCompPoseW_ = T_0_d.inv() * mc_tasks::TransformTask::target() * targetPoseW_.inv() * T_0_d;
  }

  // 5. Set compliance values to the targets of mc_tasks::TransformTask
  mc_tasks::TransformTask::refAccel(T_0_s * (targetAccelW_ + deltaCompAccelW_)); // represented in the surface frame
  mc_tasks::TransformTask::refVelB(T_0_s * (targetVelW_ + deltaCompVelW_)); // represented in the surface frame
  mc_tasks::TransformTask::target(compliancePose()); // represented in the world frame

  t_ += dt; // for plot
}

void ObserverbasedImpedanceTask::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  mc_rtc::log::info("[ObserverbasedImpedanceTask] {}", config.dump(true, true));
  if(config.has("gains")) { gains_ = config("gains"); }
  if(config.has("wrench")) { targetWrench(config("wrench")); }
  if(config.has("cutoffPeriod")) { cutoffPeriod(config("cutoffPeriod")); }
  TransformTask::load(solver, config);

  targetPose(TransformTask::target());

  robot_ = config("robot", robots.robot(rIndex).name());
  MaxContacts_ = config("MaxContacts", 4);
  if(config.has("exportValue"))
  {
    auto exportValueConfig = config("exportValue");
    if(exportValueConfig.has("exportContactWrench")) { exportValueConfig("exportContactWrench", exportContactWrench_); }
    if(exportValueConfig.has("exportExternalWrench"))
    {
      exportValueConfig("exportExternalWrench", exportExternalWrench_);
    }
    if(exportValueConfig.has("usingWrench"))
    {
      exportValueConfig("usingWrench", usingWrench_);
      mc_rtc::log::info("[ObserverbasedImpedance] usingWrench_: {}", usingWrench_);
    }
  }
  else { mc_rtc::log::error("[ObserverbasedImpedanceTask] No exportValue is specified in the config file"); }
  if(config.has("addPlot")) { addPlot_ = config("addPlot"); }
}

void ObserverbasedImpedanceTask::getestimatedExternalWrench()
{
  if(exportExternalWrench_)
  {
    if(controller_->datastore().has(robot_ + "::estimatedExternalWrench_Force")
       && controller_->datastore().has(robot_ + "::estimatedExternalWrench_Torque"))
    {
      estimatedExternalWrench_centroid_.force() =
          controller_->datastore().get<Eigen::Vector3d>(robot_ + "::estimatedExternalWrench_Force");
      estimatedExternalWrench_centroid_.couple() =
          controller_->datastore().get<Eigen::Vector3d>(robot_ + "::estimatedExternalWrench_Torque");
    }
    else
    {
      auto keys = controller_->datastore().keys();
      std::string keys_str;
      for(size_t i = 0; i < keys.size(); ++i)
      {
        keys_str += keys[i];
        if(i < keys.size() - 1) keys_str += ", ";
      }

      // mc_rtc::log::error("[ObserverbasedImpedanceTask] {} is empty. \n Available keys are {}",
      //                    robot_ + "::estimatedExternalWrench", keys_str);
    }
    if(controller_->datastore().has(robot_ + "::worldCentroidKinePTrans"))
    {
      worldCentroidKinePTrans_ = controller_->datastore().get<sva::PTransformd>(robot_ + "::worldCentroidKinePTrans");
    }
  }
  // else { mc_rtc::log::error("[ObserverbasedImpedanceTask] No EstimatedExternalWrench is exported"); }
  return;
}

void ObserverbasedImpedanceTask::getestimatedContactWrench(const std::string & surface)
{
  static const std::map<std::string, int> surfaceMap = {{"RightFoot", 0},   {"LeftFoot", 1},  {"RightGripper", 2},
                                                        {"LeftGripper", 3}, {"RightHand", 2}, {"LeftHand", 3}};

  auto it = surfaceMap.find(surface);

  int i = it->second;
  if(exportContactWrench_)
  {
    if(it == surfaceMap.end())
    {
      mc_rtc::log::error("[ObserverbasedImpedanceTask] Surface name is not correct");
      return;
    }
    if(controller_->datastore().has(robot_ + "::estimatedContactWrench_" + std::to_string(i)))
    {
      estimatedContactWrench_ =
          controller_->datastore().get<sva::ForceVecd>(robot_ + "::estimatedContactWrench_" + std::to_string(i));
      estimatedContactWrench_ = replaceForceTorque(estimatedContactWrench_);
    }
  }
}

sva::ForceVecd ObserverbasedImpedanceTask::replaceForceTorque(sva::ForceVecd target)
{
  sva::ForceVecd tmp = sva::ForceVecd::Zero();
  tmp.couple() = target.force();
  tmp.force() = target.couple();

  return tmp;
}

sva::ForceVecd ObserverbasedImpedanceTask::transformContactWrench(const sva::ForceVecd wrench,
                                                                  const std::string surface,
                                                                  const std::string forceSensor)
{
  sva::PTransformd X_0_surface = robots.robot(rIndex).frame(surface).position();

  sva::PTransformd X_0_ft = robots.robot(rIndex).forceSensor(forceSensor).X_0_f(robots.robot(rIndex));

  sva::PTransformd X_surface_ft = X_0_ft * X_0_surface.inv();

  sva::ForceVecd wrench_out = X_surface_ft.dualMul(wrench);

  return wrench_out;
}

sva::ForceVecd ObserverbasedImpedanceTask::transformExternalWrench(const sva::ForceVecd wrench,
                                                                   const std::string surface)
{
  sva::PTransformd X_0_surface = robots.robot(rIndex).frame(surface).position(); // ^surface X_0

  sva::PTransformd X_0_centroid = worldCentroidKinePTrans_; // ^controid X_0

  sva::PTransformd X_surface_com = X_0_surface * X_0_centroid.inv();

  sva::ForceVecd wrench_out = X_surface_com.dualMul(wrench);

  return wrench_out;
}

void ObserverbasedImpedanceTask::addToLogger(mc_rtc::Logger & logger)
{
  TransformTask::addToLogger(logger);
  std::string category = name_;
  std::string subcategory_est = "estimation";
  std::string subcategory_force = "forcesensor_surfaceFrame";
  std::string wrench_category = "_wrench_";

  // impedance parameters
  logger.addLogEntry(name_ + "_gains_M", this, [this]() -> const sva::ImpedanceVecd & { return gains().M().vec(); });
  logger.addLogEntry(name_ + "_gains_D", this, [this]() -> const sva::ImpedanceVecd & { return gains().D().vec(); });
  logger.addLogEntry(name_ + "_gains_K", this, [this]() -> const sva::ImpedanceVecd & { return gains().K().vec(); });
  logger.addLogEntry(name_ + "_gains_wrench", this,
                     [this]() -> const sva::ImpedanceVecd & { return gains().wrench().vec(); });

  // compliance values
  logger.addLogEntry(name_ + "_compliancePose", this, [this]() { return compliancePose(); });
  MC_RTC_LOG_HELPER(name_ + "_deltaCompliancePose", deltaCompPoseW_);
  MC_RTC_LOG_HELPER(name_ + "_deltaComplianceVel", deltaCompVelW_);
  MC_RTC_LOG_HELPER(name_ + "_deltaComplianceAccel", deltaCompAccelW_);

  // target values
  MC_RTC_LOG_HELPER(name_ + "_targetPose", targetPoseW_);
  MC_RTC_LOG_HELPER(name_ + "_targetVel", targetVelW_);
  MC_RTC_LOG_HELPER(name_ + "_targetAccel", targetAccelW_);

  MC_RTC_LOG_HELPER(category + wrench_category + subcategory_force, surfaceWrench_);
  MC_RTC_LOG_HELPER(category + wrench_category + subcategory_est + "_ExternalWrench_centroid",
                    estimatedExternalWrench_centroid_);
  MC_RTC_LOG_HELPER(category + wrench_category + subcategory_est + "_ExternalWrench_surfaceFrame", measuredWrench_);
  MC_RTC_LOG_HELPER(category + wrench_category + subcategory_est + "_estimationError", estimationError_);
  MC_RTC_LOG_HELPER(category + wrench_category + usingWrench_ + "_targetWrench", targetWrench_);
  MC_RTC_LOG_HELPER(category + wrench_category + usingWrench_ + "_measuredWrench", measuredWrench_);
  MC_RTC_LOG_HELPER(category + wrench_category + usingWrench_ + "_wrenchError", wrenchError_);
}

void ObserverbasedImpedanceTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  // Don't add TransformTask because the target of TransformTask should not be set by user
  TrajectoryTaskGeneric::addToGUI(gui);

  gui.addElement({"Tasks", name_},
                 mc_rtc::gui::Transform(
                     "targetPose", [this]() -> const sva::PTransformd & { return this->targetPose(); },
                     [this](const sva::PTransformd & pos) { this->targetPose(pos); }));
  gui.addElement({"Tasks", name_},
                 mc_rtc::gui::Transform("compliancePose", [this]() { return this->compliancePose(); }));
  gui.addElement({"Tasks", name_}, mc_rtc::gui::Transform("pose", [this]() { return this->surfacePose(); }));
  gui.addElement({"Tasks", name_}, mc_rtc::gui::ArrayInput(
                                       "targetWrench", {"cx", "cy", "cz", "fx", "fy", "fz"},
                                       [this]() { return this->targetWrench().vector(); },
                                       [this](const Eigen::Vector6d & a) { this->targetWrench(a); }));
  gui.addElement({"Tasks", name_}, mc_rtc::gui::ArrayLabel("measuredWrench", {"cx", "cy", "cz", "fx", "fy", "fz"},
                                                           [this]() { return this->measuredWrench_.vector(); }));
  gui.addElement({"Tasks", name_},
                 mc_rtc::gui::ArrayLabel("filteredMeasuredWrench", {"cx", "cy", "cz", "fx", "fy", "fz"},
                                         [this]() { return this->filteredMeasuredWrench_.vector(); }));
  gui.addElement({"Tasks", name_}, mc_rtc::gui::NumberInput(
                                       "cutoffPeriod", [this]() { return this->cutoffPeriod(); },
                                       [this](double a) { return this->cutoffPeriod(a); }));
  gui.addElement({"Tasks", name_},
                 mc_rtc::gui::Checkbox("hold", [this]() { return hold_; }, [this]() { hold_ = !hold_; }));
  gui.addElement(
      {"Tasks", name_, "Observerbased Impedance gains"},
      mc_rtc::gui::ArrayInput(
          "mass", {"cx", "cy", "cz", "fx", "fy", "fz"}, [this]() -> const sva::ImpedanceVecd &
          { return gains().mass().vec(); }, [this](const Eigen::Vector6d & a) { gains().mass().vec(a); }),
      mc_rtc::gui::ArrayInput(
          "damper", {"cx", "cy", "cz", "fx", "fy", "fz"}, [this]() -> const sva::ImpedanceVecd &
          { return gains().damper().vec(); }, [this](const Eigen::Vector6d & a) { gains().damper().vec(a); }),
      mc_rtc::gui::ArrayInput(
          "spring", {"cx", "cy", "cz", "fx", "fy", "fz"}, [this]() -> const sva::ImpedanceVecd &
          { return gains().spring().vec(); }, [this](const Eigen::Vector6d & a) { gains().spring().vec(a); }),
      mc_rtc::gui::ArrayInput(
          "wrench", {"cx", "cy", "cz", "fx", "fy", "fz"}, [this]() -> const sva::ImpedanceVecd &
          { return gains().wrench().vec(); }, [this](const Eigen::Vector6d & a) { gains().wrench().vec(a); }));

  if(addPlot_)
  {
    gui.addPlot(name_ + "_wrench fx", mc_rtc::gui::plot::X("t", [this]() { return t_; }),
                mc_rtc::gui::plot::AxisConfiguration("Y", {-100, 100}),
                mc_rtc::gui::plot::Y(
                    "target fx", [this]() { return this->targetWrench().vector()[3]; }, Color::Red),
                mc_rtc::gui::plot::Y(
                    "estimated fx", [this]() { return this->measuredWrench_.vector()[3]; }, Color::Blue,
                    mc_rtc::gui::plot::Style::Dashed),
                mc_rtc::gui::plot::Y(
                    "measured fx", [this]() { return this->surfaceWrench_.vector()[3]; }, Color::Green,
                    mc_rtc::gui::plot::Style::Dotted));

    gui.addPlot(name_ + "_wrench fy", mc_rtc::gui::plot::X("t", [this]() { return t_; }),
                mc_rtc::gui::plot::AxisConfiguration("Y", {-100, 100}),
                mc_rtc::gui::plot::Y(
                    "target fy", [this]() { return this->targetWrench().vector()[4]; }, Color::Red),
                mc_rtc::gui::plot::Y(
                    "estimated fy", [this]() { return this->measuredWrench_.vector()[4]; }, Color::Blue,
                    mc_rtc::gui::plot::Style::Dashed),
                mc_rtc::gui::plot::Y(
                    "measured fy", [this]() { return this->surfaceWrench_.vector()[4]; }, Color::Green,
                    mc_rtc::gui::plot::Style::Dotted));
    gui.addPlot(name_ + "_wrench fz", mc_rtc::gui::plot::X("t", [this]() { return t_; }),
                mc_rtc::gui::plot::AxisConfiguration("Y", {-1000, 1000}),
                mc_rtc::gui::plot::Y(
                    "target fz", [this]() { return this->targetWrench().vector()[5]; }, Color::Red),
                mc_rtc::gui::plot::Y(
                    "estimated fz", [this]() { return this->measuredWrench_.vector()[5]; }, Color::Blue,
                    mc_rtc::gui::plot::Style::Dashed),
                mc_rtc::gui::plot::Y(
                    "measured fz", [this]() { return this->surfaceWrench_.vector()[5]; }, Color::Green,
                    mc_rtc::gui::plot::Style::Dotted));
    gui.addPlot(name_ + "_wrench cx", mc_rtc::gui::plot::X("t", [this]() { return t_; }),
                mc_rtc::gui::plot::AxisConfiguration("Y", {-50, 50}),
                mc_rtc::gui::plot::Y(
                    "target cx", [this]() { return this->targetWrench().vector()[0]; }, Color::Red),
                mc_rtc::gui::plot::Y(
                    "estimated cx", [this]() { return this->measuredWrench_.vector()[0]; }, Color::Blue,
                    mc_rtc::gui::plot::Style::Dashed),
                mc_rtc::gui::plot::Y(
                    "measured cx", [this]() { return this->surfaceWrench_.vector()[0]; }, Color::Green,
                    mc_rtc::gui::plot::Style::Dotted));
    gui.addPlot(name_ + "_wrench cy", mc_rtc::gui::plot::X("t", [this]() { return t_; }),
                mc_rtc::gui::plot::AxisConfiguration("Y", {-50, 50}),
                mc_rtc::gui::plot::Y(
                    "target cy", [this]() { return this->targetWrench().vector()[1]; }, Color::Red),
                mc_rtc::gui::plot::Y(
                    "estimated cy", [this]() { return this->measuredWrench_.vector()[1]; }, Color::Blue,
                    mc_rtc::gui::plot::Style::Dashed),
                mc_rtc::gui::plot::Y(
                    "measured cy", [this]() { return this->surfaceWrench_.vector()[1]; }, Color::Green,
                    mc_rtc::gui::plot::Style::Dotted));
    gui.addPlot(name_ + "_wrench cz", mc_rtc::gui::plot::X("t", [this]() { return t_; }),
                mc_rtc::gui::plot::AxisConfiguration("Y", {-50, 50}),
                mc_rtc::gui::plot::Y(
                    "target cz", [this]() { return this->targetWrench().vector()[2]; }, Color::Red),
                mc_rtc::gui::plot::Y(
                    "estimated cz", [this]() { return this->measuredWrench_.vector()[2]; }, Color::Blue,
                    mc_rtc::gui::plot::Style::Dashed),
                mc_rtc::gui::plot::Y(
                    "measured cz", [this]() { return this->surfaceWrench_.vector()[2]; }, Color::Green,
                    mc_rtc::gui::plot::Style::Dotted));
  }
}

} // namespace force
} // namespace mc_tasks

namespace
{
static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "ObserverbasedImpedance",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
    {
      using Allocator = Eigen::aligned_allocator<mc_tasks::force::ObserverbasedImpedanceTask>;
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
          std::allocate_shared<mc_tasks::force::ObserverbasedImpedanceTask>(Allocator{}, frame, solver.controller());
      t->reset();
      t->load(solver, config);
      return t;
    });
} // namespace