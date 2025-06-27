#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/ObserverbasedImpedanceTask.h>
#include "mc_rtc/log/Logger.h"
#include "mc_rtc/logging.h"

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
  getestimatedContactWrench(surface());
  getestimatedExternalWrench();

  // choose the wrench to use
  if(usingWrench_ == "Contact")
  {
    measuredWrench_ = transformContactWrench(estimatedContactWrench_, surface(), frame_->forceSensor().name());
    wrenchError_ = estimatedContactWrench_ - targetWrench_;
    estimationError_ = measuredWrench() - estimatedContactWrench_;
  }
  else if(usingWrench_ == "External")
  {
    measuredWrench_ = transformExternalWrench(estimatedExternalWrench_centroid_, surface());
    wrenchError_ = measuredWrench_ - targetWrench_;
    estimationError_ = measuredWrench() - measuredWrench_;
  }
  else { measuredWrench_ = frame_->wrench(); }

  lowPass_.update(estimatedContactWrench_);
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
}

void ObserverbasedImpedanceTask::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
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
    if(controller_->datastore().has(robot_ + "::worldCentroidKinePTrans"))
    {
      worldCentroidKinePTrans_ = controller_->datastore().get<sva::PTransformd>(robot_ + "::worldCentroidKinePTrans");
    }
  }
  return;
}

void ObserverbasedImpedanceTask::getestimatedContactWrench(const std::string & surface)
{
  static const std::map<std::string, int> surfaceMap = {
      {"RightFoot", 0}, {"LeftFoot", 1}, {"RightGripper", 2}, {"LeftGripper", 3}};

  auto it = surfaceMap.find(surface);
  if(it == surfaceMap.end())
  {
    mc_rtc::log::error("[ObserverbasedImpedanceTask] Surface name is not correct");
    return;
  }

  int i = it->second;
  if(exportContactWrench_)
  {
    if(controller_->datastore().has(robot_ + "::estimatedContactWrench_" + std::to_string(i)))
    {
      estimatedContactWrench_ =
          controller_->datastore().get<sva::ForceVecd>(robot_ + "::estimatedContactWrench_" + std::to_string(i));
      estimatedContactWrench_ = replaceForceTorque(estimatedContactWrench_);
    }
  }
  else { mc_rtc::log::error("[ObserverbasedImpedanceTask] No EstimatedContactWrench is exported"); }
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
  std::string category = "ObserverbasedImpedanceTask_";
  std::string subcategory_est = "estimation";
  std::string subcategory_force = "forcesensor_surfaceFrame";

  logger.addLogEntry(category + "forcesensor_" + "surfaceFrame",
                     [this]() { return this->robots.robot(rIndex).surfaceWrench(this->surface()); });
  MC_RTC_LOG_HELPER(category + subcategory_est + "_ContactWrench_surfance", estimatedContactWrench_);
  MC_RTC_LOG_HELPER(category + subcategory_est + "_ContactWrench_sensorFrame", estimatedContactWrench_sensorFrame_);
  MC_RTC_LOG_HELPER(category + subcategory_est + "_ExternalWrench_centroid", estimatedExternalWrench_centroid_);
  MC_RTC_LOG_HELPER(category + subcategory_est + "_estimationError", estimationError_);
}

} // namespace force
} // namespace mc_tasks

namespace
{
static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "ObserverbasedImpedanceTask",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
    {
      using Allocator = Eigen::aligned_allocator<mc_tasks::force::ObserverbasedImpedanceTask>;
      const auto robotIndex = robotIndexFromConfig(config, solver.robots(), "ObserverbasedImpedanceTask");
      const auto & robot = solver.robots().robot(robotIndex);
      const auto & frame = [&]() -> const mc_rbdyn::RobotFrame &
      {
        if(config.has("surface"))
        {
          mc_rtc::log::deprecated("ObserverbasedImpedanceTask", "surface", "frame");
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