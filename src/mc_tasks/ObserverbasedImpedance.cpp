#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/ObserverbasedImpedance.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include "mc_rtc/logging.h"

namespace mc_tasks
{
namespace force
{

ObserverbasedImpedance::ObserverbasedImpedance(const std::string & surfaceName,
                                               const mc_rbdyn::Robots & robots,
                                               unsigned robotIndex,
                                               double stiffness,
                                               double weight)
: ObserberbasedImpedance(robots.robot(robotIndex).frame(surfaceName), stiffness, weight)
{
}

ObserverbasedImpedance::ObserverbasedImpedance(const mc_rbdyn::RobotFrame & frame, double stiffness, double weight)
: ImpedanceTask(frame, stiffness, weight)
{
  type_ = "observerbasedImpedance";
  name_ = "observerbased_impedance_" + robots.robot(rIndex).name() + "_" + frame.name();
}

void ObserverbasedImpedance::update(mc_solver::QPSolver & solver)
{
  double dt = solver.dt();

  // 1. Filter the estimated wrench
  getestimatedExternalWrench();
  lowPass_.update(estimatedContactWrench_);
  filteredMeasuredWrench_ = lowPass_.eval();

  getestimatedContactWrench(surface());

  // TODO: replace measuredWrench_ with EstimatedContactWrench
  // TODO: Transform the estimated contact wrench to the frame which is same to measuredWrench_

  sva::MotionVecd deltaCompVelWPrev = deltaCompVelW_;
  sva::PTransformd T_0_s(surfacePose().rotation());
  deltaCompVelW_ = T_0_s.invMul(sva::MotionVecd(gains().D().vector().cwiseInverse().cwiseProduct(
      -gains().K().vector().cwiseProduct((T_0_s * sva::transformVelocity(deltaCompPoseW_)).vector())
      + gains().wrench().vector().cwiseProduct((filteredMeasuredWrench_ - targetWrench_).vector()))));
  deltaCompAccelW_ = (deltaCompVelW_ - deltaCompVelWPrev) / dt;

  if(deltaCompAccelW_.linear().norm() > deltaCompAccelLinLimit_)
  {
    mc_rtc::log::warning("[FirstOrderImpedanceTask] Linear deltaCompAccel limited from {} to {}",
                         deltaCompAccelW_.linear().norm(), deltaCompAccelLinLimit_);
    deltaCompAccelW_.linear().normalize();
    deltaCompAccelW_.linear() *= deltaCompAccelLinLimit_;
  }
  if(deltaCompAccelW_.angular().norm() > deltaCompAccelAngLimit_)
  {
    mc_rtc::log::warning("[FirstOrderImpedanceTask] Angular deltaCompAccel limited from {} to {}",
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
    mc_rtc::log::warning("[FirstOrderImpedanceTask] Linear deltaCompVel limited from {} to {}",
                         deltaCompVelW_.linear().norm(), deltaCompVelLinLimit_);
    deltaCompVelW_.linear().normalize();
    deltaCompVelW_.linear() *= deltaCompVelLinLimit_;
  }
  if(deltaCompVelW_.angular().norm() > deltaCompVelAngLimit_)
  {
    mc_rtc::log::warning("[FirstOrderImpedanceTask] Angular deltaCompVel limited from {} to {}",
                         deltaCompVelW_.angular().norm(), deltaCompVelLinLimit_);
    deltaCompVelW_.angular().normalize();
    deltaCompVelW_.angular() *= deltaCompVelAngLimit_;
  }

  if(deltaCompPoseW_.translation().norm() > deltaCompPoseLinLimit_)
  {
    mc_rtc::log::warning("[FirstOrderImpedanceTask] Linear deltaCompPose limited from {} to {}",
                         deltaCompPoseW_.translation().norm(), deltaCompPoseLinLimit_);
    deltaCompPoseW_.translation().normalize();
    deltaCompPoseW_.translation() *= deltaCompPoseLinLimit_;
  }
  Eigen::AngleAxisd aaDeltaCompRot(deltaCompPoseW_.rotation());
  if(aaDeltaCompRot.angle() > deltaCompPoseAngLimit_)
  {
    mc_rtc::log::warning("[FirstOrderImpedanceTask] Angular deltaCompPose limited from {} to {}",
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

void ObserverbasedImpedance::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  if(config.has("gains")) { gains_ = config("gains"); }
  if(config.has("wrench")) { targetWrench(config("wrench")); }
  if(config.has("cutoffPeriod")) { cutoffPeriod(config("cutoffPeriod")); }
  TransformTask::load(solver, config);
  // The TransformTask::load function above only sets
  // the TrajectoryTaskGeneric's target, but not the compliance target, so we
  // need to set it manually here.
  targetPose(TransformTask::target());

  robot_ = config("robot", robot().name());
  MaxContacts_ = config("MaxContacts", 4);
  if(config.has("exportValue"))
  {
    auto exportValueConfig = config("exportValue");
    if(exportValueConfig.has("exportContactWrench")) { exportValueConfig("exportContactWrench", exportContactWrench_); }
    if(exportValueConfig.has("exportExternalWrench"))
    {
      exportValueConfig("exportExternalWrench", exportExternalWrench_);
    }
  }
  mc_rtc::log::info("exportContactWrench_: {}", exportContactWrench_); // for debug
  mc_rtc::log::info("exportExternalWrench_: {}", exportExternalWrench_); // for debug
}

void ObserverbasedImpedance::getestimatedExternalWrench()
{
  if(exportExternalWrench_)
  {
    if(datastore().has(robot_ + "::estimatedExternalWrench"))
    {
      estimatedExternalWrench_centroid_ = datastore().get<sva::ForceVecd>(robot_ + "::estimatedExternalWrench");
    }
  }

  return;
}

void ObserverbasedImpedance::getestimatedContactWrench(const std::string & surface)
{
  static const std::map<std::string, int> surfaceMap = {
      {"RightFoot", 0}, {"LeftFoot", 1}, {"RightHand", 2}, {"LeftHand", 3}};

  auto it = surfaceMap.find(surface);
  if(it == surfaceMap.end())
  {
    mc_rtc::log::error("[ObserverbasedImpedance] Surface name is not correct");
    return;
  }

  int i = it->second;
  if(exportContactWrench_)
  {
    if(datastore().has(robot_ + "::estimatedContactWrench_" + std::to_string(i)))
    {
      estimatedContactWrench_ =
          datastore().get<sva::ForceVecd>(robot_ + "::estimatedContactWrench_" + std::to_string(i));
      estimatedContactWrench_ = replaceForceTorque(estimatedContactWrench_);
    }
  }
  else { mc_rtc::log::error("[ObserverbasedImpedance] No EstimatedContactWrench is exported"); }
}

sva::ForceVecd ObserverbasedImpedance::replaceForceTorque(sva::ForceVecd target)
{
  sva::ForceVecd tmp = sva::ForceVecd::Zero();
  tmp.couple() = target.force();
  tmp.force() = target.couple();

  return tmp;
}

} // namespace force
} // namespace mc_tasks

namespace
{
static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "observerbasedImpedance",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
    {
      using Allocator = Eigen::aligned_allocator<mc_tasks::force::ObserverbasedImpedance>;
      const auto robotIndex = robotIndexFromConfig(config, solver.robots(), "observerbasedImpedance");
      const auto & robot = solver.robots().robot(robotIndex);
      const auto & frame = [&]() -> const mc_rbdyn::RobotFrame &
      {
        if(config.has("surface"))
        {
          mc_rtc::log::deprecated("ObserverbasedImpedance", "surface", "frame");
          return robot.frame(config("surface"));
        }
        return robot.surface(config("frame"));
      }();

      auto t = std::make_shared<mc_tasks::force::ObserverbasedImpedance>(Allocator{}, frame);
      t->reset();
      t->load(solver, config);
      return t;
    });
} // namespace