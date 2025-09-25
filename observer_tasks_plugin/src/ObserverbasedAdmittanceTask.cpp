#include "ObserverbasedAdmittanceTask.h"

namespace observer_tasks
{

using mc_filter::utils::clampInPlaceAndWarn;

ObserverbasedAdmittanceTask::ObserverbasedAdmittanceTask(const std::string & surfaceName,
                                                         const mc_rbdyn::Robots & robots,
                                                         mc_control::MCController * controller,
                                                         unsigned int robotIndex,
                                                         double stiffness,
                                                         double weight)
: ObserverbasedAdmittanceTask(robots.robot(robotIndex).frame(surfaceName), controller, stiffness, weight)
{
}

ObserverbasedAdmittanceTask::ObserverbasedAdmittanceTask(const mc_rbdyn::RobotFrame & frame,
                                                         mc_control::MCController * controller,
                                                         double stiffness,
                                                         double weight)
: mc_tasks::TransformTask(frame, stiffness, weight), robot_(const_cast<mc_rbdyn::Robot &>(frame.robot())),
  controller_(controller)
{
  if(!frame.hasForceSensor())
  {
    mc_rtc::log::error_and_throw(
        "[mc_tasks::ObserverbasedAdmittanceTask] Frame {} does not have a force sensor attached", frame.name());
  }
  name_ = "Observerbasedadmittance_" + frame.robot().name() + "_" + frame.name();
  reset();
}

void ObserverbasedAdmittanceTask::update(mc_solver::QPSolver &)
{
  // Compute wrench error
  getestimatedContactWrench(surface());
  getestimatedExternalWrench();
  estimatedExternalWrench_surfaceFrame_ = transformExternalWrench(estimatedExternalWrench_centroid_, surface());

  if(usingWrench_ == "Contact")
  {
    wrenchError_ = estimatedContactWrench_ - targetWrench_;
    estimationError_ = measuredWrench() - estimatedContactWrench_;
  }
  else if(usingWrench_ == "External")
  {
    wrenchError_ = estimatedExternalWrench_surfaceFrame_ - targetWrench_;
    estimationError_ = measuredWrench() - estimatedExternalWrench_surfaceFrame_;
  }
  else { wrenchError_ = measuredWrench() - targetWrench_; }
  // Compute linear and angular velocity based on wrench error and admittance
  Eigen::Vector3d linearVel = Observerbasedadmittance_.force().cwiseProduct(wrenchError_.force());
  Eigen::Vector3d angularVel = Observerbasedadmittance_.couple().cwiseProduct(wrenchError_.couple());

  // Clamp both values in order to have a 'security'
  clampInPlaceAndWarn(linearVel, (-maxLinearVel_).eval(), maxLinearVel_, name_ + " linear velocity");
  clampInPlaceAndWarn(angularVel, (-maxAngularVel_).eval(), maxAngularVel_, name_ + " angular velocity");

  // Filter
  refVelB_ = velFilterGain_ * refVelB_ + (1 - velFilterGain_) * sva::MotionVecd(angularVel, linearVel);

  // Compute position and rotation delta
  sva::PTransformd delta(mc_rbdyn::rpyToMat(timestep_ * refVelB_.angular()), timestep_ * refVelB_.linear());

  // Acceleration
  mc_tasks::TransformTask::refAccel((refVelB_ + feedforwardVelB_ - mc_tasks::TransformTask::refVelB()) / timestep_);

  // Velocity
  mc_tasks::TransformTask::refVelB(refVelB_ + feedforwardVelB_);

  // Position
  target(delta * target());
}

void ObserverbasedAdmittanceTask::reset()
{
  mc_tasks::TransformTask::reset();
  Observerbasedadmittance_ = sva::ForceVecd(Eigen::Vector6d::Zero());
  feedforwardVelB_ = sva::MotionVecd(Eigen::Vector6d::Zero());
  targetWrench_ = sva::ForceVecd(Eigen::Vector6d::Zero());
  wrenchError_ = sva::ForceVecd(Eigen::Vector6d::Zero());

  estimatedContactWrench_ = sva::ForceVecd(Eigen::Vector6d::Zero());
  estimatedExternalWrench_centroid_ = sva::ForceVecd(Eigen::Vector6d::Zero());
  estimatedExternalWrench_surfaceFrame_ = sva::ForceVecd(Eigen::Vector6d::Zero());
  estimationError_ = sva::ForceVecd(Eigen::Vector6d::Zero());
  worldCentroidKinePTrans_ = sva::PTransformd::Identity();
}

void ObserverbasedAdmittanceTask::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  if(config.has("Observerbasedadmittance")) { Observerbasedadmittance(config("Observerbasedadmittance")); }
  else if(config.has("targetPose"))
  {
    mc_rtc::log::warning("[{}] property \"targetPose\" is deprecated, use \"target\" instead", name());
    targetPose(config("targetPose"));
  }
  if(config.has("wrench")) { targetWrench(config("wrench")); }
  if(config.has("refVelB")) { refVelB(config("refVelB")); }
  if(config.has("maxVel"))
  {
    sva::MotionVecd maxVel = config("maxVel");
    maxLinearVel(maxVel.linear());
    maxAngularVel(maxVel.angular());
  }
  mc_tasks::TransformTask::load(solver, config);
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
      mc_rtc::log::info("usingWrench_: {}", usingWrench_);
    }
  }
}

void ObserverbasedAdmittanceTask::addToLogger(mc_rtc::Logger & logger)
{
  mc_tasks::TransformTask::addToLogger(logger);
  MC_RTC_LOG_HELPER(name_ + "_Observerbasedadmittance", Observerbasedadmittance_);
  MC_RTC_LOG_HELPER(name_ + "_estimation" + "_ContactWrench_surfance", estimatedContactWrench_);
  MC_RTC_LOG_HELPER(name_ + "_estimation" + "_ExternalWrench_centroid", estimatedExternalWrench_centroid_);
  MC_RTC_LOG_HELPER(name_ + "_estimation" + "_ExternalWrench_surfaceFrame", estimatedExternalWrench_surfaceFrame_);
  MC_RTC_LOG_HELPER(name_ + "_estimation" + "_centroidFrame_Ptransformd", worldCentroidKinePTrans_);

  MC_RTC_LOG_HELPER(name_ + "_estimation" + "_estimationError", estimationError_);
  MC_RTC_LOG_HELPER(name_ + "_target_body_vel", feedforwardVelB_);
  MC_RTC_LOG_HELPER(name_ + "_target_wrench", targetWrench_);
  MC_RTC_LOG_HELPER(name_ + "_vel_filter_gain", velFilterGain_);
  MC_RTC_LOG_HELPER(name_ + "_wrenchError", wrenchError_);
  MC_RTC_LOG_HELPER(name_ + "_RightHandForceSensor" + "_surfaceFrame", measuredWrench());
}

void ObserverbasedAdmittanceTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.addElement(
      {"Tasks", name_},
      mc_rtc::gui::Transform(
          "pos_target", [this]() { return this->targetPose(); },
          [this](const sva::PTransformd & pos) { this->targetPose(pos); }),
      mc_rtc::gui::Transform("pos", [this]() { return frame_->position(); }),
      mc_rtc::gui::ArrayInput(
          "Observerbasedadmittance", {"cx", "cy", "cz", "fx", "fy", "fz"},
          [this]() { return this->Observerbasedadmittance().vector(); },
          [this](const Eigen::Vector6d & a) { this->Observerbasedadmittance(a); }),
      mc_rtc::gui::ArrayInput(
          "wrench", {"cx", "cy", "cz", "fx", "fy", "fz"}, [this]() { return this->targetWrench().vector(); },
          [this](const Eigen::Vector6d & a) { this->targetWrench(a); }),
      mc_rtc::gui::ArrayLabel("estimatedContactWrench", {"cx", "cy", "cz", "fx", "fy", "fz"},
                              [this]() { return this->estimatedContactWrench().vector(); }),
      mc_rtc::gui::NumberInput(
          "Velocity filter gain", [this]() { return velFilterGain_; }, [this](double g) { velFilterGain(g); }));
  // Don't add TransformTask as target configuration is different
  mc_tasks::TrajectoryTaskGeneric::addToGUI(gui);
}

void ObserverbasedAdmittanceTask::addToSolver(mc_solver::QPSolver & solver)
{
  timestep_ = solver.dt();
  mc_tasks::TransformTask::addToSolver(solver);
}

void ObserverbasedAdmittanceTask::getestimatedExternalWrench()
{
  if(exportExternalWrench_)
  {
    if(controller_->datastore().has(robot_.name() + "::estimatedExternalWrench_Force")
       && controller_->datastore().has(robot_.name() + "::estimatedExternalWrench_Torque"))
    {
      estimatedExternalWrench_centroid_.force() =
          controller_->datastore().get<Eigen::Vector3d>(robot_.name() + "::estimatedExternalWrench_Force");
      estimatedExternalWrench_centroid_.couple() =
          controller_->datastore().get<Eigen::Vector3d>(robot_.name() + "::estimatedExternalWrench_Torque");
    }
    if(controller_->datastore().has(robot_.name() + "::worldCentroidKinePTrans"))
    {
      worldCentroidKinePTrans_ =
          controller_->datastore().get<sva::PTransformd>(robot_.name() + "::worldCentroidKinePTrans");
    }
  }

  return;
}

void ObserverbasedAdmittanceTask::getestimatedContactWrench(const std::string & surface)
{
  static const std::map<std::string, int> surfaceMap = {
      {"RightFoot", 0}, {"LeftFoot", 1}, {"RightHand", 2}, {"LeftHand", 3}};

  auto it = surfaceMap.find(surface);
  if(it == surfaceMap.end())
  {
    mc_rtc::log::error("[ObserverbasedAdmittanceTask] Surface name is not correct");
    return;
  }

  int i = it->second;
  if(exportContactWrench_)
  {
    if(controller_->datastore().has(robot_.name() + "::estimatedContactWrench_" + std::to_string(i)))
    {
      estimatedContactWrench_ =
          controller_->datastore().get<sva::ForceVecd>(robot_.name() + "::estimatedContactWrench_" + std::to_string(i));
      estimatedContactWrench_ = replaceForceTorque(estimatedContactWrench_);
    }
    else
    {
      mc_rtc::log::error("[ObserverbasedAdmittanceTask] {} is empty",
                         robot_.name() + "::estimatedContactWrench_" + std::to_string(i));
    }
  }
}

sva::ForceVecd ObserverbasedAdmittanceTask::replaceForceTorque(sva::ForceVecd target)
{
  sva::ForceVecd tmp = sva::ForceVecd::Zero();
  tmp.couple() = target.force();
  tmp.force() = target.couple();

  return tmp;
}

sva::ForceVecd ObserverbasedAdmittanceTask::transformContactWrench(const sva::ForceVecd wrench,
                                                                   const std::string surface,
                                                                   const std::string forceSensor)
{
  sva::PTransformd X_0_surface = robot_.frame(surface).position();

  sva::PTransformd X_0_ft = robot_.forceSensor(forceSensor).X_0_f(robot_);

  sva::PTransformd X_surface_ft = X_0_ft * X_0_surface.inv();

  sva::ForceVecd wrench_out = X_surface_ft.dualMul(wrench);

  return wrench_out;
}

sva::ForceVecd ObserverbasedAdmittanceTask::transformExternalWrench(const sva::ForceVecd wrench,
                                                                    const std::string surface)
{
  sva::PTransformd X_0_surface = robot_.frame(surface).position(); // ^surface X_0

  sva::PTransformd X_0_centroid = worldCentroidKinePTrans_; // ^controid X_0

  sva::PTransformd X_surface_com = X_0_surface * X_0_centroid.inv();

  sva::ForceVecd wrench_out = X_surface_com.dualMul(wrench);

  return wrench_out;
}

} // namespace observer_tasks