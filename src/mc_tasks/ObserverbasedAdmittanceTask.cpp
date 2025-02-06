/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/ObserverbasedAdmittanceTask.h>

namespace mc_tasks
{

namespace force
{

using mc_filter::utils::clampInPlaceAndWarn;

ObserverbasedAdmittanceTask::ObserverbasedAdmittanceTask(const std::string & surfaceName,
                                                         const mc_rbdyn::Robots & robots,
                                                         unsigned int robotIndex,
                                                         double stiffness,
                                                         double weight)
: ObserverbasedAdmittanceTask(robots.robot(robotIndex).frame(surfaceName), stiffness, weight)
{
}

ObserverbasedAdmittanceTask::ObserverbasedAdmittanceTask(const mc_rbdyn::RobotFrame & frame,
                                                         double stiffness,
                                                         double weight)
: TransformTask(frame, stiffness, weight), robot_(const_cast<mc_rbdyn::Robot &>(frame.robot()))
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
  estimatedContactWrench_ = transformContactWrench(estimatedContactWrench_, surface(), frame_->forceSensor().name());
  wrenchError_ = estimatedContactWrench_ - targetWrench_;

  // Compute linear and angular velocity based on wrench error and admittance
  Eigen::Vector3d linearVel = admittance_.force().cwiseProduct(wrenchError_.force());
  Eigen::Vector3d angularVel = admittance_.couple().cwiseProduct(wrenchError_.couple());

  // Clamp both values in order to have a 'security'
  clampInPlaceAndWarn(linearVel, (-maxLinearVel_).eval(), maxLinearVel_, name_ + " linear velocity");
  clampInPlaceAndWarn(angularVel, (-maxAngularVel_).eval(), maxAngularVel_, name_ + " angular velocity");

  // Filter
  refVelB_ = velFilterGain_ * refVelB_ + (1 - velFilterGain_) * sva::MotionVecd(angularVel, linearVel);

  // Compute position and rotation delta
  sva::PTransformd delta(mc_rbdyn::rpyToMat(timestep_ * refVelB_.angular()), timestep_ * refVelB_.linear());

  // Acceleration
  TransformTask::refAccel((refVelB_ + feedforwardVelB_ - TransformTask::refVelB()) / timestep_);

  // Velocity
  TransformTask::refVelB(refVelB_ + feedforwardVelB_);

  // Position
  target(delta * target());
}

void ObserverbasedAdmittanceTask::reset()
{
  TransformTask::reset();
  admittance_ = sva::ForceVecd(Eigen::Vector6d::Zero());
  feedforwardVelB_ = sva::MotionVecd(Eigen::Vector6d::Zero());
  targetWrench_ = sva::ForceVecd(Eigen::Vector6d::Zero());
  wrenchError_ = sva::ForceVecd(Eigen::Vector6d::Zero());

  estimatedContactWrench_ = sva::ForceVecd(Eigen::Vector6d::Zero());
  estimatedExternalWrench_centroid_ = sva::ForceVecd(Eigen::Vector6d::Zero());
}

/*! \brief Load parameters from a Configuration object */
void ObserverbasedAdmittanceTask::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  if(config.has("admittance")) { admittance(config("admittance")); }
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
  TransformTask::load(solver, config);
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

void ObserverbasedAdmittanceTask::addToLogger(mc_rtc::Logger & logger)
{
  TransformTask::addToLogger(logger);
  MC_RTC_LOG_HELPER(name_ + "_admittance", admittance_);
  // MC_RTC_LOG_HELPER(name_ + "_measured_wrench", measuredWrench);
  MC_RTC_LOG_HELPER(name_ + "_estimatedContactWrench", estimatedContactWrench_);
  MC_RTC_LOG_HELPER(name_ + "_target_body_vel", feedforwardVelB_);
  MC_RTC_LOG_HELPER(name_ + "_target_wrench", targetWrench_);
  MC_RTC_LOG_HELPER(name_ + "_vel_filter_gain", velFilterGain_);
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
          "admittance", {"cx", "cy", "cz", "fx", "fy", "fz"}, [this]() { return this->admittance().vector(); },
          [this](const Eigen::Vector6d & a) { this->admittance(a); }),
      mc_rtc::gui::ArrayInput(
          "wrench", {"cx", "cy", "cz", "fx", "fy", "fz"}, [this]() { return this->targetWrench().vector(); },
          [this](const Eigen::Vector6d & a) { this->targetWrench(a); }),
      // mc_rtc::gui::ArrayLabel("measured_wrench", {"cx", "cy", "cz", "fx", "fy", "fz"},
      //                         [this]() { return this->measuredWrench().vector(); }),
      mc_rtc::gui::ArrayLabel("estimatedContactWrench", {"cx", "cy", "cz", "fx", "fy", "fz"},
                              [this]() { return this->estimatedContactWrench().vector(); }),
      mc_rtc::gui::NumberInput(
          "Velocity filter gain", [this]() { return velFilterGain_; }, [this](double g) { velFilterGain(g); }));
  // Don't add TransformTask as target configuration is different
  TrajectoryTaskGeneric::addToGUI(gui);
}

void ObserverbasedAdmittanceTask::addToSolver(mc_solver::QPSolver & solver)
{
  timestep_ = solver.dt();
  TransformTask::addToSolver(solver);
}

void ObserverbasedAdmittanceTask::getestimatedExternalWrench()
{
  if(exportExternalWrench_)
  {
    if(datastore.has(robot_.name() + "::estimatedExternalWrench"))
    {
      estimatedExternalWrench_centroid_ = datastore.get<sva::ForceVecd>(robot_.name() + "::estimatedExternalWrench");
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
    if(datastore.has(robot_.name() + "::estimatedContactWrench_" + std::to_string(i)))
    {
      estimatedContactWrench_ =
          datastore.get<sva::ForceVecd>(robot_.name() + "::estimatedContactWrench_" + std::to_string(i));
      estimatedContactWrench_ = replaceForceTorque(estimatedContactWrench_);
    }
  }
  else { mc_rtc::log::error("[ObserverbasedAdmittanceTask] No EstimatedContactWrench is exported"); }
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
} // namespace force

} // namespace mc_tasks

namespace
{

static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "admittance",
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
      auto rIndex = robotIndexFromConfig(config, solver.robots(), "admittance");
      auto t =
          std::make_shared<mc_tasks::force::ObserverbasedAdmittanceTask>(solver.robots().robot(rIndex).frame(frame));
      t->reset();
      t->load(solver, config);
      return t;
    });
} // namespace
