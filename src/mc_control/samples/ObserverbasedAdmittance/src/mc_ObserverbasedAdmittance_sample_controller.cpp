/*
 * Copyright 2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "mc_ObserverbasedAdmittance_sample_controller.h"
#include <SpaceVecAlg/SpaceVecAlg>
#include "mc_control/MCController.h"
#include <Eigen/src/Core/Matrix.h>

ObserverbasedAdmittanceSampleController::ObserverbasedAdmittanceSampleController(mc_rbdyn::RobotModulePtr rm,
                                                                                 double dt,
                                                                                 const mc_rtc::Configuration & config,
                                                                                 Backend backend)
: mc_control::fsm::Controller(rm, dt, config, backend)
{
  datastore().make_call(
      "KinematicAnchorFrame::" + robot().name(), [](const mc_rbdyn::Robot & robot)
      { return sva::interpolate(robot.surfacePose("LeftFoot"), robot.surfacePose("RightFoot"), 0.5); });

  auto stabiConf = robot().module().defaultLIPMStabilizerConfiguration();

  lipm_stabilizer_ptr_ = std::make_shared<mc_tasks::lipm_stabilizer::StabilizerTask>(
      solver().robots(), solver().realRobots(), robots().robotIndex(), stabiConf.leftFootSurface,
      stabiConf.rightFootSurface, stabiConf.torsoBodyName, solver().dt());
  lipm_stabilizer_ptr_->reset();
  lipm_stabilizer_ptr_->setContacts(
      {mc_tasks::lipm_stabilizer::ContactState::Left, mc_tasks::lipm_stabilizer::ContactState::Right});

  lipm_stabilizer_ptr_->configure(stabiConf);
  solver().addTask(lipm_stabilizer_ptr_);
}

void ObserverbasedAdmittanceSampleController::reset(const mc_control::ControllerResetData & reset_data)
{
  Controller::reset(reset_data);
  auto handForceConfig = mc_rtc::gui::ForceConfig(mc_rtc::gui::Color(0., 1., 0.));
  handForceConfig.force_scale *= 10;

  lipm_stabilizer_ptr_.reset();

  gui()->addElement({"Forces"},
                    mc_rtc::gui::Force(
                        "RightHand", handForceConfig, [this]() { return robot().surfaceWrench("RightHand"); },
                        [this]() { return robot().surfacePose("RightHand"); }));
}

bool ObserverbasedAdmittanceSampleController::run()
{
  t_ += timeStep;

  return Controller::run();
}
