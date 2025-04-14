/*
 * Copyright 2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "mc_ObserverbasedAdmittance_sample_controller.h"
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

  mc_rtc::log::info("LIPM Stabilizer Configuration: {}", stabiConf.torsoBodyName);
  mc_rtc::log::info("LIPM Stabilizer Configuration: {}", stabiConf.leftFootSurface);
  mc_rtc::log::info("LIPM Stabilizer Configuration: {}", stabiConf.rightFootSurface);
  mc_rtc::log::info("LIPM Stabilizer Configuration: {}", stabiConf.comHeight);

  auto lipm_stabilizer_ptr = std::make_shared<mc_tasks::lipm_stabilizer::StabilizerTask>(
      solver().robots(), solver().realRobots(), robots().robotIndex(), stabiConf.leftFootSurface,
      stabiConf.rightFootSurface, stabiConf.torsoBodyName, solver().dt());
  lipm_stabilizer_ptr->reset();
  lipm_stabilizer_ptr->setContacts(
      {mc_tasks::lipm_stabilizer::ContactState::Left, mc_tasks::lipm_stabilizer::ContactState::Right});
  solver().addTask(lipm_stabilizer_ptr);
}

void ObserverbasedAdmittanceSampleController::reset(const mc_control::ControllerResetData & reset_data)
{
  Controller::reset(reset_data);
  auto handForceConfig = mc_rtc::gui::ForceConfig(mc_rtc::gui::Color(0., 1., 0.));
  handForceConfig.force_scale *= 10;
  gui()->addElement({"Forces"},
                    mc_rtc::gui::Force(
                        "RightHand", handForceConfig, [this]() { return robot().surfaceWrench("RightHand"); },
                        [this]() { return robot().surfacePose("RightHand"); }));

  using Color = mc_rtc::gui::Color;

  gui()->addPlot(
      "RightFoot Force (t)", mc_rtc::gui::plot::X("t", [this]() { return t_; }),
      mc_rtc::gui::plot::Y("RF(x)", [this]() { return robot().surfaceWrench("RightFoot").force().x(); }, Color::Red));
  gui()->addPlot(
      "RightFoot Force (t)", mc_rtc::gui::plot::X("t", [this]() { return t_; }),
      mc_rtc::gui::plot::Y("RZ(y)", [this]() { return robot().surfaceWrench("RightFoot").force().z(); }, Color::Blue));

  gui()->addPlot(
      "LeftFoot Force (t)", mc_rtc::gui::plot::X("t", [this]() { return t_; }),
      mc_rtc::gui::plot::Y("LF(x)", [this]() { return robot().surfaceWrench("LeftFoot").force().x(); }, Color::Green));
  gui()->addPlot(
      "LeftFoot Force (t)", mc_rtc::gui::plot::X("t", [this]() { return t_; }),
      mc_rtc::gui::plot::Y("LF(z)", [this]() { return robot().surfaceWrench("LeftFoot").force().z(); }, Color::Yellow));
}

bool ObserverbasedAdmittanceSampleController::run()
{
  t_ += timeStep;

  mc_tasks::lipm_stabilizer::StabilizerTask stabilizerTask(solver().robots(), solver().realRobots(),
                                                           robots().robotIndex(), solver().dt());
  mc_tasks::lipm_stabilizer::StabilizerTask & stabilizer = stabilizerTask;

  datastore().make_call("KinematicAnchorFrame" + robot().name(),
                        [this](const mc_rbdyn::Robot & robot)
                        {
                          return sva::interpolate(robot.surfacePose("RightFootCenter"),
                                                  robot.surfacePose("LeftFootCenter"), leftFootRatio_);
                        });

  datastore().make_call("AnchorFrameReal", [&]() { return stabilizer.anchorFrame(solver().realRobot()); });

  return Controller::run();
}
