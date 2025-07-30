/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/api.h>
#include <mc_control/mc_controller.h>

#include <mc_tasks/CoMTask.h>
#include <mc_tasks/ObserverbasedImpedanceTask.h>
#include "mc_control/MCController.h"
// #include <mc_tasks/ImpedanceTask.h>

namespace mc_control
{

struct MC_CONTROL_DLLAPI ObserverbasedImpedanceSampleController : public MCController
{
  ObserverbasedImpedanceSampleController(std::shared_ptr<mc_rbdyn::RobotModule> robot,
                                         double dt,
                                         const mc_rtc::Configuration & config,
                                         Backend backend);
  void reset(const ControllerResetData & reset_data) override;
  bool run() override;
  void stop() override;

protected:
  void addGUI();
  Eigen::Vector3d circleTrajectory(double angle);

protected:
  double angle_ = 0.0;
  std::shared_ptr<mc_tasks::CoMTask> comTask_;
  std::shared_ptr<mc_tasks::force::ObserverbasedImpedanceTask> ObserverbasedImpedanceTask_;
  // std::shared_ptr<mc_tasks::force::ImpedanceTask> impedanceTask_;
  double radius_ = 0.2;
  double speed_ = 0.5;
  Eigen::Vector3d center_ = Eigen::Vector3d(0.3, 0.5, 1.0);
  Eigen::Matrix3d orientation_ = Eigen::Matrix3d::Identity();

  mc_control::MCController & ctl_;
};

} // namespace mc_control
