/*
 * Copyright 2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/api.h>
#include <mc_control/fsm/Controller.h>
#include <mc_control/mc_controller.h>
#include <mc_tasks/lipm_stabilizer/StabilizerTask.h>

struct MC_CONTROL_DLLAPI ObserverbasedAdmittanceSampleController : public mc_control::fsm::Controller
{
  ObserverbasedAdmittanceSampleController(mc_rbdyn::RobotModulePtr rm,
                                          double dt,
                                          const mc_rtc::Configuration & config,
                                          Backend backend);

  void reset(const mc_control::ControllerResetData & reset_data) override;
  bool run() override;

  void supported_robots(std::vector<std::string> & out) const override { out = {"jvrc1", "hrp5_p"}; }

protected:
  double t_ = 0; ///< Elapsed time since the controller started

private:
  std::shared_ptr<mc_tasks::lipm_stabilizer::StabilizerTask> lipm_stabilizer_ptr_;
};
