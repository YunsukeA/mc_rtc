#pragma once
#include <mc_tasks/ImpedanceTask.h>

#include <mc_rtc/log/Logger.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include "mc_control/MCController.h"

namespace mc_tasks
{

namespace force
{
struct MC_TASKS_DLLAPI ObserverbasedImpedanceTask : ImpedanceTask
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ObserverbasedImpedanceTask(const std::string & surfaceName,
                             const mc_rbdyn::Robots & robots,
                             mc_control::MCController * controller,
                             unsigned robotIndex,
                             double stiffness = 5.0,
                             double weight = 1000.0);

  ObserverbasedImpedanceTask(const mc_rbdyn::RobotFrame & frame,
                             mc_control::MCController * controller,
                             double stiffness = 5.0,
                             double weight = 1000.0);

  void update(mc_solver::QPSolver & solver) override;
  void load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) override;

  void getestimatedContactWrench(const std::string & surface);
  void getestimatedExternalWrench();

  // state-observation's value is [force, torque]^T style, but the ImpedanceTask's value is [torque, force]^T style
  sva::ForceVecd replaceForceTorque(sva::ForceVecd target);

  void addToLogger(mc_rtc::Logger & logger) override;
  sva::ForceVecd transformContactWrench(const sva::ForceVecd wrench,
                                        const std::string surface,
                                        const std::string forceSensor);
  sva::ForceVecd transformExternalWrench(const sva::ForceVecd wrench, const std::string surface);

protected:
  sva::ForceVecd wrenchError_ = sva::ForceVecd(Eigen::Vector6d::Zero());

private:
  std::string robot_;
  bool exportContactWrench_ = true;
  bool exportExternalWrench_ = true;
  int MaxContacts_ = 4;
  std::string usingWrench_;

  sva::ForceVecd estimatedContactWrench_;
  sva::ForceVecd estimatedContactWrench_sensorFrame_;

  sva::ForceVecd estimatedExternalWrench_centroid_;
  mc_control::MCController * controller_;

  sva::ForceVecd estimationError_;

  sva::PTransformd worldCentroidKinePTrans_;
};

} // namespace force
} // namespace mc_tasks