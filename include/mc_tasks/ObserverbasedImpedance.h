#pragma once
#include <mc_tasks/ImpedanceTask.h>

namespace mc_tasks
{

namespace force
{
struct MC_TASKS_DLLAPI ObserverbasedImpedance : ImpedanceTask
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ObserverbasedImpedance(const std::string & surfaceName,
                         const mc_rbdyn::Robots & robots,
                         unsigned robotIndex,
                         double stiffness = 5.0,
                         double weight = 1000.0);

  ObserverbasedImpedance(const mc_rbdyn::RobotFrame & frame, double stiffness = 5.0, double weight = 1000.0);

  void update(mc_solver::QPSolver & solver) override;
  void load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) override;

  void getestimatedContactWrench(const std::string & surface);
  void getestimatedExternalWrench();

  // state-observation's value is [force, torque]^T style, but the ImpedanceTask's value is [torque, force]^T style
  sva::ForceVecd replaceForceTorque(sva::ForceVecd target);

private:
  std::string robot_;
  bool exportContactWrench_ = false;
  bool exportExternalWrench_ = false;
  int MaxContacts_ = 4;

  sva::ForceVecd estimatedContactWrench_;

  sva::ForceVecd estimatedExternalWrench_centroid_;
};

} // namespace force
} // namespace mc_tasks