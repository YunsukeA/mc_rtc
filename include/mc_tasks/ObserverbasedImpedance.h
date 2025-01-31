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
};

} // namespace force
} // namespace mc_tasks