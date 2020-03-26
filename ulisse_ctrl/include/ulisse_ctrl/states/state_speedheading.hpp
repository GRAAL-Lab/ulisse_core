#ifndef ULISSE_CTRL_STATESPEEDHEADING_HPP
#define ULISSE_CTRL_STATESPEEDHEADING_HPP

#include "ulisse_ctrl/states/genericstate.hpp"
#include "ulisse_ctrl/tasks/SafetyBoundaries.h"

namespace ulisse {

namespace states {

class StateSpeedHeading : public GenericState {
  std::chrono::system_clock::time_point t_start_, t_now_;
  std::chrono::seconds total_elapsed_;

protected:
  std::shared_ptr<ikcl::AlignToTarget> angularPositionTask_;
  std::shared_ptr<ikcl::LinearVelocity> linearVelocityTask_;
   std::shared_ptr<ikcl::SafetyBoundaries> safetyBoundariesTask_;


  double surgeRef;
  double headingError;

public:
  StateSpeedHeading();
  virtual ~StateSpeedHeading();
  virtual fsm::retval OnEntry();
  virtual fsm::retval Execute();
  void ResetTimer();

  void SetAngularPositionTask(
      std::shared_ptr<ikcl::AlignToTarget> angularPositionTask);
  void SetLinearVelocityTask(
      std::shared_ptr<ikcl::LinearVelocity> linearVelocityTask);
  void SetSafetyBoundariesTask(
      std::shared_ptr<ikcl::SafetyBoundaries> safetyBoundariesTask);
};
} // namespace states
} // namespace ulisse

#endif // ULISSE_CTRL_STATEMOVE_HPP
