#include "ulisse_ctrl/states/state_speedheading.hpp"
#include "ulisse_ctrl/fsm_defines.hpp"
#include <ulisse_ctrl/geometry_defines.h>
#include <ulisse_ctrl/ulisse_definitions.h>

namespace ulisse {

namespace states {

void StateSpeedHeading::ResetTimer() {
  t_start_ = std::chrono::system_clock::now();
}

StateSpeedHeading::StateSpeedHeading() {}

StateSpeedHeading::~StateSpeedHeading() {}

void StateSpeedHeading::SetAngularPositionTask(
    std::shared_ptr<ikcl::AbsoluteAxisAlignment> absoluteAxisAlignmentTask) {
  absoluteAxisAlignmentTask_ = absoluteAxisAlignmentTask;
}

void StateSpeedHeading::SetLinearVelocityTask(
    std::shared_ptr<ikcl::LinearVelocity> linearVelocityTask) {
  linearVelocityTask_ = linearVelocityTask;
}

void StateSpeedHeading::SetSafetyBoundariesTask(
    std::shared_ptr<ikcl::SafetyBoundaries> safetyBoundariesTask) {
  safetyBoundariesTask_ = safetyBoundariesTask;
}

fsm::retval StateSpeedHeading::OnEntry() {
  actionManager_->SetAction(ulisse::action::speed_heading, true);

  return fsm::ok;
}

fsm::retval StateSpeedHeading::Execute() {
  for (auto &task : unifiedHierarchy_) {
    try {
      task->Update();
    } catch (tpik::ExceptionWithHow &e) {
      std::cerr << "UPDATE TASK EXCEPTION" << std::endl;
      std::cerr << "who " << e.what() << " how: " << e.how() << std::endl;
    }
  }

  CheckRadioController();

  t_now_ = std::chrono::system_clock::now();
  total_elapsed_ =
      std::chrono::duration_cast<std::chrono::seconds>(t_now_ - t_start_);

  if (goalCxt_->cmdTimeout != 0 &&
      total_elapsed_.count() > goalCxt_->cmdTimeout) {
    std::cout << "Speed Heading Timeout reached!" << std::endl;
    fsm_->ExecuteCommand(ulisse::commands::ID::halt);
  }
  // check if the activaction function of safetyB is no null
  if (safetyBoundariesTask_->GetInternalActivationFunction().norm() > 0) {

    // Use the activation function of SafetyB to activate the
    // angluarPositionTask
    Eigen::VectorXd Aexternal;

    Aexternal =
        safetyBoundariesTask_->GetInternalActivationFunction().maxCoeff() *
        Aexternal.setOnes(absoluteAxisAlignmentTask_->GetTaskSpace());

    absoluteAxisAlignmentTask_->SetExternalActivationFunction(Aexternal);
    absoluteAxisAlignmentTask_->SetAxisAlignment(Eigen::Vector3d(1, 0, 0),
                                                 ulisse::robotModelID::ASV);
    absoluteAxisAlignmentTask_->SetDirectionAlignment(
        safetyBoundariesTask_->GetAlignVector(), rml::FrameID::WorldFrame);

    safetyBoundariesTask_->SetDesiredVelocity(Eigen::Vector3d{1.0, 0, 0.0});
  } else {
    absoluteAxisAlignmentTask_->SetAxisAlignment(Eigen::Vector3d(1, 0, 0), ulisse::robotModelID::ASV);
    absoluteAxisAlignmentTask_->SetDirectionAlignment(
        Eigen::Vector3d(cos(goalCxt_->goalHeading), sin(goalCxt_->goalHeading),
                        0),
        rml::FrameID::WorldFrame);
    linearVelocityTask_->SetVelocity(
        Eigen::Vector3d(goalCxt_->goalSurge, 0, 0));
  }

  std::cout << "STATE SPEED HEADING " << std::endl;
  std::cout << "Goal Heading: " << goalCxt_->goalHeading << std::endl;
  std::cout << "Goal Surge: " << goalCxt_->goalSurge << std::endl;

  return fsm::ok;
}
} // namespace states
} // namespace ulisse
