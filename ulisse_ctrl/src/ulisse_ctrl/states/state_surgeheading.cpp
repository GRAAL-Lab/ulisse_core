#include "ulisse_ctrl/states/state_surgeheading.hpp"
#include "ulisse_ctrl/ulisse_defines.hpp"

namespace ulisse {

namespace states {

    StateSurgeHeading::StateSurgeHeading() : goalSurge(0.0), goalHeading (0.0)
    {
        maxHeadingError_ = M_PI / 16;
        minHeadingError_ = M_PI / 64;
    }

    StateSurgeHeading::~StateSurgeHeading() { }

    void StateSurgeHeading::ResetTimer()
    {
        tStart_ = std::chrono::system_clock::now();
    }

    void StateSurgeHeading::SetSurgeHeading(double speed, double heading)
    {
        goalSurge = speed;
        goalHeading = heading;
    }

    bool StateSurgeHeading::ConfigureStateFromFile(libconfig::Config& confObj)
    {
        const libconfig::Setting& root = confObj.getRoot();
        const libconfig::Setting& states = root["states"];

        const libconfig::Setting& state = states.lookup(ulisse::states::ID::surgeheading);

        if (!ctb::GetParam(state, maxHeadingError_, "maxHeadingError"))
            return false;
        if (!ctb::GetParam(state, minHeadingError_, "minHeadingError"))
            return false;
        return true;
    }

    fsm::retval StateSurgeHeading::OnEntry()
    {
        // Set tasks
        //safetyBoundariesTask_ = std::dynamic_pointer_cast<ikcl::SafetyBoundaries>(tasksMap.find(ulisse::task::asvSafetyBoundaries)->second.task);
        //absoluteAxisAlignmentSafetyTask_ = std::dynamic_pointer_cast<ikcl::AbsoluteAxisAlignment>(tasksMap.find(ulisse::task::asvAbsoluteAxisAlignmentSafety)->second.task);
        linearVelocityTask_ = std::dynamic_pointer_cast<ikcl::LinearVelocity>(tasksMap.find(ulisse::task::asvLinearVelocity)->second.task);
        absoluteAxisAlignmentTask_ = std::dynamic_pointer_cast<ikcl::AbsoluteAxisAlignment>(tasksMap.find(ulisse::task::asvAbsoluteAxisAlignment)->second.task);

        if (actionManager->SetAction(ulisse::action::surge_heading, true)) {
            return fsm::ok;
        } else {
            return fsm::fail;
        }
    }

    fsm::retval StateSurgeHeading::Execute()
    {
        CheckRadioController();

        tNow_ = std::chrono::system_clock::now();
        totalElapsed_ = std::chrono::duration_cast<std::chrono::seconds>(tNow_ - tStart_);

        if (timeout != 0 && totalElapsed_.count() > timeout) {
            std::cout << "Speed Heading Timeout reached!" << std::endl;
            fsm_->ExecuteCommand(ulisse::commands::ID::halt);
        }

        // SafetyBoundaries task: it's a velocity task base on the distance from the boundaries. The behaviour that has to achive is align to
        // a desired escape directon and to generate a desired velocity. To do this we use the task AbsoluteAxisAlignment to cope with
        // the align behavior activated in function of the internal activation function of the safety task.

/*        safetyBoundariesTask_->VehiclePosition() = ctrlData->inertialF_linearPosition;

        Eigen::MatrixXd Aexternal;

        Aexternal = safetyBoundariesTask_->InternalActivationFunction().maxCoeff()* Aexternal.setIdentity(
                        absoluteAxisAlignmentSafetyTask_->TaskSpace(), absoluteAxisAlignmentSafetyTask_->TaskSpace());
        absoluteAxisAlignmentSafetyTask_->ExternalActivationFunction() = Aexternal;

        safetyBoundariesTask_->ExternalActivationFunction() = 1.0 * Eigen::MatrixXd::Identity(safetyBoundariesTask_->TaskSpace(),
                                                                  safetyBoundariesTask_->TaskSpace());

        absoluteAxisAlignmentSafetyTask_->SetRobotAxis2Align(Eigen::Vector3d(1, 0, 0), ulisse::robotModelID::ASV);
        absoluteAxisAlignmentSafetyTask_->SetDirectionAlignment(safetyBoundariesTask_->GetAlignVector(rml::FrameID::WorldFrame),
                                                                                                   rml::FrameID::WorldFrame);

        // To avoid the case in which the error between the goal heading and the current heading is too big
        // we activate the the cartesian distance through the gain based on a bell-shaped function on the heading error

        // Compute the heading error
        double headingErrorsafety = absoluteAxisAlignmentSafetyTask_->ControlVariable().norm();

        // Compute the gain of the safety task
        double taskGainSafety = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, 1.0, headingErrorsafety);

        // Set the gain of the cartesian distance task
        safetyBoundariesTask_->TaskParameter().gain = taskGainSafety * safetyBoundariesTask_->TaskParameter().conf_gain;
*/

        //////     surgeheading task     /////////
        absoluteAxisAlignmentTask_->SetRobotAxis2Align(Eigen::Vector3d(1, 0, 0), ulisse::robotModelID::ASV);
        absoluteAxisAlignmentTask_->SetDirectionAlignment(Eigen::Vector3d(
                                          cos(goalHeading), sin(goalHeading), 0), rml::FrameID::WorldFrame);

        linearVelocityTask_->SetReferenceRate(Eigen::Vector3d(goalSurge, 0, 0), robotModel->BodyFrameID());

        // Compute the heading error
        double headingError = absoluteAxisAlignmentTask_->ControlVariable().norm();

        // Compute the gain of the cartesian distance
        double taskGain = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, 1, headingError);

        // Set the gain of the cartesian distance task
        linearVelocityTask_->ExternalActivationFunction() = taskGain *
            Eigen::MatrixXd::Identity(linearVelocityTask_->TaskSpace(), linearVelocityTask_->TaskSpace());


        /*std::cout << "* STATE SPEED HEADING *" << std::endl;

        Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", " / ", "", "", "", ";");

        std::cout << "minHeadingError_: " << minHeadingError_ << std::endl;
        std::cout << "maxHeadingError_: " << maxHeadingError_ << std::endl;
        std::cout << "headingError(Safety): " << absoluteAxisAlignmentSafetyTask_->ControlVariable().norm() << std::endl;
        std::cout << "taskGain(Safety): " << taskGainSafety << std::endl;
        std::cout << "- - -" << std::endl;
        std::cout << "headingError: " << absoluteAxisAlignmentTask_->ControlVariable().norm() << std::endl;
        std::cout << "linearVel activation Gain: " << taskGain << std::endl;
        std::cout << "- - -" << std::endl;

        std::cout << "linearVelocityTask_->ReferenceRate(): " << linearVelocityTask_->ReferenceRate().format(CommaInitFmt) << std::endl;

        std::cout << "linearVelocityTask_->InternalActivation(): " << linearVelocityTask_->InternalActivationFunction().format(CommaInitFmt) << std::endl;
        std::cout << "linearVelocityTask_->ExternalActivation(): " << linearVelocityTask_->ExternalActivationFunction().format(CommaInitFmt) << std::endl;
        std::cout << "linearVelocityTask_->gain: " << linearVelocityTask_->TaskParameter().gain << std::endl;
        std::cout << "linearVelocityTask_->sauration: " << linearVelocityTask_->TaskParameter().saturation << std::endl;
        std::cout << "-----------------------" << std::endl;*/

        return fsm::ok;
    }
} // namespace states
} // namespace ulisse



