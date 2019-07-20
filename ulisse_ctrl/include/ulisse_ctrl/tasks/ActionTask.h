#ifndef __ACTIONTASK_H__
#define __ACTIONTASK_H__

#include <iostream>
#include <iostream>
#include <tpik/TPIKlib.h>

namespace tpik
{

/**
 * @brief ActionTask class, derived from the Abstract class tpik::Task
 */

    class ActionTask: public Task
    {

    public:
        ActionTask(const std::string ID, int taskSpace, int DoF);
        virtual ~ActionTask();
        void SetTaskParameter(TaskParameter taskParameter);
        void CheckInitialization() throw (ExceptionWithHow);
        TaskParameter GetTaskParameter();
        friend std::ostream& operator <<(std::ostream& os, ActionTask const& action)
        {
            return os <<"\033[1;37m"<<"ACTION TASK: " << action.ID_<<"\n" <<std::setprecision(4)<< "Internal Activation Function \n" << "\033[0m" << action.Ai_ << "\n"
                      << "\033[1;37m" << "Jacobian \n" << "\033[0m" << action.J_ << "\n"
                      << "\033[1;37m" << "Reference \n" << "\033[0m" << action.x_dot_ <<"\n"
                      << "\033[0m" << action.taskParameter_ <<"\n";
        }
    protected:
        /**
         * @brief Method updating the task reference.
         * @details Implementation of the pure virtual method of the base class tpik::EqualityTask, that updates the task reference
         */
        void UpdateReference() override;
        /**
         * @brief Method updating the task Jacobian.
         * @details Implementation of the pure virtual method of the base class tpik::EqualityTask, that updates the task jacobians.
         */
        void UpdateJacobian() override;
        /**
         * @brief  Method used to saturate the reference, such method must be called in the Update() method after the UpdateReference method.
         */
        void SaturateReference();
        /**
         * @brief Method saturating reference component wise  i.e. saturating each element of the vector individually.
         */
        void SaturateReferenceComponentWise();
        /**
         * @brief Method updating the internal activation function.
         * @details Implementation of the pure virtual method of the base class task.
         * For the equality tasks the internal activation function is equal to identity since their always active.
         * For this reason such method is called in the class constructor and there is no need to call it again.
         */
        void UpdateInternalActivationFunction() override;

        TaskParameter taskParameter_; //!< The tpik::TaskParameter.
        bool initializedTaskParameter_{false}; //!< The boolean used to check whether the task parameter have been initialized.

    };
}

#endif
