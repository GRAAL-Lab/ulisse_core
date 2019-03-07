/*
 * GenericEvent.cc
 *
 *  Created on: Nov 01, 2018
 *      Author: wanderfra
 */

#include "ulisse_ctrl/events/generic_event.hpp"

namespace ulisse {

namespace events {

    //uint16_t GenericEvent::count_ = 0;

    GenericEvent::GenericEvent()
    {
        //context_ = NULL;
    }

    GenericEvent::~GenericEvent()
    {
        //context_ = NULL;
    }

    void GenericEvent::SetCtrlContext(const std::shared_ptr<ControlContext>& ctrlCxt)
    {
        ctrlCxt_ = ctrlCxt;
    }

    /*int32_t GenericEvent::SetContext(context_t* context) {
	if (context != NULL) {
		if (context_ != NULL)
			ortos::DebugConsole::Write(ortos::LogLevel::warning, "GenericEvent::SetContext", "Overwriting existing context! (%p => %p)", context, context_);
		context_ = context;
		return ORTOS_RV_OK;
	}

	ortos::DebugConsole::Write(ortos::LogLevel::error, "GenericEvent::SetContext", "context argument is NULL");
	return ORTOS_RV_FAIL;
}
*/

    fsm::retval GenericEvent::Propagate(void)
    {
        /*eventInfo_.timestamp = ::om2ctrl::utils::GetTime();
        ortos::xcom::XCOMInterface* xcom = ortos::xcom::XCOMInterface::GetInstance();
        xcom->Synchronize();

        int ret = xcom->WriteIf(om2ctrl::fsm::events::topicnames::events, eventInfo_);
        if (ret != ORTOS_RV_OK) {
            ortos::DebugConsole::Write(ortos::LogLevel::error, "GenericEvent::Propagate",
                "Error writing topic %s (%d)", om2ctrl::fsm::events::topicnames::events, ret);
        }
        */

        std::cout << "Executing Event" << std::endl;
        return fsm::retval::ok;
    }
    /*
int32_t GenericEvent::Init(Event eventNumber) {
	eventInfo_.d.eventNumber = eventNumber;

	if (count_++ == 0) {
		ortos::DebugConsole::Write(ortos::LogLevel::info, "GenericEvent::Init", "First instance -> publishing event topic (%s)", om2ctrl::fsm::events::topicnames::events);
		ortos::xcom::XCOMInterface* xcom = ortos::xcom::XCOMInterface::GetInstance();

		int ret = xcom->Publish(om2ctrl::fsm::events::topicnames::events, eventInfo_);
		if (ret != ORTOS_RV_OK) {
			ortos::DebugConsole::Write(ortos::LogLevel::error, "GenericEvent::Init",
					"Error while publishing %s topic (%d)", om2ctrl::fsm::events::topicnames::events, ret);

			return ORTOS_RV_FAIL;
		}
	}

	return ORTOS_RV_OK;
}*/

} // namespace events

} // namespace ulisse
