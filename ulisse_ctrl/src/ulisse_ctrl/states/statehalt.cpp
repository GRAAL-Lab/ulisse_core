#include "ulisse_ctrl/states/statehalt.hpp"

namespace  ulisse {

namespace states {

StateHalt::StateHalt()
{

}

StateHalt::~StateHalt()
{

}

fsm::retval StateHalt::Execute()
{
    return fsm::ok;
}



}

}

