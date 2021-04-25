#include "tbilimitswitches.hpp"

TBILimitSwitches::TBILimitSwitches()
{
    pinMode(TBI_XLIMITPIN, INPUT_PULLUP);
    pinMode(TBI_ZLIMITPIN, INPUT_PULLUP);
}

void TBILimitSwitches::UpdateLimitSwitchStates(TBIControlStatusContainer* _controlstatus)
{
    x_limit_state = TBI_LIMIT_SWITCH_STATE_OK;
    z_limit_state = TBI_LIMIT_SWITCH_STATE_OK;

}

LimitSwitchState_t TBILimitSwitches::GetXLimitSwitchState()
{
    return this->x_limit_state;
}

LimitSwitchState_t TBILimitSwitches::GetZLimitSwitchState()
{
    return this->z_limit_state;
}