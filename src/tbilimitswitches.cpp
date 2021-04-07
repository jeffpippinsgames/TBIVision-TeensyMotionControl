#include "tbilimitswitches.hpp"

TBILimitSwitches::TBILimitSwitches()
{
    pinMode(TBI_XLIMITPINPLUS, INPUT_PULLUP);
    pinMode(TBI_XLIMITPINMINUS, INPUT_PULLUP);
    pinMode(TBI_ZLIMITPINPLUS, INPUT_PULLUP);
    pinMode(TBI_ZLIMITPINMINUS, INPUT_PULLUP);
}

void TBILimitSwitches::UpdateLimitSwitchStates(TBIControlStatusContainer* _controlstatus)
{
    x_limit_plus_state = TBI_LIMIT_SWITCH_STATE_OK;
    x_limit_minus_state = TBI_LIMIT_SWITCH_STATE_OK;
    z_limit_plus_state = TBI_LIMIT_SWITCH_STATE_OK;
    z_limit_minus_state = TBI_LIMIT_SWITCH_STATE_OK;

/*
        //All Limit Switches are Active Low.
    switch(digitalReadFast(TBI_XLIMITPINPLUS))
    {
        case LOW:
            x_limit_plus_state = TBI_LIMIT_SWITCH_STATE_TRIPPED;
            _controlstatus->SetOperationStatus(TBI_OPERATION_ERROR);
            _controlstatus->SetErrorCode(TBI_ERROR_LIMIT_TRIPPED);
            break;
        case HIGH:
            x_limit_plus_state = TBI_LIMIT_SWITCH_STATE_OK;
            break;
    }

    switch(digitalReadFast(TBI_XLIMITPINMINUS))
    {
        case LOW:
            x_limit_minus_state = TBI_LIMIT_SWITCH_STATE_TRIPPED;
            _controlstatus->SetOperationStatus(TBI_OPERATION_ERROR);
            _controlstatus->SetErrorCode(TBI_ERROR_LIMIT_TRIPPED);
            break;
        case HIGH:
            x_limit_minus_state = TBI_LIMIT_SWITCH_STATE_OK;
            break;
    }

    switch(digitalReadFast(TBI_ZLIMITPINPLUS))
    {
        case LOW:
            z_limit_plus_state = TBI_LIMIT_SWITCH_STATE_TRIPPED;
            _controlstatus->SetOperationStatus(TBI_OPERATION_ERROR);
            _controlstatus->SetErrorCode(TBI_ERROR_LIMIT_TRIPPED);
            break;
        case HIGH:
            z_limit_plus_state = TBI_LIMIT_SWITCH_STATE_OK;
            break;
    }

    switch(digitalReadFast(TBI_ZLIMITPINMINUS))
    {
        case LOW:
            z_limit_minus_state = TBI_LIMIT_SWITCH_STATE_TRIPPED;
            _controlstatus->SetOperationStatus(TBI_OPERATION_ERROR);
            _controlstatus->SetErrorCode(TBI_ERROR_LIMIT_TRIPPED);
            break;
        case HIGH:
            z_limit_minus_state = TBI_LIMIT_SWITCH_STATE_OK;
            break;
    }
    */
}

LimitSwitchState_t TBILimitSwitches::GetXPLusLimitSwitchState()
{
    return this->x_limit_plus_state;
}

LimitSwitchState_t TBILimitSwitches::GetXMinusLimitSwitchState()
{
    return this->x_limit_minus_state;
}

LimitSwitchState_t TBILimitSwitches::GetZPlusLimitSwitchState()
{
    return this->z_limit_plus_state;
}
LimitSwitchState_t TBILimitSwitches::GetZMinusLimitSwitchState()
{
    return this->z_limit_minus_state;
}