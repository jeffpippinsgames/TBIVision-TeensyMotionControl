#include "tbicontrolstatuscontainer.hpp"

TBIControlStatusContainer::TBIControlStatusContainer()
{
    this->motion_status = TBI_MOTION_STATUS_IDLE;
    this->operation_status = TBI_OPERATION_OK;
    this->control_mode = TBI_CONTROL_MODE_MANUAL_MODE;
    this->homing_status = TBI_HOMING_STATUS_NOT_HOMED;
}

void TBIControlStatusContainer::SetMotionStatus(MotionStatus_t _status)
{
    motion_status = _status;
}
void TBIControlStatusContainer::SetOperationStatus(OperationStatus_t _status)
{
    operation_status = _status;
}
void TBIControlStatusContainer::SetHomingStatus(HomingStatus_t _status)
{
    homing_status = _status;
}
void TBIControlStatusContainer::SetControlMode(ControlMode_t _mode)
{
    control_mode = _mode;
}
void TBIControlStatusContainer::SetErrorCode(ControlErrorCode_t _code)
{
    error_code = _code;
}
MotionStatus_t TBIControlStatusContainer::GetMotionStatus()
{
    return motion_status;
}
OperationStatus_t TBIControlStatusContainer::GetOperationStatus()
{
    return operation_status;
}
HomingStatus_t TBIControlStatusContainer::GetHomingStatus()
{
    return homing_status;
}
ControlMode_t TBIControlStatusContainer::GetControlMode()
{
    return control_mode;
}
ControlErrorCode_t TBIControlStatusContainer::GetErrorCode()
{
    return error_code;
}
void TBIControlStatusContainer::clearErrors()
{
    this->operation_status = TBI_OPERATION_OK;
    this->error_code = TBI_ERROR_ALL_OK;
}