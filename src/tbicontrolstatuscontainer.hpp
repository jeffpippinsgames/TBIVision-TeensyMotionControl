#ifndef __TBICONTROLSTATUSCONTAINER__
#define __TBICONTROLSTATUSCONTAINER__

//--------------------------------------------------------------------------------------------------------------------
//Includes
#include <Arduino.h>
#include "harware_defines.h"
#include "tbi-shared-enums-structs.h"
#include <TeensyStep.h>
#include <PID_v1.h> //For function documentation see:  http://playground.arduino.cc/Code/PIDLibrary


class TBIControlStatusContainer
{
    private:
        MotionStatus_t motion_status;
        OperationStatus_t operation_status;
        HomingStatus_t homing_status;
        ControlMode_t control_mode;
        ControlErrorCode_t error_code;
    public:
        TBIControlStatusContainer();
        void SetMotionStatus(MotionStatus_t _status);
        void SetOperationStatus(OperationStatus_t _status);
        void SetHomingStatus(HomingStatus_t _status);
        void SetControlMode(ControlMode_t _mode);
        void SetErrorCode(ControlErrorCode_t _error_code);
        MotionStatus_t GetMotionStatus();
        OperationStatus_t GetOperationStatus();
        HomingStatus_t GetHomingStatus();
        ControlMode_t GetControlMode();
        ControlErrorCode_t GetErrorCode();
        void clearErrors();
};



#endif