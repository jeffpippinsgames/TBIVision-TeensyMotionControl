#ifndef __TBICOMMANDMANAGER__
#define __TBICOMMANDMANAGER__

//--------------------------------------------------------------------------------------------------------------------
//Includes
#include <Arduino.h>
#include "harware_defines.h"
#include "tbi-shared-enums-structs.h"
#include <TeensyStep.h>
#include <PID_v1.h> //For function documentation see:  http://playground.arduino.cc/Code/PIDLibrary
#include "tbilimitswitches.hpp"
#include "tbicontrolstatuscontainer.hpp"
//--------------------------------------------------------------------------------------------------------------------


class TBICommandManager
{
    private:
        byte incomming_command_buffer[TBI_COMMAND_BUFFER_SIZE];
        byte outgoing_command_buffer[TBI_COMMAND_BUFFER_SIZE];
        byte status_buffer[TBI_CONTROL_STATUS_BUFFER_SIZE];
    
    public:
        TBICommandManager();
        void processCommandData(RotateControl* _x_control, RotateControl* _z_control, Stepper* _x_motor, Stepper* _z_motor, TBILimitSwitches* _limit_switches, TBIControlStatusContainer* _control_status);
    private:
        void doMotorRotation(RotateControl *_rotcontroller, Stepper* _motor, TBIControlStatusContainer *_control_status, TBILimitSwitches* _limitswitches, float _speed_fraction);
        void doStopMovement(RotateControl *_x_control, RotateControl* _z_control, TBIControlStatusContainer* _control_status);
        
};

#endif