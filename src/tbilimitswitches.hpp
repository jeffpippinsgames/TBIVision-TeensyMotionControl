#ifndef __TBILIMITSWITCHES__
#define __TBILIMITSWITCHES__

//--------------------------------------------------------------------------------------------------------------------
//Includes
#include <Arduino.h>
#include "harware_defines.h"
#include "tbi-shared-enums-structs.h"
#include "tbicontrolstatuscontainer.hpp"
#include <TeensyStep.h>
#include <PID_v1.h> //For function documentation see:  http://playground.arduino.cc/Code/PIDLibrary


class TBILimitSwitches
{
    private:
        LimitSwitchState_t x_limit_state;
        LimitSwitchState_t z_limit_state;
    public:
        TBILimitSwitches();
        void UpdateLimitSwitchStates(TBIControlStatusContainer* _controlstatus);
        LimitSwitchState_t GetXLimitSwitchState();
        LimitSwitchState_t GetZLimitSwitchState();
};

#endif