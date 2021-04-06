//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------
// TBIVision-TeensyMotionControl.cpp
// Jeff Pippins 2021
// TBailey Inc.
//
// Main Program For the Teensy 3.2 SBC
// This program is intended for use in the TBIVision Seam Tracking Software.
//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------




//--------------------------------------------------------------------------------------------------------------------
//Includes
#include <Arduino.h>
#include <TeensyStep.h>
#include <PID_v1.h> //For function documentation see:  http://playground.arduino.cc/Code/PIDLibrary
#include "harware_defines.h"
#include "tbi-shared-enums-structs.h"
#include "tbilimitswitches.hpp"
#include "tbicommandmanager.hpp"
#include "tbicontrolstatuscontainer.hpp"
//--------------------------------------------------------------------------------------------------------------------



//--------------------------------------------------------------------------------------------------------------------
//Global Variables
TBICommandManager command_manager;
TBILimitSwitches limit_switches;
TBIControlStatusContainer control_status;

Stepper x_motor(TBI_XSTEPPIN, TBI_XDIRPIN);
Stepper z_motor(TBI_ZSTEPPIN, TBI_ZDIRPIN);
RotateControl x_controller;
RotateControl z_controller;

double xPID_setpoint, xPID_input, xPID_output;
double x_Kp=2, x_Ki=5, x_Kd=1;
PID xPID(&xPID_input, &xPID_output, &xPID_setpoint, x_Kp, x_Ki, x_Kd, P_ON_E, DIRECT);

double zPID_setpoint, zPID_input, zPID_output;
double z_Kp=2, z_Ki=5, z_Kd=1;
PID zPID(&zPID_input, &zPID_output, &zPID_setpoint, z_Kp, z_Ki, z_Kd, P_ON_E, DIRECT);
//--------------------------------------------------------------------------------------------------------------------




//--------------------------------------------------------------------------------------------------------------------
//setup() function. Initializes the Hardware of the MCU
void setup() 
{
  //Setup Limit Switches
  pinMode(TBI_XLIMITPINPLUS, INPUT_PULLUP);
  pinMode(TBI_ZLIMITPINPLUS, INPUT_PULLUP);
  pinMode(TBI_XLIMITPINMINUS, INPUT_PULLUP);
  pinMode(TBI_ZLIMITPINMINUS, INPUT_PULLUP);
  limit_switches.UpdateLimitSwitchStates(&control_status);

  //Turn off LED Until Inititalized
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  //The Teensy Runs at USB Speeds. Serial.begin is there for compatibility.
  Serial.begin(9600);
  //X Motor Setup
  x_motor.setMaxSpeed(TBI_XMAXSPEED);
  x_motor.setPullInSpeed(TBI_XPULLINSPEED);
  x_motor.setPullInOutSpeed(TBI_XPULLOUTSPEED, TBI_XPULLOUTSPEED);
  x_motor.setAcceleration(TBI_XMAXACCEL);
  x_motor.setStepPinPolarity(TBI_XDRIVERPINPOLARITY);
  x_motor.setInverseRotation(TBI_XINVERTDIRPIN);
  //Z Motor Setup
  z_motor.setMaxSpeed(TBI_ZMAXSPEED);
  z_motor.setPullInSpeed(TBI_ZPULLINSPEED);
  z_motor.setPullInOutSpeed(TBI_ZPULLOUTSPEED, TBI_ZPULLOUTSPEED);
  z_motor.setAcceleration(TBI_ZMAXACCEL);
  z_motor.setStepPinPolarity(TBI_ZDRIVERPINPOLARITY);
  z_motor.setInverseRotation(TBI_ZINVERTDIRPIN);
  //
  x_motor.setPosition(0);
  z_motor.setPosition(0);
  //PID Setup
  xPID.SetMode(MANUAL); //Turn PID Controller Off
  zPID.SetMode(MANUAL); //Turn PID Controller Off
  xPID.SetSampleTime(TBI_XPIDSAMPLETIME);
  zPID.SetSampleTime(TBI_ZPIDSAMPLETIME);

  x_controller.stop();
  z_controller.stop();
  //Make Sure Serial Connection is Connected and Buffer is Flushed
  while(!Serial || !Serial.dtr()) delay(250);
  while(Serial.available()) Serial.read();
  //Turn on the Teensy LED To Indicate the System is Setup and Communicating.
  digitalWrite(LED_BUILTIN, HIGH);
  
  Serial.println("TBIVision-TeensyMotionControl Inititalzed and Connected. Ready For Commands");
}
//--------------------------------------------------------------------------------------------------------------------



//--------------------------------------------------------------------------------------------------------------------
//loop() Function. Program Main Loop
void loop()
 {

   control_status.clearErrors();
   limit_switches.UpdateLimitSwitchStates(&control_status);

  //If there is a Serial Connection Then Process The Commands.
  if(Serial.dtr() && Serial)
  {
    //digitalWrite(LED_BUILTIN, HIGH); //Make Sure LED Is On. Everything is Fine.
    command_manager.processCommandData(&x_controller, &z_controller, &x_motor, &z_motor, &limit_switches, &control_status);
    //Update PID

    //Update Motion Control
  }
  else //Bad JooJoo. We lost the Serial Connection. TURN IT OFF
  {
    digitalWrite(LED_BUILTIN, LOW); // Turn Off LED. Bad JooJoo....
    control_status.SetOperationStatus(TBI_OPERATION_ERROR);
    control_status.SetErrorCode(TBI_ERROR_LOST_SERIAL_NO_DTR);
    control_status.SetControlMode(TBI_CONTROL_MODE_MANUAL_MODE);
    control_status.SetMotionStatus(TBI_MOTION_STATUS_IDLE);
    if(x_controller.isRunning()) x_controller.stop();
    if(z_controller.isRunning()) z_controller.stop();
  }

}
//--------------------------------------------------------------------------------------------------------------------



//--------------------------------------------------------------------------------------------------------------------
//Data Parsing Functions
int parseIntData(byte *_data)
{
  return 0;
}
//--------------------------------------------------------------------------------------------------------------------
float parseFloatData(byte *_data)
{
  return 0.0;
}
//--------------------------------------------------------------------------------------------------------------------
double parseDoubleData(byte *_data)
{
  return 0.0;
}
//--------------------------------------------------------------------------------------------------------------------
long parseLongData(byte *_data)
{
  return 0;
}
//--------------------------------------------------------------------------------------------------------------------