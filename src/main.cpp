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
//--------------------------------------------------------------------------------------------------------------------




//--------------------------------------------------------------------------------------------------------------------
//Defines
#define __DEBUG
//--------------------------------------------------------------------------------------------------------------------




//--------------------------------------------------------------------------------------------------------------------
//Enumerated Types
enum PIDState_t{TBI_PID_STATE_OFF, TBI_PID_STATE_ON};
enum SerialCommand_t{TBI_CMD_DELIMITER_CR = 0x0D, TBI_CMD_DELMIITER_LF = 0x0A, TBI_CMD_SEND_WATCHDOG = 0x01, TBI_CMD_STOP_MOVEMENT = 0x02, TBI_CMD_JOG_UP = 0x03, TBI_CMD_JOG_DOWN = 0x04, TBI_JOG_LEFT = 0x05, TBI_JOG_RIGHT = 0x06, TBI_SET_X_ERROR = 0x07,
                      TBI_SET_Z_ERROR = 0x08, TBI_CMD_SEND_X_ERROR = 0x09, TBI_CMD_SEND_Z_ERROR = 0x0B, TBI_CMD_SEND_X_LIMIT_TRIPPED = 0x0C, TBI_CMD_SEND_Z_LIMIT_TRIPPED = 0x0E, TBI_CMD_SEND_X_POSITIION = 0x0F, TBI_CMD_SEND_Z_POSITION = 0x10, 
                      TBI_CMD_SET_X_POSITION = 0x11, TBI_CMD_SET_Z_POSITION = 0x12, TBI_CMD_SEND_X_LIMIT_STATUS = 0x13, TBI_CMD_SEND_Z_LIMIT_STATUS = 0x14, TBI_CMD_SET_KP_XCONTROL = 0x15, TBI_CMD_SET_KD_XCONTROL = 0x16, TBI_CMD_SET_KI_XCONTROL = 0x17,
                      TBI_CMD_SEND_KP_XCONTROL = 0x18, TBI_CMD_SEND_KD_XCONTROL = 0x19, TBI_CMD_SEND_KI_XCONTROL = 0x20, TBI_CMD_PID_ON = 0x21, TBI_CMD_PID_OFF = 0x22, TBI_CMD_SEND_STATUS = 0x23, TBI_CMD_HOME_X = 0x24, TBI_CMD_HOME_Z = 0x25, 
                      TBI_CMD_SET_KP_ZCONTROL = 0x26, TBI_CMD_SET_KD_ZCONTROL = 0x27, TBI_CMD_SET_KI_ZCONTROL = 0x28, TBI_CMD_SEND_KP_ZCONTROL = 0x29, TBI_CMD_SEND_KD_ZCONTROL = 0x30, TBI_CMD_SEND_KI_ZCONTROL = 0x31};

/*
Serial Command Key

The Teensy Appends the Standard Serial Object. See Teensy for more detaills. 

The Serial Commands Are Shared WIth The Host Application in order to facilitate control of the PID Based Motor Control.
The Command Control Schema will be one leading byte followed by any data appropriate to the command.
The Delimiter will be LF Char 

TBI_CMD_DELIMITER_CR: Command Delimiter
TBI_CMD_DELMIITER_LF: Command Delimiter
TBI_CMD_SEND_WATCHDOG: Instructs the control to send a TBI_STATUS_WATCHDOG_OK signal
TBI_CMD_STOP_MOVEMENT: Stops all Motor Movement
TBI_CMD_JOG_UP: Makes the controller jog up
TBI_CMD_JOG_DOWN: Makes the controller jog down
TBI_JOG_LEFT: Makes the controller jog left
TBI_JOG_RIGHT: Makes the controller jog right.
TBI_SET_X_ERROR: Sets the controller PID error value for the X axis
TBI_SET_Z_ERROR: Sets the controller PID error value for the Z axis
TBI_CMD_SEND_X_ERROR: Instructs the controller to send the current PID X axis error value
TBI_CMD_SEND_Z_ERROR: Instructs the controller to send the current PID Z axis error value
TBI_CMD_SEND_X_LIMIT_TRIPPED: Sends the TBI_STATUS_X_LIMIT_TRIPPED status.
TBI_CMD_SEND_Z_LIMIT_TRIPPED: Sends the TBI_STATUS_Z_LIMIT_TRIPPED status.
TBI_CMD_SEND_X_POSITIION: Sends the position of the X axis
TBI_CMD_SEND_Z_POSITION = Sends the position of the Z axis
TBI_CMD_SET_X_POSITION = Sets the current position of the X axis
TBI_CMD_SET_Z_POSITION = Sets the current position of the Z axis
TBI_CMD_SEND_X_LIMIT_STATUS: Sends either TBI_STATUS_X_LIMIT_TRIPPED or TBI_STATUS_X_LIMIT_OK depending on the X axis limit switch state.
TBI_CMD_SEND_Z_LIMIT_STATUS: Sends either TBI_STATUS_Z_LIMIT_TRIPPED or TBI_STATUS_Z_LIMIT_OK depending on the Z axis limit switch state.
TBI_CMD_SET_KP_XCONTROL: Sets the PID KP for the X axis PID controller
TBI_CMD_SET_KD_XCONTROL: Sets the PID KD for the X axis PID controller
TBI_CMD_SET_KI_XCONTROL: Sets the PID KI for the X axis PID controller
TBI_CMD_SEND_KP_XCONTROL: Sends the PID KP for the X axis PID controller
TBI_CMD_SEND_KD_XCONTROL: Sends the PID KD for the X axis PID controller
TBI_CMD_SEND_KI_XCONTROL: Sends the PID KI for the X axis PID controller
TBI_CMD_PID_ON: Turns on the PID control of the motors
TBI_CMD_PID_OFF: Turns off the PID control of the motors
TBI_CMD_SEND_STATUS: Sends the status of the control as a sequence of ControlStatus_t bytes.
TBI_CMD_HOME_X: Performs a homing of the X axis
TBI_CMD_HOME_Z: Peroforms a homing of the Z axis
TBI_CMD_SET_KP_ZCONTROL: Sets the PID KP for the Z axis PID controller
TBI_CMD_SET_KD_ZCONTROL: Sets the PID KD for the Z axis PID controller
TBI_CMD_SET_KI_ZCONTROL: Sets the PID KI for the Z axis PID controller
TBI_CMD_SEND_KP_ZCONTROL: Sends the PID KP for the Z axis PID controller
TBI_CMD_SEND_KD_ZCONTROL: Sends the PID KD for the Z axis PID controller
TBI_CMD_SEND_KI_ZCONTROL: Sends the PID KI for the Z axis PID controller
*/
enum ControlStatus_t {TBI_STATUS_IDLE = 0x00, TBI_STATUS_MOVING = 0x01, TBI_STATUS_X_LIMIT_TRIPPED = 0x02, TBI_STATUS_Z_LIMIT_TRIPPED = 0x03, TBI_STATUS_LIMIT_TRIPPED = 0x04, TBI_STATUS_GENERAL_ERROR = 0x05, TBI_STATUS_WATCHDOG_OK = 0x06, TBI_STATUS_X_LIMIT_OK = 0x07, TBI_STATUS_Z_LIMIT_OK = 0x08};
//--------------------------------------------------------------------------------------------------------------------




//--------------------------------------------------------------------------------------------------------------------
//Global Variables
PIDState_t pid_state = TBI_PID_STATE_OFF;

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

  //If there is a Serial Connection Then Process The Commands.
  if(Serial.dtr() && Serial)
  {
    //Check Serial Data
    while(Serial.available())
    {
      
    }

    //Process Command

    //Update PID

    //Update Motion Control

  }
  else //Bad JooJoo. We lost the Serial Connection. TURN IT OFF
  {
   pid_state = TBI_PID_STATE_OFF;
   if(x_controller.isRunning()) x_controller.stop();
   if(z_controller.isRunning()) z_controller.stop();
  }
}
//--------------------------------------------------------------------------------------------------------------------




//--------------------------------------------------------------------------------------------------------------------
//Command Processing Functions.
//--------------------------------------------------------------------------------------------------------------------




//--------------------------------------------------------------------------------------------------------------------
//Data Parsing Functions
int parseIntData(byte *_data)
{
  return 0;
}

float parseFloatData(byte *_data)
{
  return 0.0;
}

double parseDoubleData(byte *_data)
{
  return 0.0;
}

long parseLongData(byte *_data)
{
  return 0;
}
//--------------------------------------------------------------------------------------------------------------------

