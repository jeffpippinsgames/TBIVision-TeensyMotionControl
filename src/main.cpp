#include <Arduino.h>
#include <TeensyStep.h>
#include <PID_v1.h> //For function documentation see:  http://playground.arduino.cc/Code/PIDLibrary
#include "harware_defines.h"

#define __DEBUG

enum PIDState_t{TBI_PID_STATE_OFF, TBI_PID_STATE_ON};
enum SerialCommand_t{TBI_CMD_DELIMITER_CR = 0x0D, TBI_CMD_DELMIITER_LF = 0x0A, TBI_CMD_ALIVE = 0x01, TBI_CMD_MOVE_OFF = 0x02, TBI_CMD_JOG_UP = 0x03, TBI_CMD_JOG_DOWN = 0x04, TBI_JOG_LEFT = 0x05, TBI_JOG_RIGHT = 0x06, TBI_SET_X_ERROR = 0x07,
                      TBI_SET_Z_ERROR = 0x08, TBI_CMD_SEND_X_ERROR = 0x09, TBI_CMD_SEND_Z_ERROR = 0x0B, TBI_CMD_X_LIMIT_TRIPPED = 0x0C, TBI_CMD_Z_LIMIT_TRIPPED = 0x0E, TBI_CMD_SEND_X_POSITIION = 0x0F, TBI_CMD_SEND_Z_POSITION = 0x10, 
                      TBI_CMD_SET_X_POSITION = 0x11, TBI_CMD_SET_Z_POSITION = 0x12, TBI_CMD_SEND_X_LIMIT_STATUS = 0x13, TBI_CMD_SEND_Z_LIMIT_STATUS = 0x14, TBI_CMD_SET_KP = 0x15, TBI_CMD_SET_KD = 0x16, TBI_CMD_SET_KI = 0x17, TBI_CMD_SEND_KP = 0x18,
                      TBI_CMD_SEND_KD = 0x19, TBI_CMD_SEND_KI = 0x20, TBI_CMD_PID_ON = 0x21, TBI_CMD_PID_OFF = 0x22, TBI_CMD_SEND_PID_STATUS = 0x23, TBI_CMD_HOME_X = 0x24, TBI_CMD_HOME_Z = 0x25};

/*
Serial Command Key

The Teensy Appends the Standard Serial Object. See Teensy for more detaills. 

The Serial Commands Are Shared WIth The Host Application in order to facilitate control of the PID Based Motor Control.
The Command Control Schema will be one leading byte followed by any data appropriate to the command.
The Delimiter will be LF Char 

*/

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


void setup() 
{
  Serial.begin(TBI_SERIALBAUD);
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
}

void loop()
 {
   //Safety Function
   while (!Serial)
  {
   //Stay Locked in Loop If No Serial Connection.
   //This will Ensure Nothing Bad Happens
   //Make Sure The PID is Off
   pid_state = TBI_PID_STATE_OFF;
   //Make Sure All Motors are OFF
   if(x_controller.isRunning()) x_controller.stop();
   if(z_controller.isRunning()) z_controller.stop();
  }

   //Check Serial Data
   if(Serial.available())
   {

   }

  

}

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
  
}


