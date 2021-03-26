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
//--------------------------------------------------------------------------------------------------------------------




//--------------------------------------------------------------------------------------------------------------------
//Defines
#define __DEBUG
#define BUFFER_SIZE 8
//--------------------------------------------------------------------------------------------------------------------




//--------------------------------------------------------------------------------------------------------------------
//Global Variables
PIDState_t pid_state = TBI_PID_STATE_OFF;
MotionStatus_t motion_status = TBI_MOTION_STATUS_IDLE;


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



byte incomming_serial_buffer[BUFFER_SIZE];
byte outgoing_serial_buffer[BUFFER_SIZE];

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
    
    clearIncommingSerialBuffer(); //Clear Serial Buffers
    clearOutgoingSerialBuffer(); //Clear Serial Buffers
    digitalWrite(LED_BUILTIN, HIGH); //Make Sure LED Is On. Everything is Fine.
    //Get The Next Command
    processIncommingSerialCommand(incomming_serial_buffer);
    //Update PID

    //Update Motion Control
    

  }
  else //Bad JooJoo. We lost the Serial Connection. TURN IT OFF
  {
    digitalWrite(LED_BUILTIN, LOW); // Turn Off LED. Bad JooJoo....
    pid_state = TBI_PID_STATE_OFF;
    if(x_controller.isRunning()) x_controller.stop();
    if(z_controller.isRunning()) z_controller.stop();
  }

}
//--------------------------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------------------------
//Command Processing Functions.
//--------------------------------------------------------------------------------------------------------------------
void processIncommingSerialCommand(byte* _data)
{
  if(!_data) return; 
  if(Serial.available() == BUFFER_SIZE)
  {
      Serial.readBytes((char*)_data, BUFFER_SIZE);
      switch(_data[0])
      {
        case TBI_CMD_STOP_MOVEMENT:
          processStopMovementCmd();
          break;
        case TBI_CMD_JOG_UP:
          processJogUpCmd();
          break;
        case TBI_CMD_JOG_DOWN:
          processJogDownCmd();
          break;
        case TBI_CMD_JOG_LEFT:
          processJogLeftCmd();
          break;
        case TBI_CMD_JOG_RIGHT:
          processJogRightCmd();
          break;
        
      }


  
  }
}

void processJogUpCmd()
{
  if(motion_status != TBI_MOTION_STATUS_IDLE) return;
  motion_status = TBI_MOTION_STATUS_JOGGING;
  z_controller.overrideSpeed(.5);
  z_controller.rotateAsync(z_motor);
}

void processJogDownCmd()
{
  if(motion_status != TBI_MOTION_STATUS_IDLE) return;
  motion_status = TBI_MOTION_STATUS_JOGGING;
  z_controller.overrideSpeed(-.5);
  z_controller.rotateAsync(z_motor);
}

void processJogLeftCmd()
{
  if(motion_status != TBI_MOTION_STATUS_IDLE) return;
  motion_status = TBI_MOTION_STATUS_JOGGING;
  x_controller.overrideSpeed(.5);
  x_controller.rotateAsync(x_motor);
}

void processJogRightCmd()
{
  if(motion_status != TBI_MOTION_STATUS_IDLE) return;
  motion_status = TBI_MOTION_STATUS_JOGGING;
  x_controller.overrideSpeed(-.5);
  x_controller.rotateAsync(x_motor);  
}

void processStopMovementCmd()
{
  motion_status = TBI_MOTION_STATUS_IDLE;
  x_controller.stop();
  z_controller.stop();
}





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




//--------------------------------------------------------------------------------------------------------------------
//Misc Functions
void clearIncommingSerialBuffer()
{
    for(int i = 0; i < BUFFER_SIZE; ++i) incomming_serial_buffer[i] = 0x00;
}

void clearOutgoingSerialBuffer()
{
  for(int i = 0; i < BUFFER_SIZE; ++i) outgoing_serial_buffer[i] = 0x00;
}

void processLimitSwitches()
{
  
  

}
//--------------------------------------------------------------------------------------------------------------------