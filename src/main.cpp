#include <Arduino.h>
#include <TeensyStep.h>
#include <PID_v1.h> //For function documentation see:  http://playground.arduino.cc/Code/PIDLibrary
#include "harware_defines.h"

enum ControlState{TBI_PID_OFF, TBI_PID_ON};

ControlState m_state = TBI_PID_OFF;

Stepper x_motor(TBI_XSTEPPIN, TBI_XDIRPIN), z_motor(TBI_ZSTEPPIN, TBI_ZDIRPIN);
RotateControl x_controller;
RotateControl Z_controller;

double xPID_setpoint, xPID_input, xPID_output;
double x_Kp=2, x_Ki=5, x_Kd=1;
PID xPID(&xPID_input, &xPID_output, &xPID_setpoint, x_Kp, x_Ki, x_Kd, P_ON_E, DIRECT);

double zPID_setpoint, zPID_input, zPID_output;
double z_Kp=2, z_Ki=5, z_Kd=1;
PID zPID(&zPID_input, &zPID_output, &zPID_setpoint, z_Kp, z_Ki, z_Kd, P_ON_E, DIRECT);


void setup() {
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

void loop() {
  // put your main code here, to run repeatedly:
  if(m_state == TBI_PID_ON)
  {
    if(xPID.Compute())
    {

    }
    if(zPID.Compute())
    {

    }
  }
  else
  {

  }

}