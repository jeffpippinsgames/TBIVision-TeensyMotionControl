#include <Arduino.h>
#include <TeensyStep.h>
#include <PID_v1.h>
#include "harware_defines.h"


Stepper x_motor(TBI_XSTEPPIN, TBI_XDIRPIN), z_motor(TBI_ZSTEPPIN, TBI_ZDIRPIN);
RotateControl x_controller;
RotateControl Z_controller;





void setup() {
  Serial.begin(TBI_SERIALBAUD);
  //X Motor Setup
  x_motor.setMaxSpeed(TBI_XMAXSPEED);
  x_motor.setPullInSpeed(TBI_XPULLINSPEED);
  x_motor.setPullInOutSpeed(TBI_XPULLOUTSPEED);
  x_motor.setAcceleration(TBI_XMAXACCEL);
  x_motor.setStepPinPolarity(TBI_XDRIVERPINPOLARITY);
  x_motor.setInverseRotation(TBI_XINVERTDIRPIN);
  //Z Motor Setup
  z_motor.setMaxSpeed(TBI_ZMAXSPEED);
  z_motor.setPullInSpeed(TBI_ZPULLINSPEED);
  z_motor.setPullInOutSpeed(TBI_ZPULLOUTSPEED);
  z_motor.setAcceleration(TBI_ZMAXACCEL);
  z_motor.setStepPinPolarity(TBI_ZDRIVERPINPOLARITY);
  z_motor.setInverseRotation(TBI_ZINVERTDIRPIN);
  //
  x_motor.setPosition(0);
  z_motor.setPosition(0);

  

}

void loop() {
  // put your main code here, to run repeatedly:
}