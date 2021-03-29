#include "tbicommandmanager.hpp"

TBICommandManager::TBICommandManager()
{

}

void TBICommandManager::processCommandData(RotateControl* _x_control, RotateControl* _z_control, Stepper* _x_motor, Stepper* _z_motor, TBILimitSwitches* _limitswitches, TBIControlStatusContainer* _control_status)
{
  
  if(Serial.available() == TBI_COMMAND_BUFFER_SIZE)
  {
      Serial.readBytes((char*)incomming_command_buffer, TBI_COMMAND_BUFFER_SIZE);
      switch(incomming_command_buffer[0])
      {
        //----------------------------------------------------------
        case TBI_CMD_STOP_MOVEMENT:
          this->doStopMovement(_x_control, _z_control, _control_status);
          break;
        //----------------------------------------------------------
        case TBI_CMD_JOG_UP:
          if(_limitswitches->GetZPlusLimitSwitchState() == TBI_LIMIT_SWITCH_STATE_TRIPPED)
          {
            this->doStopMovement(_x_control, _z_control, _control_status);
          } 
          else
          {
            this->doMotorRotation(_z_control, _z_motor, _control_status, _limitswitches, .5);
          }
          break;
        //----------------------------------------------------------
        case TBI_CMD_JOG_DOWN:
          if(_limitswitches->GetZMinusLimitSwitchState() == TBI_LIMIT_SWITCH_STATE_TRIPPED)
          {
            this->doStopMovement(_x_control, _z_control, _control_status);
          } 
          else
          {
            this->doMotorRotation(_z_control, _z_motor, _control_status, _limitswitches, -.5);
          }
          break;
        //----------------------------------------------------------
        case TBI_CMD_JOG_LEFT:
          if(_limitswitches->GetXMinusLimitSwitchState() == TBI_LIMIT_SWITCH_STATE_TRIPPED)
          {
            this->doStopMovement(_x_control, _z_control, _control_status);
          } 
          else
          {
            this->doMotorRotation(_x_control, _x_motor, _control_status, _limitswitches, .5);
          }
          break;
        //----------------------------------------------------------
        case TBI_CMD_JOG_RIGHT:
          if(_limitswitches->GetXPLusLimitSwitchState() == TBI_LIMIT_SWITCH_STATE_TRIPPED)
          {
            this->doStopMovement(_x_control, _z_control, _control_status);
          } 
          else
          {
            this->doMotorRotation(_x_control, _x_motor, _control_status, _limitswitches, -.5);
          }
          break; 
        //---------------------------------------------------------- 
      }
  }
}

//--------------------------------------------------------------------------------------------------------------------
//Command Processing Functions.
//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------
void TBICommandManager::doMotorRotation(RotateControl *_rotcontroller, Stepper* _motor, TBIControlStatusContainer *_control_status, TBILimitSwitches* _limitswitches, float _speed_fraction)
{
    if(_control_status->GetMotionStatus() != TBI_MOTION_STATUS_IDLE) return;

  _control_status->SetMotionStatus(TBI_MOTION_STATUS_JOGGING);
  _rotcontroller->overrideSpeed(.5);
  _rotcontroller->rotateAsync(*_motor);
}
//--------------------------------------------------------------------------------------------------------------------
void TBICommandManager::doStopMovement(RotateControl *_x_control, RotateControl* _z_control, TBIControlStatusContainer* _control_status)
{
  _x_control->stop();
  _z_control->stop();
  _control_status->SetMotionStatus(TBI_MOTION_STATUS_IDLE);
}
