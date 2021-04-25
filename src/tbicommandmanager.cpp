#include "tbicommandmanager.hpp"

TBICommandManager::TBICommandManager()
{

}

void TBICommandManager::processCommandData(RotateControl* _x_control, RotateControl* _z_control, Stepper* _x_motor, Stepper* _z_motor, TBILimitSwitches* _limitswitches, TBIControlStatusContainer* _control_status)
{
  
  if(Serial.available())
  {

      byte _cmd;
      _cmd = (byte)Serial.read();
      switch(_cmd)
      {
        //----------------------------------------------------------
        case TBI_CMD_STOP_MOVEMENT:
          this->doStopMovement(_x_control, _z_control, _control_status);
          digitalWrite(LED_BUILTIN, HIGH);
          break;
        //----------------------------------------------------------
        case TBI_CMD_JOG_UP:
        
          if(_limitswitches->GetZLimitSwitchState() == TBI_LIMIT_SWITCH_STATE_TRIPPED)
          {
            this->doStopMovement(_z_control, _z_control, _control_status);
          } 
          else
          {
            _z_motor->setMaxSpeed(TBI_ZMAXSPEED);
            this->doMotorRotation(_z_control, _z_motor, _control_status, _limitswitches);
          }
          break;
        //----------------------------------------------------------
        case TBI_CMD_JOG_DOWN:
          if(_limitswitches->GetZLimitSwitchState() == TBI_LIMIT_SWITCH_STATE_TRIPPED)
          {
            this->doStopMovement(_z_control, _z_control, _control_status);
          } 
          else
          {
            _z_motor->setMaxSpeed(-TBI_ZMAXSPEED);
            this->doMotorRotation(_z_control, _z_motor, _control_status, _limitswitches);
          }
          break;
          
        //----------------------------------------------------------
        case TBI_CMD_JOG_LEFT:
          if(_limitswitches->GetXLimitSwitchState() == TBI_LIMIT_SWITCH_STATE_TRIPPED)
          {
            this->doStopMovement(_x_control, _x_control, _control_status);
          } 
          else
          {
              _x_motor->setMaxSpeed(TBI_XMAXSPEED);
              this->doMotorRotation(_x_control, _x_motor, _control_status, _limitswitches);
          }
          break;
        //----------------------------------------------------------
        case TBI_CMD_JOG_RIGHT:
          if(_limitswitches->GetXLimitSwitchState() == TBI_LIMIT_SWITCH_STATE_TRIPPED)
          {
            this->doStopMovement(_x_control, _x_control, _control_status);
          } 
          else
          {
            _x_motor->setMaxSpeed(-TBI_XMAXSPEED);
            this->doMotorRotation(_x_control, _x_motor, _control_status, _limitswitches);
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
void TBICommandManager::doMotorRotation(RotateControl *_rotcontroller, Stepper* _motor, TBIControlStatusContainer *_control_status, TBILimitSwitches* _limitswitches)
{
    if(_control_status->GetMotionStatus() != TBI_MOTION_STATUS_IDLE) return;
  _rotcontroller->rotateAsync(*_motor);
  _control_status->SetMotionStatus(TBI_MOTION_STATUS_JOGGING);
}
//--------------------------------------------------------------------------------------------------------------------
void TBICommandManager::doStopMovement(RotateControl *_x_control, RotateControl* _z_control, TBIControlStatusContainer* _control_status)
{
  _x_control->stop();
  _z_control->stop();
  _control_status->SetMotionStatus(TBI_MOTION_STATUS_IDLE);
}
