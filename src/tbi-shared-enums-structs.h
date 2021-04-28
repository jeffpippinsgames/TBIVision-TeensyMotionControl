//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------
// tbi-shared-enums-structs.cpp
// Jeff Pippins 2021
// TBailey Inc.
//
// Shared Enums and Structs 
// This program is intended for use in the TBIVision Seam Tracking Software.
//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------
//A Note on Buffers and Serial Comm.
//All Serial Streams are Terminated with the 0x00 byte. No Serial Data Type will Contain the 0x00 value.
//This Value is reserved for the null terminition byte


#ifndef __TBIVISION_SHARED_ENUMS_AND_STRUCTS__
#define __TBIVISION_SHARED_ENUMS_AND_STRUCTS__

//--------------------------------------------------------------------------------------------------------------------
// Various #defines for Serial Buffer Sizes.
#define TBI_COMMAND_BUFFER_SIZE 8
#define TBI_CONTROL_STATUS_BUFFER_SIZE 18
//--------------------------------------------------------------------------------------------------------------------




//--------------------------------------------------------------------------------------------------------------------
//Command Bytes
enum SerialCommand_t{ TBI_CMD_DELIMITER_CR = 0x0D, TBI_CMD_DELMIITER_LF = 0x0A, TBI_CMD_SEND_WATCHDOG = 0x01, TBI_CMD_STOP_MOVEMENT = 0x02, TBI_CMD_JOG_UP = 0x03, TBI_CMD_JOG_DOWN = 0x04, TBI_CMD_JOG_LEFT = 0x05, TBI_CMD_JOG_RIGHT = 0x06, TBI_SET_X_ERROR = 0x07,
                      TBI_SET_Z_ERROR = 0x08, TBI_CMD_SEND_X_ERROR = 0x09, TBI_CMD_SEND_Z_ERROR = 0x0B, TBI_CMD_SEND_X_LIMIT_TRIPPED = 0x0C, TBI_CMD_SEND_Z_LIMIT_TRIPPED = 0x0E, TBI_CMD_SEND_X_POSITIION = 0x0F, TBI_CMD_SEND_Z_POSITION = 0x10, 
                      TBI_CMD_SET_X_POSITION = 0x11, TBI_CMD_SET_Z_POSITION = 0x12, TBI_CMD_SEND_X_LIMIT_STATUS = 0x13, TBI_CMD_SEND_Z_LIMIT_STATUS = 0x14, TBI_CMD_SET_KP_XCONTROL = 0x15, TBI_CMD_SET_KD_XCONTROL = 0x16, TBI_CMD_SET_KI_XCONTROL = 0x17,
                      TBI_CMD_SEND_KP_XCONTROL = 0x18, TBI_CMD_SEND_KD_XCONTROL = 0x19, TBI_CMD_SEND_KI_XCONTROL = 0x1A, TBI_CMD_PID_ON = 0x1B, TBI_CMD_PID_OFF = 0x1C, TBI_CMD_SEND_STATUS = 0x1D, TBI_CMD_HOME_X = 0x1E, TBI_CMD_HOME_Z = 0x1F, 
                      TBI_CMD_SET_KP_ZCONTROL = 0x20, TBI_CMD_SET_KD_ZCONTROL = 0x21, TBI_CMD_SET_KI_ZCONTROL = 0x22, TBI_CMD_SEND_KP_ZCONTROL = 0x23, TBI_CMD_SEND_KD_ZCONTROL = 0x24, TBI_CMD_SEND_KI_ZCONTROL = 0x25, TBI_CMD_TOGGLE_LASER_POWER = 0x26};

/*
Serial Command Key

The Teensy Appends the Standard Serial Object. See Teensy for more detaills. 

The Serial Commands Are Shared WIth The Host Application in order to facilitate control of the PID Based Motor Control.
The Command Control Schema will be one leading byte which represents the Command Followed By Status Bytes and or raw data.
The Schema is a fixed data buffer size schema of max size BUFFER_SIZE. It is up to the application to appropriatly manage the 
buffer and serial send and recieve management.

TBI_CMD_DELIMITER_CR: Command Delimiter
TBI_CMD_DELMIITER_LF: Command Delimiter
TBI_CMD_SEND_WATCHDOG: Instructs the control to send a TBI_STATUS_WATCHDOG_OK signal
TBI_CMD_STOP_MOVEMENT: Stops all Motor Movement
TBI_CMD_JOG_UP: Makes the controller jog up
TBI_CMD_JOG_DOWN: Makes the controller jog down
TBI_CMD_JOG_LEFT: Makes the controller jog left
TBI_CMD_JOG_RIGHT: Makes the controller jog right.
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





//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//Status Types
/*
A Note About Status Update Command. (TBI_CMD_SEND_STATUS 0x1D)

The Byte Sequence of Status Updates is as follows.

[Byte 0] - Is the TBI_CMD_SEND_STATUS 0x1D
[Byte 1] - The Controller Motion Status. Of Type MotionStatus_t.
[Byte 2] - The Controller Control Mode, PID or Manual. ie either TBI_CONTROL_MODE_MANUAL_MODE or TBI_CONTROL_MODE_PID_MODE. Of Type ControlMode_t
[Byte 3] - The Controller X Axis Plus Limit Switch State - Either TBI_LIMIT_SWITCH_STATE_OK or BI_LIMIT_SWITCH_STATE_TRIPPED. Of Type LimitSwitchState_t 
[Byte 4] - The Controller X Axis Minus Limit Switch State - Either TBI_LIMIT_SWITCH_STATE_OK or BI_LIMIT_SWITCH_STATE_TRIPPED. Of Type LimitSwitchState_t 
[Byte 5] - The Controller Z Axis Plus Limit Switch State - Either TBI_LIMIT_SWITCH_STATE_OK or BI_LIMIT_SWITCH_STATE_TRIPPED. Of Type LimitSwitchState_t 
[Byte 6] - The Controller Z Axis Minus Switch State - Either TBI_LIMIT_SWITCH_STATE_OK or BI_LIMIT_SWITCH_STATE_TRIPPED. Of Type LimitSwitchState_t 
[Byte 7] - The Controller Operation Status Condition. Either TBI_OPERATION_OK or TBI_OPERATION_ERROR. Of Type OperationStatus_t.
[Byte 8] - Error Code. Of type ControlErrorCode. Of Type ControlErrorCode_t
[Byte 9,10,11,12] X Position of Controller as a 4 byte float.
[Byte 13,14,15,16] Z Position of Controller as a 4 byte float.
[Byte 17] Has Controller been homed. Either TBI_HOMING_STATUS_HOMED or TBI_HOMING_STATUS_NOT_HOMED. Of Type HomingStatus_t

This mandates the TBI_CONTROL_STATUS_BUFFER_SIZE be 18 bytes.
*/
//--------------------------------------------------------------------------------------------------------------------
enum MotionStatus_t {TBI_MOTION_STATUS_IDLE = 0x00, TBI_MOTION_STATUS_MOVING_PID = 0x01, TBI_MOTION_STATUS_JOGGING = 0x02, TBI_MOTION_STATUS_GENERAL_ERROR = 0x03, TBI_MOTION_STATUS_LIMIT_TRIPPED = 0x04};
enum ControlMode_t {TBI_CONTROL_MODE_MANUAL_MODE = 0x01, TBI_CONTROL_MODE_PID_MODE = 0x02};
enum LimitSwitchState_t {TBI_LIMIT_SWITCH_STATE_TRIPPED = 0x01, TBI_LIMIT_SWITCH_STATE_OK = 0x02};
enum OperationStatus_t {TBI_OPERATION_OK = 0x01, TBI_OPERATION_ERROR = 0x02};
enum ControlErrorCode_t {TBI_ERROR_ALL_OK = 0x01, TBI_ERROR_LIMIT_TRIPPED = 0x02, TBI_ERROR_CANNOT_PROCESS_LAST_MOVEMENT = 0x03, TBI_ERROR_LOST_SERIAL_NO_DTR = 0x04};
enum HomingStatus_t {TBI_HOMING_STATUS_HOMED = 0x01, TBI_HOMING_STATUS_NOT_HOMED = 0x02};

#endif //__TBIVISION_SHARED_ENUMS_AND_STRUCTS__