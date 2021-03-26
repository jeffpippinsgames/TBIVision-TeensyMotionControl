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


//--------------------------------------------------------------------------------------------------------------------
//Enumerated Types
enum PIDState_t{TBI_PID_STATE_OFF, TBI_PID_STATE_ON};
enum SerialCommand_t{ TBI_CMD_DELIMITER_CR = 0x0D, TBI_CMD_DELMIITER_LF = 0x0A, TBI_CMD_SEND_WATCHDOG = 0x01, TBI_CMD_STOP_MOVEMENT = 0x02, TBI_CMD_JOG_UP = 0x03, TBI_CMD_JOG_DOWN = 0x04, TBI_CMD_JOG_LEFT = 0x05, TBI_CMD_JOG_RIGHT = 0x06, TBI_SET_X_ERROR = 0x07,
                      TBI_SET_Z_ERROR = 0x08, TBI_CMD_SEND_X_ERROR = 0x09, TBI_CMD_SEND_Z_ERROR = 0x0B, TBI_CMD_SEND_X_LIMIT_TRIPPED = 0x0C, TBI_CMD_SEND_Z_LIMIT_TRIPPED = 0x0E, TBI_CMD_SEND_X_POSITIION = 0x0F, TBI_CMD_SEND_Z_POSITION = 0x10, 
                      TBI_CMD_SET_X_POSITION = 0x11, TBI_CMD_SET_Z_POSITION = 0x12, TBI_CMD_SEND_X_LIMIT_STATUS = 0x13, TBI_CMD_SEND_Z_LIMIT_STATUS = 0x14, TBI_CMD_SET_KP_XCONTROL = 0x15, TBI_CMD_SET_KD_XCONTROL = 0x16, TBI_CMD_SET_KI_XCONTROL = 0x17,
                      TBI_CMD_SEND_KP_XCONTROL = 0x18, TBI_CMD_SEND_KD_XCONTROL = 0x19, TBI_CMD_SEND_KI_XCONTROL = 0x20, TBI_CMD_PID_ON = 0x21, TBI_CMD_PID_OFF = 0x22, TBI_CMD_SEND_STATUS = 0x23, TBI_CMD_HOME_X = 0x24, TBI_CMD_HOME_Z = 0x25, 
                      TBI_CMD_SET_KP_ZCONTROL = 0x26, TBI_CMD_SET_KD_ZCONTROL = 0x27, TBI_CMD_SET_KI_ZCONTROL = 0x28, TBI_CMD_SEND_KP_ZCONTROL = 0x29, TBI_CMD_SEND_KD_ZCONTROL = 0x30, TBI_CMD_SEND_KI_ZCONTROL = 0x31};

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
enum ControlStatus_t {TBI_STATUS_IDLE = 0x00, TBI_STATUS_MOVING = 0x01, TBI_STATUS_X_LIMIT_TRIPPED = 0x02, TBI_STATUS_Z_LIMIT_TRIPPED = 0x03, TBI_STATUS_LIMIT_TRIPPED = 0x04, TBI_STATUS_GENERAL_ERROR = 0x05, TBI_STATUS_WATCHDOG_OK = 0x06, TBI_STATUS_X_LIMIT_OK = 0x07, TBI_STATUS_Z_LIMIT_OK = 0x08};
//--------------------------------------------------------------------------------------------------------------------
enum MotionStatus_t {TBI_MOTION_STATUS_IDLE = 0x00, TBI_MOTION_STATUS_MOVING_PID = 0x01, TBI_MOTION_STATUS_JOGGING = 0x02 };