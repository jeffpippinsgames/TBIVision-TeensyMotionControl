#ifndef __TBIVISION_HARWARE_DEFINES__
#define __TBIVISION_HARDWARE_DEFINES__

#define __DEBUG 

#define TBI_SERIALBAUD 115200

#define TBI_XSTEPPIN 1
#define TBI_ZSTEPPIN 4
#define TBI_XDIRPIN 2
#define TBI_ZDIRPIN  5
#define TBI_XENABLEPIN 3
#define TBI_ZENABLEPIN 6
#define TBI_XLIMITPINPLUS 7
#define TBI_ZLIMITPINPLUS 8
#define TBI_XLIMITPINMINUS 9
#define TBI_ZLIMITPINMINUS 10
#define TBI_ERRORPIN 12

#define TBI_XMAXSPEED 35000//Steps Per Sec
#define TBI_XPULLINSPEED 5 //Initial Speed Without Accel. 
#define TBI_XPULLOUTSPEED 5 //Initial Speed Without Accel. 
#define TBI_XMAXACCEL 100000//Steps Per Sec ^2
#define TBI_XDRIVERPINPOLARITY LOW // The Stepper Motors Active Step and Dir Polarity
#define TBI_XINVERTDIRPIN false //Inverts the Dir Pin
#define TBI_XJOYSTICKSPEED 1200 //Default Joystick Speed

#define TBI_ZMAXSPEED 400 //Steps Per Sec
#define TBI_ZPULLINSPEED 5 //Initial Speed Without Accel. 
#define TBI_ZPULLOUTSPEED 5 //Initial Speed Without Accel. 
#define TBI_ZMAXACCEL 50 //Steps Per Sec ^2
#define TBI_ZDRIVERPINPOLARITY LOW // The Stepper Motors Active Step and Dir Polarity
#define TBI_ZINVERTDIRPIN false //Inverts the Dir Pin
#define TBI_ZJOYSTICKSPEED 1200 //Default Joystick Speed

#define TBI_XPIDSAMPLETIME 1 //PID Compute Sample Time;
#define TBI_ZPIDSAMPLETIME 1 //PID Computer Sample Time;



#endif //__TBIVISION_HARDWARE_DEFINES__