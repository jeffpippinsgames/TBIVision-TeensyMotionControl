#ifndef __TBIVISION_HARWARE_DEFINES__
#define __TBIVISION_HARDWARE_DEFINES__

#define __DEBUG 

#define TBI_SERIALBAUD 115200

#define TBI_LOGICLEVELOEPIN 14
#define TBI_XSTEPPIN 15
#define TBI_XDIRPIN 16
#define TBI_XENABLEPIN 17
#define TBI_XLIMITPIN 18
#define TBI_ZSTEPPIN 19
#define TBI_ZDIRPIN  20
#define TBI_ZENABLEPIN 21
#define TBI_ZLIMITPIN 22
#define TBI_LASERRELAYPIN 23


#define TBI_XMAXSPEED 28000//Steps Per Sec
#define TBI_XPULLINSPEED 5 //Initial Speed Without Accel. 
#define TBI_XPULLOUTSPEED 5 //Initial Speed Without Accel. 
#define TBI_XMAXACCEL 120000//Steps Per Sec ^2
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