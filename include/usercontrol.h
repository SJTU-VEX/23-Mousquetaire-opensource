#ifndef USERCONTROL_H_
#define USERCONTROL_H_

void baseControl();
void baseControlbyHeading();
void intakerControl();
void intakerLift();
void anglerControl();
void triggerControl();
void flyWheelControl();

void usrCtlThread(void * childThread);
void autoCtlThread();
void usercontrol();
#endif