#ifndef AUTOAIMING_H_
#define AUTOAIMING_H_

extern bool isAiming;
extern bool isAiming_vision;
extern int offset;

int autonAiming();
int autonAiming_vision();

// Basic control
void setAimingStatus(bool _input);
void setAimingStatus(bool _input, int _offset);
void setVisionAimingStatus(bool _input);
void setVisionAimingStatus(bool _input, int _offset);
#endif