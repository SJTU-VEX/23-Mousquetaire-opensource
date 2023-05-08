#ifndef PID_H_
#define PID_H_

#include "my-timer.h"
#include "geometry.h"

class PID{
protected:
    float errorCurt, errorPrev, errorDev, errorInt;
    float P, I, D;
    bool firstTime;
    bool arrived;
    float kp, ki, kd;
    float target, errorTol, DTol;
    float IMax, IRange; // I < abs(IMAX) // I starts to increase when P < IRangef
    float output;
    float jumpTime;
    MyTimer myTimer;

public:
    PID();
    void setFirstTime();
    void setCoefficient(float, float, float);
    virtual void setTarget(float);
    void setIMax(float);
    void setIRange(float);
    void setErrorTolerance(float);
    void setDTolerance(float);
    void setJumpTime(float);
    virtual void update(float input);
    bool targetArrived();
    void setArrived(bool _arrived);
    float getOutput();
};

class PosPID : public PID{
private:
    Point targetP;
public:
    void setTarget(Point p);
    void update(Point input);
};

void AutoPIDAdjustment(PID pid, float adjIntervalLen, float (*_GetMeasurement)(void), void (*_ControlFunction)(float), int _SampleTime);

#endif