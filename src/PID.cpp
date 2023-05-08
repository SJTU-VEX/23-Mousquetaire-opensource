#include "PID.h"
#include "basic-functions.h"
#include "my-timer.h"
#include "calc.h"
#include <math.h>
#include "vex.h"
#include "geometry.h"
#include <iostream>
using namespace std;

PID::PID() : firstTime(true), arrived(false), IMax(20), IRange(50), jumpTime(50)
{
    myTimer.reset();
}

void PID::setFirstTime()
{
    firstTime = true;
}

void PID::setCoefficient(float _kp, float _ki, float _kd)
{
    kp = _kp;
    ki = _ki;
    kd = _kd;
}
void PID::setTarget(float _target) { target = _target; }
void PID::setIMax(float _IMax) { IMax = _IMax; }
void PID::setIRange(float _IRange) { IRange = _IRange; }
void PID::setErrorTolerance(float _errorTol) { errorTol = _errorTol; }
void PID::setDTolerance(float _DTol) { DTol = _DTol; }
void PID::setJumpTime(float _jumpTime) { jumpTime = _jumpTime; }
void PID::setArrived(bool _arrived) {arrived = _arrived;}
bool PID::targetArrived() { return arrived; }
float PID::getOutput() { return output; }

void PID::update(float input)
{
    errorCurt = target - input; // calculate current error
    P = kp * errorCurt;
    if (firstTime)
    { // first time to update
        firstTime = false;
        errorPrev = errorCurt;
        errorInt = 0;
    }
    errorDev = errorCurt - errorPrev; // calculate the derivative of error
    errorPrev = errorCurt;            // record error
    D = kd * errorDev;                // calculate D
    if (fabs(P) >= IRange)
    { // I = 0 for P > IRange
        errorInt = 0;
    }
    else
    { // P <= IRange -> Integrate
        errorInt += errorCurt;
        if (fabs(errorInt) * ki > IMax) // Limit I to IMax
            errorInt = sign(errorInt) * IMax / ki;
    }
    if (sign(errorInt) != sign(errorCurt) || (fabs(errorCurt) <= errorTol)) // Clear I for small enough error
        errorInt = 0;
    I = ki * errorInt; // Calculate I
    if (fabs(errorCurt) <= errorTol && fabs(D) <= DTol)
    { // Exit when staying in tolerated region and maintaining a low enough speed for enough time
        if (myTimer.getTime() >= jumpTime)
            arrived = true;
    }
    else
    {
        myTimer.reset();
        arrived = false;
    }
    output = P + I + D;
}

void PosPID::setTarget(Point p){
    targetP = p;
}

void PosPID::update(Point input){
    Vector err = targetP - input;
    errorCurt = err.mod(); // calculate current error
    P = kp * errorCurt;
    if (firstTime)
    { // first time to update
        firstTime = false;
        errorPrev = errorCurt;
        errorInt = 0;
    }
    errorDev = errorCurt - errorPrev; // calculate the derivative of error
    errorPrev = errorCurt;            // record error
    D = kd * errorDev;                // calculate D
    if (fabs(P) >= IRange)
    { // I = 0 for P > IRange
        errorInt = 0;
    }
    else
    { // P <= IRange -> Integrate
        errorInt += errorCurt;
        if (fabs(errorInt) * ki > IMax) // Limit I to IMax
            errorInt = sign(errorInt) * IMax / ki;
    }
    if (sign(errorInt) != sign(errorCurt) || (fabs(errorCurt) <= errorTol)) // Clear I for small enough error
        errorInt = 0;
    I = ki * errorInt; // Calculate I
    if (fabs(errorCurt) <= errorTol && fabs(D) <= DTol)
    { // Exit when staying in tolerated region and maintaining a low enough speed for enough time
        if (myTimer.getTime() >= jumpTime)
            arrived = true;
    }
    else
    {
        myTimer.reset();
        arrived = false;
    }
    output = P + I + D;
}

void AutoPIDAdjustment(PID pid, float adjIntervalLen, float (*_GetMeasurement)(void), void (*_ControlFunction)(float), int _SampleTime)
{
    //初始化整定器
    double kp = 0.6, ki = 0, kd = 0;
    const double iniKp = kp;
    MyTimer Timer; //控制等幅震荡持续时间
    float startVal, endVal;
    float tarVal; 
    bool findP = false;
    bool reverse = false;
    double Tu = 0;  //阶跃响应等幅振荡周期
    int passZeroCnt = 0;
    int errSign;
    double timestamp = 0;
    startVal = _GetMeasurement(); //默认整定开始时机器位置为被控变量整定区间起�?
    endVal = startVal + adjIntervalLen;
    //查找临界比例度（P�?
    while (!findP)
    {
        //pid初始�?
        if(reverse) tarVal = startVal;
        else tarVal = endVal;
        pid.setTarget(tarVal);
        pid.setCoefficient(kp, ki, kd);
        pid.setArrived(false);
        cout << "-------------------------------" << endl;
        cout << "kp: " << kp << endl;
        Timer.reset();
        passZeroCnt = 0;
        //初始化误差符�?
        errSign = reverse ? -sign(adjIntervalLen) : sign(adjIntervalLen);
        timestamp = 0;
        //阻塞运行获取阶跃响应曲线
        while (!pid.targetArrived() && Timer.getTime() <= 8000){
            double err = tarVal - _GetMeasurement();
            pid.update(_GetMeasurement());
            _ControlFunction(pid.getOutput());

            if (sign(err) != errSign){
                errSign = sign(err);
                ++passZeroCnt;
                //计算震荡周期
                if (passZeroCnt == 1){
                    timestamp = Timer.getTime();
                }
                if (passZeroCnt == 3){
                    Tu = Timer.getTime() - timestamp;
                }
            }
            if (abs(err) > abs(adjIntervalLen * 1.5)){
                kp -= iniKp / 6 * 5;
                cout << "kp too large, adjust to: " << kp <<endl; 
                break;
            }
            this_thread::sleep_for(_SampleTime);
        }
        //分析曲线统计特征
        if (Timer.getTime() >= 8000 && passZeroCnt >= 6){
            findP = true;
        }
        if (passZeroCnt <= 0){ //无过�?
            kp += 0.5 * iniKp;
        }
        if (passZeroCnt == 1 || passZeroCnt == 2){ 
            kp += iniKp / 3;
        }
        if (passZeroCnt > 2){
            kp += iniKp / 6;
        }
        reverse = !reverse;
        cout << "pass zero: " << passZeroCnt << endl;
    }
    double ku = kp;
    Tu /= 1000;
    kp = 0.2 * ku;
    ki = 2 * kp / Tu;
    kd = kp * Tu / 3;
    cout << "Z-N: ";
    cout << "kp: " << kp << " ki: " << ki << " kd: " << kd << endl;
    // cout << "C-C: ";
    // kp = Tu / ku 
    // cout << "kp: " << kp << " ki: " << ki << " kd: " << kd << endl;
}