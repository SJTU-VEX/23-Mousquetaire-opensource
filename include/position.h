#ifndef POSITION_H_
#define POSITION_H_
#include "parameters.h"
#include "geometry.h"
#include "robot-config.h"
#include "my-timer.h"
using namespace vex;
class Position{
private:
    double curIMUHeading;
    double lastIMUHeading;
    double angleSpeed;
    double curLMileage, curRMileage;
    double lastLMileage, lastRMileage;
    double curLSpeed, curRSpeed;
    double filtedLSpeed, filtedRSpeed;
    double lastLSpeed, lastRSpeed;
    double LSpeed[3], RSpeed[3];
    double FiltLSpeed[3], FiltRSpeed[3];
    double selfYSpeed, selfXSpeed;
    double globalYSpeed, globalXSpeed;
    double lastglobalYSpeed, lastglobalXSpeed;
    double globalY, globalX;
    double lastTime;
    double sampleTime;
    MyTimer Timer;

    double fliter_b[3];
    double fliter_a[3];

    Position();
    ~Position();
    void updateInertialHeading();
    void updateAngleSpeed();
    void updateLMileage();
    void updateRMileage();
    void updateLSpeed();
    void updateRSpeed();
    void updateSelfYSpeed();
    void updateSelfXSpeed();
    void updateGlobalYSpeed();
    void updateGlobalXSpeed();
    void updateGlobalY();
    void updateGlobalX();
public:
    static Position *getInstance(){
        static Position *p = NULL;
        if (p == NULL){
            p = new Position();
        }
        return p;
    }
    static void deleteInstance(){
        Position *p = Position::getInstance();
        if(p != NULL){
            delete p;
            p = NULL;
        }
    }
    void updatePos();
    Point getPos() const;
    double getXSpeed() const;
    double getYSpeed() const;
    double getLMileage() const;
    double getRMileage() const;
    void resetYPosition();
    void resetXPosition();
    void setGlobalPosition(double _x, double _y);
};

void updatePosition();

#endif
