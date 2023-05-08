#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_
#include "vex.h"
#include "my-timer.h"
#include "PID.h"
#include "geometry.h"
#include "position.h"
#include "chassis.h"
using namespace vex;






/**
 * @brief 路径规划自动机类
 *
 */
class Trajectory
{
public:
    // Trajectory(Point, double, double, double, double);
    Trajectory(){};
    // Trajectory(Point _tarPos, double _tarAng, double _maxSpeed);
    // Trajectory(double _tarAng, double _maxSpeed);
    // Trajectory(Point _tarPos, double _tarAng, double _maxSpeed, double _accPctv, double _decPctv);
    
    void setJumpTime(double t);
    void setTranTol(double tol);
    void setAngTol(double tol);
    void setPIDV(float _p, float _i, float _d, float _IMax, float _IRange);
    void setPIDR(float _p, float _i, float _d, float _IMax, float _IRange);
    bool targetArrived();
    Vector getVelocity();
    double getVelocityR();
    void update();

protected:
    /**
     * @brief 路径规划自动机的状态集合
     *
     */
    enum STAGE
    {
        BEGIN,      ///< 开始
        SHORTMOVE,  ///< 小距离pid
        ACCEL,      ///< 加速
        CONSTSPEED, ///< 匀速
        DECEL,      ///< 减速
        STOP,       ///< 结束
        DONOTHING   ///< 空置
    };

    STAGE stageV, stageR; ///< 当前状态

    Point initPos;    // 初始位置
    double initAng;   // 初始角度
    Point targetPos;  // 目标位置
    double targetAng; // 目标角度

    PosPID pidv, pidvs;
    PID pidr, pidrs;
    PID pidt;
    Vector velocity;
    double velocityR;

    double accPctv, decPctv;
    double accPctr, decPctr;
    double maxSpeed;
    MyTimer timer1, timer2;
    bool varrived, rarrived;

    double jumpTime;
    double tranTol, angTol;
    
    void initPID();
    void softStartV();
    void softStartR();
    void constSpeedV();
    void constSpeedR();
    void pidControlV(PosPID &pid);
    void pidControlR(PID &pid);
    virtual void updateVelocity() = 0;
    virtual void updateStage() = 0;
    void updateArriveState();
};

class NormalTrajectory : public Trajectory{
public:
    NormalTrajectory(Point _tarPos, double _tarAng, double _maxSpeed);
    NormalTrajectory(double _tarAng, double _maxSpeed);
    NormalTrajectory(Point _tarPos, double _tarAng, double _maxSpeed, double _accPctv, double _decPctv);
private:
    void updateVelocity();
    void updateStage();
};

class PIDTrajectory : public Trajectory{
public:
    PIDTrajectory(Point _tarPos, double _tarAng, double _maxSpeed);
    PIDTrajectory(double _tarAng, double _maxSpeed);
    PIDTrajectory(Point _tarPos, double _tarAng, double _maxSpeed, double _accPctv, double _decPctv);
private:
    void updateVelocity();
    void updateStage();
};

class TrajectoryFactory{
public:
    static Trajectory* getPerciseTraj(Point _tarPos, double _tarAng, double _maxSpeed);
    static Trajectory* getPerciseTurnTraj(double _tarAng, double _maxSpeed);
    static Trajectory* getQuickTurnTraj(double _tarAng, double _maxSpeed);
    static Trajectory* getQuickTraj(Point _tarPos, double _tarAng, double _maxSpeed);
    static Trajectory* getQuickTrajWithoutStop(Point _tarPos, double _tarAng, double _maxSpeed);
};

#endif
