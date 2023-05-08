#ifndef MYTIMER_H_
#define MYTIMER_H_

class MyTimer
{
private:
    double startTime;

public:
    MyTimer();
    MyTimer(double);
    void reset();
    int getTime() const;
    double getTimeDouble() const;
};

#endif