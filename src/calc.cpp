#include "calc.h"
#include "math.h"

int sign(double x)
{
    return (x > 0) - (x < 0);
}

int sign(float x)
{
    return (x > 0) - (x < 0);
}

double rad2deg(double rad)
{
    return rad / M_PI * 180.0;
}

double deg2rad(double deg)
{
    return deg * M_PI / 180.0;
}
