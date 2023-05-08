#include "geometry.h"
#include "calc.h"
#include <cmath>
Vector ::Vector(const Point &p1, const Point &p2) : _x(p2._x - p1._x), _y(p2._y - p1._y) {}

Vector &Vector ::operator=(Vector const &v)
{
    if (this == &v)
        return *this;
    else
    {
        this->_x = v._x;
        this->_y = v._y;
        return *this;
    }
}

Vector Vector ::rotateTrans(double theta) const
{
    double x = cos(deg2rad(dir() + theta)) * mod();
    double y = sin(deg2rad(dir() + theta)) * mod();
    return Vector(x, y);
}
// void Vector :: setV(const Point& p1, const Point& p2) {_x = p2._x - p1._x; _y = p2._y - p1._y;}