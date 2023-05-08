#ifndef GEOMETRY_H_
#define GEOMETRY_H_
#include "calc.h"
#include <cmath>
class Point;
class Vector
{
public:
    double _x;
    double _y;
    Vector() : _x(0), _y(0) {}
    Vector(double x, double y) : _x(x), _y(y) {}
    Vector(const Point &, const Point &); //
    Vector(const Vector &v) : _x(v._x), _y(v._y) {}
    Vector &operator=(Vector const &); //
    Vector operator+(Vector const &v) const { return Vector(_x + v._x, _y + v._y); }
    Vector operator-(Vector const &v) const { return Vector(_x - v._x, _y - v._y); }
    double operator*(Vector const &v) const { return _x * v._x + _y * v._y; } // dot product
    Vector operator*(double a) const { return Vector(_x * a, _y * a); }
    Vector operator/(double a) const { return Vector(_x / a, _y / a); }
    friend Vector operator*(double a, Vector v) { return Vector(v._x * a, v._y * a); }

    double mod() const { return sqrt(_x * _x + _y * _y); }
    /**
     * @brief Returns direction in degree.
     * 
     * @return double
     */
    double dir() const { return rad2deg(atan2(_y, _x)); }
    Vector rotateTrans(double) const; //

    void setV(double x, double y)
    {
        _x = x;
        _y = y;
    }
    void resetV()
    {
        _x = 0;
        _y = 0;
    }

    // void setV(const Point&, const Point&);
};

class Point
{
public:
    double _x;
    double _y;
    Point() : _x(0), _y(0) {}
    Point(double x, double y) : _x(x), _y(y) {}
    Point(const Point &p) : _x(p._x), _y(p._y) {}

    Point &operator=(Point const &p)
    {
        if (this == &p)
            return *this;
        else
        {
            this->_x = p._x;
            this->_y = p._y;
            return *this;
        }
    }
    Point operator+(const Vector &v) const { return Point(_x + v._x, _y + v._y); }
    Point operator-(const Vector &v) const { return Point(_x - v._x, _y - v._y); }
    Vector operator-(const Point &p) const { return Vector(_x - p._x, _y - p._y); }
    friend Point operator+(const Vector &v, const Point &p) { return Point(p._x + v._x, p._y + v._y); }

    void setP(double x, double y)
    {
        _x = x;
        _y = y;
    }
    void resetP()
    {
        _x = 0;
        _y = 0;
    }
};

class Arc
{
};

#endif
