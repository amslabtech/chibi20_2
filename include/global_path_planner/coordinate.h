#ifndef COORDINATE_H_
#define COORDINATE_H_

struct Coordinate
{
    Coordinate() :
        x(0), y(0) {}

    Coordinate(int _x,int _y) :
        x(_x), y(_y) {}

    void set_parameter(int _x,int _y)
    {
        x = _x;
        y = _y;
    }

    int x;
    int y;
};

#endif  // COORDINATE_H_
