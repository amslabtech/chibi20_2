#ifndef WAYPOINT_H_
#define WAYPOINT_H_

struct Waypoint
{
    Waypoint() :
        id(0), x(0), y(0) {}

    Waypoint(int _id,int _x,int _y) :
        id(_id), x(_x), y(_y) {}

    int id;
    int x;
    int y;
};

#endif  // WAYPOINT_H_
