#ifndef COST_H_
#define COST_H_

struct Cost
{
    Cost() :
        g(0.0), f(0.0), parent_x(0), parent_y(0) {}

    void set_parameter(float _g,float _f,int _parent_x,int _parent_y)
    {
        g = _g;
        f = _f;
        parent_x = _parent_x;
        parent_y = _parent_y;
    }

    float g;
    float f;
    int parent_x;
    int parent_y;
};

#endif  // COST_H_
