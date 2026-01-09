#ifndef ABVSTATE_HPP
#define ABVSTATE_HPP

#include <vector> 

struct AbvState
{
    double x, y, theta; 
    double vx, vy, omega; 
};

inline std::vector<double> toVector(const AbvState& x) 
{ 
    return { x.x, x.y, x.theta, x.vx, x.vy, x.omega }; 
}

#endif 
