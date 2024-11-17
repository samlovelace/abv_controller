
#include "abv_controller/ThrusterCommander.h"

ThrusterCommander::ThrusterCommander(/* args */)
{
}

ThrusterCommander::~ThrusterCommander()
{
}

void ThrusterCommander::commandThrusters(Eigen::Vector3d aControlInput)
{
    // 1.  convert control input to thruster dir vector

    // 2.  convert thrust dir vector into thruster combination 

    // 3.  send to thrusters via UDP 
}