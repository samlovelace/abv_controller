
#include "abv_controller/Vehicle.h"


Vehicle::Vehicle(/* args */) : mThrusterCommander(std::make_unique<ThrusterCommander>())
{
}

Vehicle::~Vehicle()
{
}

void Vehicle::doThrusterControl()
{
    printf("doing Thruster control\n");
    Eigen::Vector3d controlInput = getControlInput(); 
    mThrusterCommander->commandThrusters(controlInput);
}

void Vehicle::doPoseControl()
{
    // 1. get the current pose of the vehicle and subtract from goal pose

    // 2. give error to mController to calc control input vector

    // 3. setControlInput()

    // 4. doThrusterControl()
}