#ifndef THRUSTERCOMMANDER_H
#define THRUSTERCOMMANDER_H

#include <eigen3/Eigen/Dense>

class ThrusterCommander
{
public:
    ThrusterCommander(/* args */);
    ~ThrusterCommander();

    void commandThrusters(Eigen::Vector3d aControlInput); 

private:
    /* data */

};

#endif //THRUSTERCOMMANDER_H