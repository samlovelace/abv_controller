#ifndef THRUSTERCOMMANDER_H
#define THRUSTERCOMMANDER_H

#include <eigen3/Eigen/Dense>
#include "abv_controller/Configurations.h"
#include "abv_controller/ConfigurationManager.h"
#include "abv_controller/UdpClient.h"

#include <mutex> 
#include <memory>

class ThrusterCommander
{
public:
    ThrusterCommander(/* args */);
    ~ThrusterCommander();

    void commandThrusters(Eigen::Vector3d aControlInput); 

private:
    Eigen::Vector3i convertToThrustVector(Eigen::Vector3d aControlInput); 
    void determineThrusterCommand(Eigen::Vector3i aThrustDirVec);

    ThrusterConfig mConfig; 
    std::string mThrusterCommand; 
    std::mutex mThrusterCommandMutex;
    Eigen::Matrix<int, 3, 27> mMatrixOfThrustDirCombinations; 

    std::unique_ptr<UdpClient> mUdpClient; 

};

#endif //THRUSTERCOMMANDER_H