#ifndef CONFIGURATIONS_H
#define CONFIGURATIONS_H

#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>

struct StateMachineConfig {
    std::string ControlMode;
    int Frequency;
};

struct Socket {
    std::string IP;
    int CmdPort;
    int DataPort;
};

struct Network {
    Socket Server;
    Socket Local;
    Socket Multicast;
    Socket Arduino;
};

struct StateTrackerConfig {
    std::string Interface;
    int Rate;
    Network network;
};

struct ThrusterConfig {
    Socket arduino;
    double uOn;
    double uOff;
};

struct ControllerConfig {
    Eigen::Vector3d Kp;
    Eigen::Vector3d Ki;
    Eigen::Vector3d Kd;
    ThrusterConfig thrusterConfig;
};

struct VehicleConfig {
    std::string Name;
    double Mass;
    std::string MassUnits;
    double Inertia;
    std::string InertiaUnits;
    double Force1;
    std::string Force1Units;
    double Force2;
    std::string Force2Units;
    StateTrackerConfig stateTrackerConfig;
    ControllerConfig controllerConfig;
};

struct Configurations {
    StateMachineConfig stateMachineConfig;
    VehicleConfig vehicleConfig;
};

namespace ConfigUtils
{
    inline Eigen::Vector3d parseVector3d(const char* text) {
    Eigen::Vector3d vec;
    std::istringstream iss(text);
    iss >> vec[0] >> vec[1] >> vec[2];
    return vec;
}
}

#endif // CONFIGURATIONS_H