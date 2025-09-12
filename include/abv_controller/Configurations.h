#ifndef CONFIGURATIONS_H
#define CONFIGURATIONS_H

#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>

struct StateMachineConfig {
    std::string mControlMode;
    int mFrequency;
};

struct SocketConfig {
    std::string IP;
    int CmdPort;
    int DataPort;
};

struct NetworkConfig {
    SocketConfig Server;
    SocketConfig Local;
    SocketConfig Multicast;
    SocketConfig Arduino;
};

struct StateTrackerConfig {
    std::string mInterface;
    int mRate;
    int mRigidBodyId; 
    NetworkConfig mNetwork;
};

struct StatePublisherConfig {
    std::string mInterface; 
    int mRate; 
};

struct ThrusterConfig {
    SocketConfig arduino;
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
    double Inertia;
    double Force1;
    double Force2;

    StateTrackerConfig stateTrackerConfig;
    StatePublisherConfig statePublisherConfig; 
    ControllerConfig controllerConfig;
};

struct Configurations {
    StateMachineConfig stateMachineConfig;
    VehicleConfig vehicleConfig;
};

namespace ConfigUtils
{
    static Eigen::Vector3d parseVector3d(const YAML::Node& node) 
    {
        Eigen::Vector3d vec;

        if (node && node.IsSequence() && node.size() == 3) {
            vec[0] = node[0].as<double>();
            vec[1] = node[1].as<double>();
            vec[2] = node[2].as<double>();
        } else {
            throw std::runtime_error("Invalid Vector3d format in YAML.");
        }

        return vec;
    }
}

#endif // CONFIGURATIONS_H