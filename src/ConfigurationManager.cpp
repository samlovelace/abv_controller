
#include "abv_controller/ConfigurationManager.h"
#include "plog/Log.h"
#include <yaml-cpp/yaml.h>

ConfigurationManager::ConfigurationManager()
{
    // do nothing 
}

ConfigurationManager::~ConfigurationManager()
{
    // do nothing
}

bool ConfigurationManager::loadConfiguration(const std::string& aFilePath)
{   
    LOGD << R"(
        __    __   _______  __       __        ______               ___      .______   ____    ____ 
       |  |  |  | |   ____||  |     |  |      /  __  \             /   \     |   _  \  \   \  /   / 
       |  |__|  | |  |__   |  |     |  |     |  |  |  |           /  ^  \    |  |_)  |  \   \/   /  
       |   __   | |   __|  |  |     |  |     |  |  |  |          /  /_\  \   |   _  <    \      /   
       |  |  |  | |  |____ |  `----.|  `----.|  `--'  |  __     /  _____  \  |  |_)  |    \    /    
       |__|  |__| |_______||_______||_______| \______/  (_ )   /__/     \__\ |______/      \__/     
                                                         |/                                         
       )";
   
    YAML::Node config = YAML::LoadFile(aFilePath); 
    std::stringstream s; 
    s << "Configuration: ";
    s << YAML::Dump(config);
    LOGD << s.str();  

    // Parse StateMachine
    mConfigurations.stateMachineConfig.mFrequency = config["StateMachine"]["Rate"].as<int>(); 

    // Parse Vehicle
    YAML::Node vehicleNode = config["Vehicle"];
    mConfigurations.vehicleConfig.Name = vehicleNode["Name"].as<std::string>();
    mConfigurations.vehicleConfig.Mass = vehicleNode["Mass"].as<double>();
    mConfigurations.vehicleConfig.Inertia = vehicleNode["Inertia"].as<double>();
    mConfigurations.vehicleConfig.Force1 = vehicleNode["Force1"].as<double>();
    mConfigurations.vehicleConfig.Force2 = vehicleNode["Force2"].as<double>();

    // Parse Controller Gains
    YAML::Node controllerNode = config["Controller"];
    mConfigurations.vehicleConfig.controllerConfig.Kp = ConfigUtils::parseVector3d(controllerNode["Kp"]);
    mConfigurations.vehicleConfig.controllerConfig.Ki = ConfigUtils::parseVector3d(controllerNode["Ki"]);
    mConfigurations.vehicleConfig.controllerConfig.Kd = ConfigUtils::parseVector3d(controllerNode["Kd"]);

    // Parse Thrusters
    YAML::Node thrusterNode = config["Thrusters"]["InputDiscretization"];
    mConfigurations.vehicleConfig.controllerConfig.thrusterConfig.uOn = thrusterNode["On"].as<double>();
    mConfigurations.vehicleConfig.controllerConfig.thrusterConfig.uOff = thrusterNode["Off"].as<double>();

    // Parse Network Configuration
    auto parseSocket = [](const YAML::Node& node) -> SocketConfig {
        return {
            node["Ip"].as<std::string>(),
            node["cmdPort"].as<int>(),
            node["dataPort"].as<int>()
        };
    };

    YAML::Node networkNode = config["Network"];
    mConfigurations.vehicleConfig.stateTrackerConfig.mNetwork.Server = parseSocket(networkNode["Server"]);
    mConfigurations.vehicleConfig.stateTrackerConfig.mNetwork.Local = parseSocket(networkNode["Local"]);
    mConfigurations.vehicleConfig.stateTrackerConfig.mNetwork.Multicast = parseSocket(networkNode["Multicast"]);
    mConfigurations.vehicleConfig.stateTrackerConfig.mNetwork.Arduino = parseSocket(networkNode["Arduino"]);

    YAML::Node stateTrackerNode = config["StateTracker"]; 
    mConfigurations.vehicleConfig.stateTrackerConfig.mInterface = stateTrackerNode["Interface"].as<std::string>(); 
    mConfigurations.vehicleConfig.stateTrackerConfig.mRate = stateTrackerNode["Rate"].as<int>(); 

    return true; 
}
