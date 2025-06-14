#ifndef CONFIGURATIONMANAGER_H
#define CONFIGURATIONMANAGER_H

#include "Configurations.h"


class ConfigurationManager
{
public:
    static ConfigurationManager* getInstance()
    {
        static ConfigurationManager instance;
        return &instance;
    }

    bool loadConfiguration(const std::string& aFilePath);
    void logConfiguration();

    ThrusterConfig getThrusterConfig() { return mConfigurations.vehicleConfig.controllerConfig.thrusterConfig; }
    StateTrackerConfig getStateTrackerConfig() {return mConfigurations.vehicleConfig.stateTrackerConfig;}
    StatePublisherConfig getStatePublisherConfig() {return mConfigurations.vehicleConfig.statePublisherConfig;}
    ControllerConfig getControllerConfig() {return mConfigurations.vehicleConfig.controllerConfig;}

private: 
    ConfigurationManager();
    ~ConfigurationManager();

    Configurations mConfigurations;
    
};

#endif

    