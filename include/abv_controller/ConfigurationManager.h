#ifndef CONFIGURATIONMANAGER_H
#define CONFIGURATIONMANAGER_H

#include <tinyxml2.h>
#include "Configurations.h"

using namespace tinyxml2;

class ConfigurationManager
{
public:
    static ConfigurationManager* getInstance()
    {
        static ConfigurationManager instance;
        return &instance;
    }

    bool loadConfiguration(const char* aConfigFile);
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

    