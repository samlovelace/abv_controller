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



private: 
    ConfigurationManager();
    ~ConfigurationManager();

    Configurations mConfigurations;
    
};

#endif

    