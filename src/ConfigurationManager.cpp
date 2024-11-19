
#include "abv_controller/ConfigurationManager.h"
#include "plog/Log.h"

ConfigurationManager::ConfigurationManager()
{
    // do nothing 
}

ConfigurationManager::~ConfigurationManager()
{
    // do nothing
}

bool ConfigurationManager::loadConfiguration(const char* aConfigFile)
{ 
    tinyxml2::XMLDocument configDoc;

    if (configDoc.LoadFile(aConfigFile) != tinyxml2::XML_SUCCESS) {
        return false;
    }

    // if the root element is not Configuration, something is wrong with the config file 
    tinyxml2::XMLElement* root = configDoc.FirstChildElement("Configuration");
    if (!root) {
        return false;
    }

    // parse the state machine specific configuration settings
    tinyxml2::XMLElement* stateMachineElement = root->FirstChildElement("StateMachine");
    if (stateMachineElement) {
        mConfigurations.stateMachineConfig.mControlMode = stateMachineElement->FirstChildElement("ControlMode")->GetText();
        mConfigurations.stateMachineConfig.mFrequency = stateMachineElement->FirstChildElement("Frequency")->IntText();
    }


    // parse the vehicle specific configuration settings
    tinyxml2::XMLElement* vehicleElement = root->FirstChildElement("Vehicle");
    if (vehicleElement) {
        mConfigurations.vehicleConfig.Name = vehicleElement->Attribute("Name");
        mConfigurations.vehicleConfig.Mass = vehicleElement->FirstChildElement("Mass")->DoubleText();
        mConfigurations.vehicleConfig.Inertia = vehicleElement->FirstChildElement("Inertia")->DoubleText();
        mConfigurations.vehicleConfig.Force1 = vehicleElement->FirstChildElement("Force1")->DoubleText();
        mConfigurations.vehicleConfig.Force2 = vehicleElement->FirstChildElement("Force2")->DoubleText();

        tinyxml2::XMLElement* stateTrackerElement = vehicleElement->FirstChildElement("StateTracker");
        if (stateTrackerElement) {
            mConfigurations.vehicleConfig.stateTrackerConfig.mInterface = stateTrackerElement->FirstChildElement("Interface")->GetText();
            mConfigurations.vehicleConfig.stateTrackerConfig.mRate = stateTrackerElement->FirstChildElement("Rate")->IntText();

            tinyxml2::XMLElement* networkElement = stateTrackerElement->FirstChildElement("Network");
            if (networkElement) {
                mConfigurations.vehicleConfig.stateTrackerConfig.mNetwork.Server.IP = networkElement->FirstChildElement("Server")->Attribute("ip");
                mConfigurations.vehicleConfig.stateTrackerConfig.mNetwork.Server.CmdPort = networkElement->FirstChildElement("Server")->IntAttribute("cmdPort");
                mConfigurations.vehicleConfig.stateTrackerConfig.mNetwork.Server.DataPort = networkElement->FirstChildElement("Server")->IntAttribute("dataPort");

                mConfigurations.vehicleConfig.stateTrackerConfig.mNetwork.Local.IP = networkElement->FirstChildElement("Local")->Attribute("ip");
                mConfigurations.vehicleConfig.stateTrackerConfig.mNetwork.Local.CmdPort = networkElement->FirstChildElement("Local")->IntAttribute("cmdPort");
                mConfigurations.vehicleConfig.stateTrackerConfig.mNetwork.Local.DataPort = networkElement->FirstChildElement("Local")->IntAttribute("dataPort");

                mConfigurations.vehicleConfig.stateTrackerConfig.mNetwork.Multicast.IP = networkElement->FirstChildElement("Multicast")->Attribute("ip");
            }
        }

        tinyxml2::XMLElement* controllerElement = vehicleElement->FirstChildElement("Controller");
        if(controllerElement)
        {
            mConfigurations.vehicleConfig.controllerConfig.Kp = ConfigUtils::parseVector3d(controllerElement->FirstChildElement("Kp")->GetText());
            mConfigurations.vehicleConfig.controllerConfig.Ki = ConfigUtils::parseVector3d(controllerElement->FirstChildElement("Ki")->GetText());
            mConfigurations.vehicleConfig.controllerConfig.Kd = ConfigUtils::parseVector3d(controllerElement->FirstChildElement("Kd")->GetText());

            mConfigurations.vehicleConfig.controllerConfig.thrusterConfig.arduino.IP = controllerElement->FirstChildElement("Thrusters")->FirstChildElement("Network")->FirstChildElement("Arduino")->Attribute("ip");
            mConfigurations.vehicleConfig.controllerConfig.thrusterConfig.arduino.CmdPort = controllerElement->FirstChildElement("Thrusters")->FirstChildElement("Network")->FirstChildElement("Arduino")->IntAttribute("cmdPort");
            mConfigurations.vehicleConfig.controllerConfig.thrusterConfig.arduino.DataPort = controllerElement->FirstChildElement("Thrusters")->FirstChildElement("Network")->FirstChildElement("Arduino")->IntAttribute("dataPort");

            mConfigurations.vehicleConfig.controllerConfig.thrusterConfig.uOn = controllerElement->FirstChildElement("Thrusters")->FirstChildElement("ControlInputDiscretizationThresholds")->FirstChildElement("On")->DoubleText();
            mConfigurations.vehicleConfig.controllerConfig.thrusterConfig.uOff = controllerElement->FirstChildElement("Thrusters")->FirstChildElement("ControlInputDiscretizationThresholds")->FirstChildElement("Off")->DoubleText();
        }

        tinyxml2::XMLElement* statePublisherElement = vehicleElement->FirstChildElement("StatePublisher"); 
        {
            if(statePublisherElement)
            {
                mConfigurations.vehicleConfig.statePublisherConfig.mInterface = statePublisherElement->FirstChildElement("Interface")->GetText();
                mConfigurations.vehicleConfig.statePublisherConfig.mRate = statePublisherElement->FirstChildElement("Frequency")->IntText();
            }
        }

    }

    return true;
}

void ConfigurationManager::logConfiguration()
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

    LOGD << "StateMachine Configuration:";
    LOGD << "   ControlMode: " << mConfigurations.stateMachineConfig.mControlMode;
    LOGD << "   Frequency: " << mConfigurations.stateMachineConfig.mFrequency;

    LOGD << "Vehicle Configuration:";
    LOGD << "   Name: " << mConfigurations.vehicleConfig.Name;
    LOGD << "   Mass (kg): " << mConfigurations.vehicleConfig.Mass;
    LOGD << "   Inertia (kg-m^2): " << mConfigurations.vehicleConfig.Inertia;
    LOGD << "   Force1 (N): " << mConfigurations.vehicleConfig.Force1;
    LOGD << "   Force2 (N): " << mConfigurations.vehicleConfig.Force2;

    LOGD << "   StateTracker Configuration:";
    LOGD << "       Interface: " << mConfigurations.vehicleConfig.stateTrackerConfig.mInterface;
    LOGD << "       Rate (Hz): " << mConfigurations.vehicleConfig.stateTrackerConfig.mRate;

    LOGD << "   StatePublisher Configuration: ";
    LOGD << "       Interface: " << mConfigurations.vehicleConfig.statePublisherConfig.mInterface; 
    LOGD << "       Rate (Hz): " << mConfigurations.vehicleConfig.statePublisherConfig.mRate; 

    LOGD << "    Network Configuration:";
    LOGD << "       Server IP: " << mConfigurations.vehicleConfig.stateTrackerConfig.mNetwork.Server.IP;
    LOGD << "       Server CmdPort: " << mConfigurations.vehicleConfig.stateTrackerConfig.mNetwork.Server.CmdPort;
    LOGD << "       Server DataPort: " << mConfigurations.vehicleConfig.stateTrackerConfig.mNetwork.Server.DataPort;

    LOGD << "       Local IP: " << mConfigurations.vehicleConfig.stateTrackerConfig.mNetwork.Local.IP;
    LOGD << "       Local CmdPort: " << mConfigurations.vehicleConfig.stateTrackerConfig.mNetwork.Local.CmdPort;
    LOGD << "       Local DataPort: " << mConfigurations.vehicleConfig.stateTrackerConfig.mNetwork.Local.DataPort;

    LOGD << "       Multicast IP: " << mConfigurations.vehicleConfig.stateTrackerConfig.mNetwork.Multicast.IP;

    LOGD << "   Controller Configuration:";
    LOGD << "       Kp: " << mConfigurations.vehicleConfig.controllerConfig.Kp;
    LOGD << "       Ki: " << mConfigurations.vehicleConfig.controllerConfig.Ki;
    LOGD << "       Kd: " << mConfigurations.vehicleConfig.controllerConfig.Kd;
    LOGD << "    Thruster Configuration:";
    LOGD << "       Arduino IP: " << mConfigurations.vehicleConfig.controllerConfig.thrusterConfig.arduino.IP;
    LOGD << "       Arduino CmdPort: " << mConfigurations.vehicleConfig.controllerConfig.thrusterConfig.arduino.CmdPort;
    LOGD << "       Arduino DataPort: " << mConfigurations.vehicleConfig.controllerConfig.thrusterConfig.arduino.DataPort;
    LOGD << "       uOn: " << mConfigurations.vehicleConfig.controllerConfig.thrusterConfig.uOn;
    LOGD << "       uOff: " << mConfigurations.vehicleConfig.controllerConfig.thrusterConfig.uOff;
}
