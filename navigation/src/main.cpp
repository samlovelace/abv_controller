
#include <cstdio>
#include "common/RosTopicManager.h"
#include "common/DataLogger.h"
#include "common/SignalHandler.hpp"
#include "common/ConfigurationManager.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

int main()
{
    std::signal(SIGINT, signalHandler); 

    // instantiate singletons 
    DataLogger::get().createMainLog("navigation");

    std::string configFilePath = ament_index_cpp::get_package_share_directory("abv_controller") + "/configuration/config.yaml"; 
    if(!ConfigurationManager::getInstance()->loadConfiguration(configFilePath))
    {
        printf("Could not load config file at %s\n", configFilePath.c_str()); 
        return 0; 
    }
    
    rclcpp::init(0, nullptr);
    RosTopicManager::getInstance("abv_navigation"); 

    while(true)
    {
        LOGD << "Nav";
        sleep(1); 
    }

    rclcpp::shutdown(); 
}