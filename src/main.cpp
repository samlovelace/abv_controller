
#include <cstdio>
#include "abv_controller/CommandHandler.h"
#include "abv_controller/StateMachine.h"
#include "abv_controller/Vehicle.h"
#include "abv_controller/Logger.h"
#include "abv_controller/ConfigurationManager.h"

// Signal handler function
void signalHandler(int signal) {

    LOGD  << "\n" << R"(
    _________________________
    |                       |
    |   SHUTTING DOWN...    |
    |_______________________|
            \   ^__^
             \  (oo)\_______
                (__)\       )\/\
                    ||----w |
                    ||     ||)";
    exit(0); // Exit the program
}

int main()
{
    std::signal(SIGINT, signalHandler); 
    createLogger(); 

    std::string configFilePath = "./src/abv_controller/configuration/config.xml";

    if(!ConfigurationManager::getInstance()->loadConfiguration(configFilePath.c_str()))
    {
        printf("Could not load %s\n", configFilePath.c_str()); 
        return 0; 
    }

    // if you got here, config file loaded successfully
    ConfigurationManager::getInstance()->logConfiguration(); 
    
    rclcpp::init(0, nullptr);
    std::shared_ptr<Vehicle> abv = std::make_shared<Vehicle>(); 
    std::shared_ptr<StateMachine> msm = std::make_shared<StateMachine>(abv);
    std::unique_ptr<CommandHandler> cmdHandler = std::make_unique<CommandHandler>(msm, abv); 

    msm->run(); 

    rclcpp::shutdown(); 
}