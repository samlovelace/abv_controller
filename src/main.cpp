
#include <cstdio>
#include "abv_controller/CommandHandler.h"
#include "abv_controller/StateMachine.h"
#include "abv_controller/Vehicle.h"

void signalHandler(int signal)
{
    exit(1); 
}

int main()
{
    std::signal(SIGINT, signalHandler); 
    rclcpp::init(0, nullptr);

    std::shared_ptr<Vehicle> abv = std::make_shared<Vehicle>(); 
    std::shared_ptr<StateMachine> msm = std::make_shared<StateMachine>(abv);
    std::unique_ptr<CommandHandler> cmdHandler = std::make_unique<CommandHandler>(msm, abv); 

    msm->run(); 

    rclcpp::shutdown(); 
}