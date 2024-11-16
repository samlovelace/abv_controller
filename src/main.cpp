
#include <cstdio>
#include "abv_controller/CommandHandler.h"
#include "abv_controller/StateMachine.h"

void signalHandler(int signal)
{
    exit(1); 
}

int main()
{
    std::signal(SIGINT, signalHandler); 
    rclcpp::init(0, nullptr);

    std::shared_ptr<StateMachine> msm = std::make_shared<StateMachine>();
    std::unique_ptr<CommandHandler> cmdHandler = std::make_unique<CommandHandler>(msm); 

    msm->run(); 

    rclcpp::shutdown(); 
}