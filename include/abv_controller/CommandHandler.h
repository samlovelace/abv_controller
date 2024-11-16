#ifndef COMMANDHANDLER_H
#define COMMANDHANDLER_H

#include "abv_controller/RosTopicManager.h"
#include "abv_idl/msg/abv_command.hpp"
#include "abv_controller/StateMachine.h"

class CommandHandler
{

public:
    CommandHandler(std::shared_ptr<StateMachine> msm);
    ~CommandHandler();

    // TODO: 
    // this should probably be in the vehicle or state machine stuff
    enum class CommandType
    {
        THRUSTER, 
        POSE, 
        IDLE, 
        NUM_TYPES
    };

    void commandCallback(abv_idl::msg::AbvCommand::SharedPtr aCmdMsg); 
    CommandType toEnum(const std::string& aType); 
    std::string toString(CommandType aCmdType); 
    void setNewActiveState(StateMachine::States aNewState); 

private: 
    std::shared_ptr<StateMachine> mStateMachine;

};
#endif // COMMANDHANDLER_H
