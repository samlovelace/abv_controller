#ifndef COMMANDHANDLER_H
#define COMMANDHANDLER_H

#include "abv_controller/RosTopicManager.h"
#include "robot_idl/msg/abv_command.hpp"
#include "abv_controller/StateMachine.h"
#include "abv_controller/Vehicle.h"

class CommandHandler
{

public:
    CommandHandler(std::shared_ptr<StateMachine> msm, std::shared_ptr<Vehicle> abv);
    ~CommandHandler();

    // TODO: 
    // this should probably be in the vehicle or state machine stuff
    enum class CommandType
    {
        THRUSTER, 
        POSE, 
        VELOCITY,
        IDLE, 
        NUM_TYPES
    };

    void commandCallback(robot_idl::msg::AbvCommand::SharedPtr aCmdMsg); 
    CommandType toEnum(const std::string& aType); 
    std::string toString(CommandType aCmdType); 
    void setNewActiveState(StateMachine::States aNewState); 
    Eigen::Vector3d convertToEigen(robot_idl::msg::AbvVec3 aVectorToConvert);

private: 
    std::shared_ptr<StateMachine> mStateMachine;
    std::shared_ptr<Vehicle> mVehicle;

};
#endif // COMMANDHANDLER_H
