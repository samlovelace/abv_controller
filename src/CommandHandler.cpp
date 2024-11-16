
#include "abv_controller/CommandHandler.h"


CommandHandler::CommandHandler(std::shared_ptr<StateMachine> msm, std::shared_ptr<Vehicle> abv) : 
    mStateMachine(msm), mVehicle(abv)
{
    auto topicManager = RosTopicManager::getInstance(); 
    topicManager->createSubscriber<abv_idl::msg::AbvCommand>("abv_command", 
                                    std::bind(&CommandHandler::commandCallback, this, std::placeholders::_1)); 

    topicManager->spinNode(); 

    while (!topicManager->isROSInitialized())
    {
    }
    
    printf("ROS Comms Initialized\n");
}

CommandHandler::~CommandHandler()
{
}

void CommandHandler::commandCallback(abv_idl::msg::AbvCommand::SharedPtr aCmdMsg)
{
    if(CommandType::THRUSTER == toEnum(aCmdMsg->type))
    {
        mVehicle->setControlInput(convertToEigen(aCmdMsg->data)); 
        setNewActiveState(StateMachine::States::THRUSTER_CONTROL); 
    }
    else if (CommandType::POSE == toEnum(aCmdMsg->type))
    {
        mVehicle->setGoalPose(convertToEigen(aCmdMsg->data)); 
        setNewActiveState(StateMachine::States::POSE_CONTROL);   
    }
    else if (CommandType::IDLE == toEnum(aCmdMsg->type))
    {
        setNewActiveState(StateMachine::States::IDLE); 
    }
}

Eigen::Vector3d CommandHandler::convertToEigen(abv_idl::msg::AbvVec3 aVectorToConvert)
{
    Eigen::Vector3d vectorToReturn;
    vectorToReturn[0] = aVectorToConvert.x; 
    vectorToReturn[1] = aVectorToConvert.y; 
    vectorToReturn[2] = aVectorToConvert.yaw; 

    return vectorToReturn; 
}

void CommandHandler::setNewActiveState(StateMachine::States aNewState)
{
    auto currentState = mStateMachine->getActiveState(); 

    if(aNewState != currentState)
    {
        printf("Transitioning state machine from %s to %s\n", mStateMachine->toString(currentState).c_str(), mStateMachine->toString(aNewState).c_str());
        mStateMachine->setActiveState(aNewState); 
    }
}

CommandHandler::CommandType CommandHandler::toEnum(const std::string& aType)
{
    CommandType enumToReturn; 

    if("Thruster" == aType || "thruster" == aType)
    {
        enumToReturn = CommandType::THRUSTER; 
    }
    else if ("Pose" == aType || "pose" == aType)
    {
        enumToReturn = CommandType::POSE; 
    }
    else if ("Idle" == aType || "idle" == aType)
    {
        enumToReturn = CommandType::IDLE; 
    }
    else
    {
        printf("Unsupported CommandType: %s", aType.c_str()); 
        enumToReturn = CommandType::NUM_TYPES; 
    }

    return enumToReturn; 
}

std::string CommandHandler::toString(CommandType aCmdType)
{
    std::string stringToReturn = ""; 
    switch (aCmdType)
    {
    case CommandType::IDLE:
        stringToReturn = "IDLE"; 
        break;
    case CommandType::THRUSTER: 
        stringToReturn = "THRUSTER_CONTROL";
        break; 
    case CommandType::POSE: 
        stringToReturn = "POSE_CONTROL"; 
        break;
    default:
        break;
    }

    return stringToReturn; 
}