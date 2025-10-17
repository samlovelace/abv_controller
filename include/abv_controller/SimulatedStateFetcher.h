#ifndef SIMULATEDSTATEFETCHER_H
#define SIMULATEDSTATEFETCHER_H 

#include "abv_controller/IStateFetcher.h"
#include "robot_idl/msg/abv_state.hpp"

class SimulatedStateFetcher : public IStateFetcher
{
public:
    SimulatedStateFetcher(/* args */);
    ~SimulatedStateFetcher() override; 

    bool init() override; 
    Eigen::Matrix<float, 12,1> fetchState() override; 

private:

    void stateCallback(robot_idl::msg::AbvState::SharedPtr aSimState); 
    void setState(Eigen::Matrix<float, 12, 1> aState); 

private: 
    
    std::mutex mStateMutex;


};

#endif // SIMULATEDSTATEFETCHER_H

