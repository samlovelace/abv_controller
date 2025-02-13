#ifndef SIMULATEDSTATEFETCHER_H
#define SIMULATEDSTATEFETCHER_H 

#include "abv_controller/IStateFetcher.h"
#include "abv_idl/msg/abv_state.hpp"

class SimulatedStateFetcher : public IStateFetcher
{
public:
    SimulatedStateFetcher(/* args */);
    ~SimulatedStateFetcher() override; 

    bool init() override; 
    Eigen::Matrix<double, 6,1> fetchState() override; 

private:
    void stateCallback(abv_idl::msg::AbvState::SharedPtr aSimState); 

    std::mutex mStateMutex; 
    Eigen::Matrix<double, 6, 1> mState; 

    void setState(Eigen::Matrix<double, 6, 1> aState); 

};

#endif // SIMULATEDSTATEFETCHER_H

