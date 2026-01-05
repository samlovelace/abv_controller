#ifndef NAVIGATIONMANAGER_H
#define NAVIGATIONMANAGER_H

#include "robot_idl/msg/abv_state.hpp"
#include <eigen3/Eigen/Dense>
#include <mutex> 

class NavigationManager
{
public:
    NavigationManager(/* args */);
    ~NavigationManager();

    bool doStateTracking() { return false; }
    Eigen::Matrix<float, 12, 1> getCurrentState() { std::scoped_lock lock(mCurrentStateMutex); return mCurrentState; }


private: 

    void stateCallback(const robot_idl::msg::AbvState::SharedPtr aMsg); 

    Eigen::Matrix<float, 12, 1> mCurrentState; 
    std::mutex mCurrentStateMutex; 
};
#endif 

