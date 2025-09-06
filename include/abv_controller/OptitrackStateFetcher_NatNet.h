#ifndef OPTITRACKSTATEFETCHER_H
#define OPTITRACKSTATEFETCHER_H

#include "IStateFetcher.h"

#ifdef ARCH_X86
#include "NatNetTypes.h"
#include "NatNetClient.h"
#include "NatNetCAPI.h"
#endif 

#include <memory>
#include "ConfigurationManager.h"
#include "Configurations.h" 

class OptitrackStateFetcher_NatNet : public IStateFetcher
{
public:
    OptitrackStateFetcher_NatNet(NetworkConfig aConfig);
    ~OptitrackStateFetcher_NatNet() override; 

    bool init() override; 
    Eigen::Matrix<double, 6, 1> fetchState() override; 

    void setLatestState(Eigen::Matrix<double, 6, 1> aLatestState) {std::lock_guard<std::mutex> lock(mStateMutex); mLatestState = aLatestState; }

private:
    
    #ifdef ARCH_X86
    std::unique_ptr<NatNetClient> mNatNetClient;
    // callback function when frame is recvd
    void frameRecvdCallback(sFrameOfMocapData* data, void* pUserData);
    #endif 

    NetworkConfig mConfig; 
    Eigen::Matrix<double, 6, 1> mLatestState; 
    int32_t mID; 

    std::mutex mStateMutex; 

    // helper/debug function to print connection info for optitrack
    void printConnectionInfo(); 
};
#endif //OPTITRACKSTATEFETCHER_H
    