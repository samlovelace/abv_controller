#ifndef OPTITRACKSTATEFETCHER_H
#define OPTITRACKSTATEFETCHER_H

#include "IStateFetcher.h"
#include "NatNetTypes.h"
#include "NatNetClient.h"
#include "NatNetCAPI.h"
#include <memory>

class OptitrackStateFetcher : public IStateFetcher
{
public:
    OptitrackStateFetcher(/* args */);
    ~OptitrackStateFetcher() override; 

    bool init() override; 
    Eigen::Matrix<double, 6, 1> fetchState() override; 

private:
    std::unique_ptr<NatNetClient> mNatNetClient;
    
    // callback function when frame is recvd
    void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData);
};
#endif //OPTITRACKSTATEFETCHER_H
    