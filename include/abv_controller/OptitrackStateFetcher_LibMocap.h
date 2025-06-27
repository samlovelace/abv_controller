#ifndef OPTITRACKSTATEFETCHER_LIBMOCAP_H
#define OPTITRACKSTATEFETCHER_LIBMOCAP_H
 
#include "IStateFetcher.h"
#include <memory>
#include "ConfigurationManager.h"
#include "Configurations.h" 



class OptitrackStateFetcher_LibMocap : public IStateFetcher
{ 
public:
    OptitrackStateFetcher_LibMocap(NetworkConfig aConfig);
    ~OptitrackStateFetcher_LibMocap() override; 

    Eigen::Matrix<double, 6, 1> fetchState() override; 
    bool init() override; 

private:

    NetworkConfig mConfig; 
    Eigen::Matrix<double, 6, 1> mLatestState; 
    int32_t mID; 

    std::mutex mStateMutex; 
   
};
#endif //OPTITRACKSTATEFETCHER_LIBMOCAP_H