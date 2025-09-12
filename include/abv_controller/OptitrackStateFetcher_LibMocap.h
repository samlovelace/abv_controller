#ifndef OPTITRACKSTATEFETCHER_LIBMOCAP_H
#define OPTITRACKSTATEFETCHER_LIBMOCAP_H
 
#include "IStateFetcher.h"
#include <memory>
#include <thread> 

#include "ConfigurationManager.h"
#include "Configurations.h" 

#include "libmotioncapture/motioncapture.h"

class OptitrackStateFetcher_LibMocap : public IStateFetcher
{ 
public:
    OptitrackStateFetcher_LibMocap(NetworkConfig aConfig, int aRigidBodyId, const std::string& aRigidBodyName);
    ~OptitrackStateFetcher_LibMocap() override; 

    Eigen::Matrix<double, 6, 1> fetchState() override; 
    bool init() override; 

    void setLatestState(Eigen::Matrix<double, 6, 1> aLatestState) {std::lock_guard<std::mutex> lock(mStateMutex); mLatestState = aLatestState; }

private:

    std::unique_ptr<libmotioncapture::MotionCapture> mMocap; 

    NetworkConfig mConfig; 
    Eigen::Matrix<double, 6, 1> mLatestState; 
    int mID; 
    std::string mRigidBodyName; 

    std::mutex mStateMutex; 

    std::thread mListenThread; 
    void listen(); 
   
};
#endif //OPTITRACKSTATEFETCHER_LIBMOCAP_H