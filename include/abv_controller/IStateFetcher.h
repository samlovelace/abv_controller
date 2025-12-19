#ifndef ISTATEFETCHER_H
#define ISTATEFETCHER_H

#include <eigen3/Eigen/Dense>
#include <mutex> 
#include <atomic>

class IStateFetcher
{
public:
    virtual ~IStateFetcher() = default; 
    virtual Eigen::Matrix<float, 12, 1> fetchState() = 0; 
    virtual bool init() = 0; 
    virtual bool isStateAcquired() {return mAcquired.load(); }
    virtual std::string type() {return "unknown"; }

protected: 
    Eigen::Matrix<float, 12, 1> mState; 
    std::atomic<bool> mAcquired; 

};
#endif // ISTATEFETCHER_H