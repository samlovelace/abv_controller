
#include "abv_controller/OptitrackStateFetcher.h"

OptitrackStateFetcher::OptitrackStateFetcher()
{

}

OptitrackStateFetcher::~OptitrackStateFetcher()
{
    
}

Eigen::Matrix<double, 6, 1> OptitrackStateFetcher::fetchState()
{
    Eigen::Matrix<double, 6, 1> state; 
    return state; 
}