
#include "abv_controller/OptitrackStateFetcher.h"

OptitrackStateFetcher::OptitrackStateFetcher() : mNatNetClient(nullptr)
{

}

OptitrackStateFetcher::~OptitrackStateFetcher()
{
    
}

bool OptitrackStateFetcher::init() 
{
    if(mNatNetClient)
    {
        mNatNetClient->Disconnect(); 
    }

    mNatNetClient = std::make_unique<NatNetClient>(); 
    //mNatNetClient->SetFrameReceivedCallback(OptitrackStateFetcher::DataHandler, mNatNetClient); 

}

Eigen::Matrix<double, 6, 1> OptitrackStateFetcher::fetchState()
{
    // mutex protected access to the latest data recvd from optitrack 
    Eigen::Matrix<double, 6, 1> state; 
    return state; 
}


void NATNET_CALLCONV OptitrackStateFetcher::DataHandler(sFrameOfMocapData* data, void* pUserData)
{
        //std::cout << "DataHandler: " << std::endl; 
        // rigid bodies
        // int rbcount = min(RigidBodyCollection::MAX_RIGIDBODY_COUNT, data->nRigidBodies);
        // rigidBodies.SetRigidBodyData(data->RigidBodies, rbcount);

        // // Set time
        // time_DH = data->fTimestamp;

        // // timecode
        // NatNetClient* pClient = (NatNetClient*)pUserData;
        // int hour, minute, second, frame, subframe;
        // NatNet_DecodeTimecode(data->Timecode, data->TimecodeSubframe, &hour, &minute, &second, &frame, &subframe);
        // // decode timecode into friendly string
        // NatNet_TimecodeStringify(data->Timecode, data->TimecodeSubframe, szTimecode, 128);

        // OT_rcvd = true;
        // Update(); 
}

