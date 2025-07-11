
#include "abv_controller/ThrusterCommander.h"

ThrusterCommander::ThrusterCommander() : 
    mConfig(ConfigurationManager::getInstance()->getThrusterConfig()), 
    mThrusterCommand("900000000")
{
    mMatrixOfThrustDirCombinations << 1, -1, 0, 0, 0, 0, 1, 1, -1, -1, 1, 1, -1, -1, 0, 0, 0, 0, 1, 1, 1, -1, 1, -1, -1, -1, 0,
			                          0, 0, 1, -1, 0, 0, 1, -1, 1, -1, 0, 0, 0, 0, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, -1, -1, 0,
			                          0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1, -1, -1, 1, -1, 0;
}

ThrusterCommander::~ThrusterCommander()
{
}

bool ThrusterCommander::init()
{
    mUdpClient = std::make_unique<UdpClient>(mConfig.arduino.IP,mConfig.arduino.CmdPort);
}

void ThrusterCommander::commandThrusters(Eigen::Vector3d aControlInput)
{
    // convert control input to thruster dir vector
    Eigen::Vector3i thrustDirVector = convertToThrustVector(aControlInput); 

    // convert thrust dir vector into thruster combination 
    determineThrusterCommand(thrustDirVector); 

    // send to thrusters via UDP
    mUdpClient->send(mThrusterCommand); 
} 

Eigen::Vector3i ThrusterCommander::convertToThrustVector(Eigen::Vector3d aControlInput)
{
    Eigen::Vector3i thrustDir;
    double uOn = mConfig.uOn;
    double uOff = mConfig.uOff;
        
    for(int i = 0; i < 3; i++)
    {
        // determine the thrust direction based on the control input
        if (aControlInput[i] >= uOn)
		{
			thrustDir[i] = 1;
		}
		else if (aControlInput[i] < -uOn)
		{
			thrustDir[i] = -1;
		}
		else if (aControlInput[i] <= uOff && aControlInput[i] >= -uOff)
		{
			thrustDir[i] = 0;
		}
		else
		{
			thrustDir[i] = 0;
		}

    }

    //LOGW << "ThrustDir" << thrustDir; 
    return thrustDir;
}

void ThrusterCommander::determineThrusterCommand(Eigen::Vector3i aThrustDir)
{
    std::lock_guard<std::mutex> lock(mThrusterCommandMutex);

    // calculate the thruster command sequence based on the control input
    if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 0))) // +x
    {
        mThrusterCommand = "900000011";
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 1))) // -x
    {
        mThrusterCommand = "900110000";
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 2))) // +y
    {
        mThrusterCommand = "911000000";
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 3))) // -y
    {
        mThrusterCommand = "900001100";
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 4))) // +phi
    {					
        mThrusterCommand = "901000100";
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 5))) // -phi
    {
        mThrusterCommand = "910001000";
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 6))) // +x +y
    {
        mThrusterCommand = "901000010";
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 7))) // +x -y
    {
        mThrusterCommand = "900001001";
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 8))) // -x +y
    {
        mThrusterCommand = "910010000";
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 9))) // -x -y
    {
        mThrusterCommand = "900100100";
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 10))) // +x +phi
    {
        mThrusterCommand = "900000001";
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 11))) // +x -phi
    {
        mThrusterCommand = "900000010";
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 12))) // -x +phi
    {
        mThrusterCommand = "900010000";
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 13))) // -x -phi
    {
        mThrusterCommand = "900100000";
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 14))) // +y +phi
    {
        mThrusterCommand = "901000000";
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 15))) // +y -phi
    {
        mThrusterCommand = "910000000";
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 16))) // -y +phi
    {
        mThrusterCommand = "900000100";
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 17))) // -y -phi
    {
        mThrusterCommand = "900001000";
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 18))) // +x +y +phi
    {
        mThrusterCommand = "901000001";
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 19))) // +x +y -phi
    {
        mThrusterCommand = "910000010";
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 20))) // +x -y +phi
    {
        mThrusterCommand = "900000101";
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 21))) // -x +y +phi
    {
        mThrusterCommand = "901010000";
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 22))) // +x -y -phi
    {
        mThrusterCommand = "900001010";
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 23))) // -x +y -phi
    {
        mThrusterCommand = "910100000";
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 24))) // -x -y +phi
    {
        mThrusterCommand = "900010100";
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 25))) // -x -y -phi
    {
        mThrusterCommand = "900101000";
    }
    else if (aThrustDir.isApprox(mMatrixOfThrustDirCombinations.block<3, 1>(0, 26))) // nothing
    {
        mThrusterCommand = "900000000";
    }
}