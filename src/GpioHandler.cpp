
#include "abv_controller/GpioHandler.h"
#include "plog/Log.h"
#include <algorithm>

GpioHandler::GpioHandler(std::vector<int> aSetOfOutputPins) : mOutputPins(aSetOfOutputPins)
{
}

GpioHandler::~GpioHandler()
{
    writeAll(0); 
    gpioTerminate(); 
}

bool GpioHandler::init()
{
    if(-1 == gpioInitialise())
    {
        LOGE << "Unable to initialize gpio"; 
        return false; 
    }

    LOGD << "Gpio Initialized successfully!"; 

    for(const auto& pin : mOutputPins)
    {
        if(-1 == gpioSetMode(pin, JET_OUTPUT))
        {
            LOGE << "Unable to set pin: " << pin << "to mode: " << JET_OUTPUT; 
            return false; 
        } 

        LOGD << "set Pin: " << pin << " Mode: " << JET_OUTPUT;  
    }

    LOGD << "Initialized all output pins successfully"; 
    
    // Other setup stuff here 

    return true; 
}

bool GpioHandler::fini()
{
    gpioTerminate(); 
}

bool GpioHandler::areAllPinsOff()
{
    for(const auto& pin : mOutputPins)
    {
        // TODO: i am assuming this returns 1 if the pin is high? 
        if(1 == gpioRead(pin))
        {
            return false; 
        }
    }

    return true; 
}

void GpioHandler::writeAll(const int aState)
{
    for(const auto& pin : mOutputPins)
    {
        // skipping isOutputPin check since we are looping through mOutputPins
        gpioWrite(pin, aState); 
    }
}

void GpioHandler::writePin(int aPin, int aState)
{
    if(isOutputPin(aPin))
        gpioWrite(aPin, aState); 
}

void GpioHandler::writePins(std::vector<int> aSetOfPins, int aState)
{
    for(const auto& pin : aSetOfPins)
    {
        writePin(pin, aState); 
    }
}

bool GpioHandler::isOutputPin(int aPin) 
{
    return std::find(mOutputPins.begin(), mOutputPins.end(), aPin) != mOutputPins.end();
}