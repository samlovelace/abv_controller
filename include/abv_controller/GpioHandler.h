#ifndef GPIOHANDLER_H
#define GPIOHANDLER_H   

#include <jetgpio.h>
#include <stdexcept>
#include <unistd.h>
#include <vector> 
#include <array>

class GpioHandler
{
public:
    GpioHandler(std::vector<int> aSetOfOutputPins);
    ~GpioHandler();

    bool init(); 
    bool fini(); 

    /**
     * @brief writeAll actuates all output pins (specified in mOutputPins) to the desired state
     * 
     * @param aState the state to actuate all output pins to. 1 = ON/HIGH, 0 = OFF/LOW
     */
    void writeAll(const int aState);

    /**
     * @brief areAllPinsOff checks if all output pins (specified in mOutputPins) are off
     * 
     * @note this is a small optimization for when we arent receieving commands so we arent continuously
     *       trying to toggle pins off that are already off
     */
    bool areAllPinsOff();

    void writePin(int aPin, int aState);
    
    void writePins(std::vector<int> aSetOfPins, int aState);

    bool isOutputPin(int aPin); 

private:

    std::vector<int> mOutputPins; 

};

#endif

