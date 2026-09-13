#include "api.h"

class inputHandler
{
private:
    double mult;
    float smoothingRate;
    double outMax;
    double outMin;

    float inputChannels[16] = {0};
    bool isChannelUsed[16] = {false};   

    //Calculated return for last finished tick
    double tickOut = 0;

    double outputRange = 0;


public:
    inputHandler(double mult, // Flat output mult
                 // Maximum time it takes for the output to reach the input
                 float smoothingRate,
                 double outMax,  // Max output value
                 double outMin // Min output value
    );

    //Finishes an inputhandler tick & calculates the filtered output
    double tick(double dT/*Time since the last tick was called*/)
    {   
        unsigned int usedChannelCount = 0;
        float avgIn = 0;
        for (int i=0; i<16; i++) {
            if (isChannelUsed[i] = true) {
                avgIn += inputChannels[i];
                usedChannelCount += 1;
            }
            
        } avgIn = avgIn / usedChannelCount;

        if (abs(avgIn*mult - tickOut) <= 1) {
            tickOut = avgIn * mult;
        } else {
            tickOut += std::clamp(avgIn*mult*dT-tickOut, outMin*dT, outMax*dT) ;
        } 

        return tickOut;
    }
    
    //Fetches the output of the last finished tick.
    double fetchTick()
    {
        return tickOut;
    }

    //Gives input for a tick, Different channels average w/ each other, rewriting to a channel overrides it.
    void input(double input, unsigned int channel)
    {
        inputChannels[channel] = input;
    }
};
