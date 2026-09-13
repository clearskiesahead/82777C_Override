#include "api.h"


class inputHandler
{
private:
    double mult;
    float smoothRate;
    double outMax;
    double outMin;

    float inputChannels[16] = {0};
    bool isChannelUsed[16] = {false};   

    //Calculated return for last finished tick
    double tickOut = 0;


public:
    inputHandler(double mult, // Flat output mult
                 // Maximum time it takes for the output to reach the input
                 float smoothRate,
                 double outMax,  // Max output value
                 double outMin // Min output value
    );

    //Finishes an inputhandler tick & calculates the filtered output
    double tick(double dT/*Time since the last tick was called*/)
    {   
        double outRange = outMax - outMin;

        unsigned int usedChannelCount = 0;
        float sumIn = 0;
        for (int i=0; i<16; i++) {
            if (isChannelUsed[i]) {
                sumIn += inputChannels[i];
                usedChannelCount += 1;
            }
            
        } double avgIn = sumIn / usedChannelCount;

        tickOut += std::clamp
             (avgIn - tickOut,
             -smoothRate*outRange,
             smoothRate*outRange) 
             * dT;

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
