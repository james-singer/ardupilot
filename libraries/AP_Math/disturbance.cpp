#include <AP_Math/AP_Math.h>
#include "disturbance.h"

Disturbance::Disturbance() {}

void Disturbance::init()
{
}

float Disturbance::update(float time)
{
    if (time - time_delay <= 0.0f) 
    {
        output = 1.0f;
    } 
    else if (time - time_delay <= timespan) 
    {
        output = magnitude;
    } 
    else 
    {
        output = 1.0f;
    }  

    complete = time > timespan;
    return output; 
}