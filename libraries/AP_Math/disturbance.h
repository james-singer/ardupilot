#pragma once

class Disturbance {

public:

    // constructor
    Disturbance();

    // initializes the disturbance object
    void init();

    // determine disturbance signal output at the specified time and amplitude
    float update(float time);

    // Return true if disturbance is completed
    bool completed() const { return complete; }

private:

    // output of disturbance signal at the requested time
    float output;

    // True if disturbance is complete, reset to false on init
    bool complete;

    // Total disturbance length in seconds
    float timespan = 0.3f;

    float time_delay = 3.0f;

    float magnitude = 0.0f;

};
