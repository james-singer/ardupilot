#pragma once

class Disturbance {

public:

    // constructor
    Disturbance();

    // initializes the chirp object
    void init();

    // determine chirp signal output at the specified time and amplitude
    float update(float time);

    // Return true if chirp is completed
    bool completed() const { return complete; }

private:

    // output of chirp signal at the requested time
    float output;

    // True if chirp is complete, reset to false on init
    bool complete;

    // Total chirp length in seconds
    float timespan = 1.0f;

    float time_delay = 5.0f;

    float magnitude = 0.0f;

};
