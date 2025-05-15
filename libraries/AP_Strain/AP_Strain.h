#pragma once


#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <Filter/DerivativeFilter.h>
#include <AP_MSP/msp.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>

// maximum number of sensor instances
// #ifndef STRAIN_MAX_INSTANCES
// #define STRAIN_MAX_INSTANCES 1
// #endif
#define STRAIN_MAX_INSTANCES 1


// timeouts for health reporting
#define STRAIN_TIMEOUT_MS                 500     // timeout in ms since last successful read
#define STRAIN_DATA_CHANGE_TIMEOUT_MS    2000     // timeout in ms since first strain gauge reading changed 

class AP_Strain_Backend;

class AP_Strain
{
    friend class AP_Strain_Backend;

    public:
    AP_Strain();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Strain);

    // get singleton
    static AP_Strain *get_singleton(void) 
    {
        return _singleton;
    }
    // test comment

    // initialise the strain object, loading backend drivers
    void init(void);

    // update the strain object, asking backends to push data to
    // the frontend
    void update(void);

    AP_Strain_Backend *get_backend(uint8_t id) const;

    int32_t get_data(void) const { return get_data(_primary); }
    int32_t get_data(uint8_t instance) const { return sensors[instance].data; } //??????????????????????????????????????? fix array pointer 

    // get last time sample was taken (in ms)
    uint32_t get_last_update(void) const { return get_last_update(_primary); }
    uint32_t get_last_update(uint8_t instance) const { return sensors[instance].last_update_ms; }

    // void calibrate(bool save=true);

    enum class Status {
        NotConnected   = 0,
        NoData         = 1,
        Good           = 2,
    };

    ////////////////////////////////////////////////////////////////////////////////////////////// 
    private:
    // singleton
    static AP_Strain *_singleton;
    // how many drivers do we have?
    AP_Strain_Backend *drivers[STRAIN_MAX_INSTANCES];
    // how many sensors do we have?
    uint8_t _num_sensors = 0;
    uint8_t _primary = 0;
    bool init_done = false;
    struct sensor
    {
        uint32_t last_update_ms;        // last update time in ms
        uint32_t last_change_ms;        // last update time in ms that included a change in reading from previous readings
        uint8_t num_data = 10;
        int32_t data;                   // 10 strain gauge measurements
        enum AP_Strain::Status status;
        bool healthy;                   // true if sensor is healthy
        bool calibrated;                // true if calculated calibrated successfully
        uint8_t I2C_id;
    } sensors[STRAIN_MAX_INSTANCES];


    // bool _add_backend(AP_Strain_Backend *backend , uint8_t instance);

    // void detect_instance(uint8_t instance);

};

namespace AP {
    AP_Strain &strain();
};
