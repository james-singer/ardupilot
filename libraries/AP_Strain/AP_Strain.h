#pragma once


#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <Filter/DerivativeFilter.h>
#include <AP_MSP/msp.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>



#define STRAIN_MAX_INSTANCES 1
#define STRAIN_SENSORS 10
#define BUS_NUMBER = 0
// timeouts for health reporting
#define STRAIN_TIMEOUT_MS                 500     // timeout in ms since last successful read
#define STRAIN_DATA_CHANGE_TIMEOUT_MS    2000     // timeout in ms since first strain gauge reading changed 

class AP_Strain_Backend;

class AP_Strain
{
    friend class AP_Strain_Backend;

    public:
    // constructor
    AP_Strain();

    // Do not allow copies 
    CLASS_NO_COPY(AP_Strain);

    // get singleton
    static AP_Strain *get_singleton(void) 
    {
        return _singleton;
    }

    // initialise the strain object, loading backend drivers
    void init(void);
    
    enum class Status {
        NotConnected   = 0,
        NoData         = 1,
        Good           = 2,
    };

    // update the strain object, asking backends to push data to
    // the frontend
    // void update(void);

// AP_Strain_Backend *get_backend(uint8_t id) const;

    int32_t* get_data(uint8_t instance);
    AP_Strain::Status get_status(uint8_t instance);
    uint32_t get_last_update(uint8_t instance);

    bool reset_all();
    bool calibrate_all();
    bool get_status_all();

    // int32_t get_data(void) const { return get_data(_primary); }
    // int32_t get_data(uint8_t instance) const { return sensors[instance].data[1]; } //??????????????????????????????????????? fix array pointer 

    // get last time sample was taken (in ms)
    // uint32_t get_last_update(void) const { return get_last_update(_primary); }
    // uint32_t get_last_update(uint8_t instance) const { return sensors[instance].last_update_ms; }

    // void calibrate(bool save=true);


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
        static const uint8_t num_data = STRAIN_SENSORS;
        int32_t data[num_data];                   // 10 strain gauge measurements
        enum AP_Strain::Status status;
        uint8_t I2C_id;
        
    } sensors[STRAIN_MAX_INSTANCES];


    
};

namespace AP {
    AP_Strain &strain();
};
