#pragma once
#include "AP_Strain.h"
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/I2CDevice.h>


class AP_Strain_Backend
{
public:
    AP_Strain_Backend(AP_Strain::sensor &_strain_arm, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev, AP_Strain* singleton);
    
    // static AP_Strain_Backend *detect(AP_Strain::sensor &_strain_arm, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);
    
    bool init();

    // each driver must provide an update method to copy accumulated
    // data to the frontend
    // void update(void);

    float data() const { return 1.0f; }

    // true if sensor is returning data
    bool has_data() const;

    bool reset(void);
    bool calibrate(void);



private:

    // reference to shared data struct
    AP_Strain::sensor &_sensor;
    AP_Strain* _singleton;

    // semaphore for access to shared frontend data
    // HAL_Semaphore _sem;

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

    // set status and update valid_count
    static void set_status(AP_Strain::sensor &_strain_arg, AP_Strain::Status status);
    void set_status(AP_Strain::Status status) { set_status(_sensor, status); }

    bool write_byte(uint8_t write_byte);

    void timer();

    bool get_reading();

    void update_last_change_ms(bool reset);
    
    // bool strain_parse_stream(uint8_t *stream_buf,
    //     size_t *p_num_processed_chars,
    //     const char *string_identifier,
    //     uint16_t &val);

    // AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

};
