#include "AP_Strain_Backend.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <stdio.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/utility/sparse-endian.h>

#define TIMEOUT 100

extern const AP_HAL::HAL& hal;

// constructor
AP_Strain_Backend::AP_Strain_Backend(AP_Strain::sensor &_strain_arm, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev, AP_Strain* singleton) : 
    _frontEnd(singleton),
    _sensor(_strain_arm),
    _dev(std::move(dev)) {}

////////// private

bool AP_Strain_Backend::write_byte(uint8_t write_byte)
{
    uint8_t msg = write_byte;
    return _dev->transfer(&msg, sizeof(msg), NULL, 0);
}


bool AP_Strain_Backend::init()
{

    DEV_PRINTF("I2C starting\n");

    // call timer() at 80Hz
    _dev->register_periodic_callback(12500, FUNCTOR_BIND_MEMBER(&AP_Strain_Backend::timer, void));
    
    // uint8_t send_msg = 0x50;
    
    if (!write_byte(0x50)) {
        return false;
    }

    return true;
}

void AP_Strain_Backend::timer(void)
{
    // if (get_reading(_sensor)) {
    //     set_status(AP_Strain::Status::Good);
    // } else {
    //     set_status(AP_Strain::Status::NoData);
    // }

    // Boolean read will be used throughout the function
    bool read = false;

    // Store the old data
    // Joe - Eventually we might want to check data from all strain gauges before arming
    // For now, we will assume either all sensors are working or none of them are working
    int32_t last_data = _sensor.data[0];
    int32_t last_time = _sensor.last_update_ms;

    // Get the semaphore
    bool has_sem = _dev->get_semaphore()->take(100);
    if (has_sem)
    {
        // If we have the semaphore, call get_reading and store the return value in read
        read = get_reading();
        _dev->get_semaphore()->give();
    }
    // Key note here: do not worry about the case where we fail to get semaphore... setting read to false by default will handle this case
    // If read returned true, it means we successfully read in data (possibly bad data, but that will be handled later)
    // Set the status
    if (read)
    {
        _sensor.status = AP_Strain::Status::Good;
    }
    // If read returned false, either our writing poll to sensor or reading data from sensor failed (or potentially we did not get the semaphore)
    else
    {
        _sensor.status = AP_Strain::Status::NotConnected;
    }

    // If the sensor data did not change, we need to call update_last_change_ms with the argument as false to extend the last change time
    if (_sensor.data[0] == last_data)
    {
        update_last_change_ms(false , last_time);
    }
    // Otherwise, reset the last change time to zero by calling update_last_change_ms with the argument as true
    else
    {
        update_last_change_ms(true , last_time);
    }

    // Final check: if last change time is greater than timeout, we need to reset the arm and calibrate all sensors
    if (_sensor.last_change_ms > TIMEOUT)
    {
        // Reset method will send 'R' to this particular arm ONLY
        reset();
        // Calling front end method will calibrate ALL strain arms
        _frontEnd->calibrate_all();
        _sensor.status = AP_Strain::Status::NoData;    
    }

    // Old code:
    // if (has_sem)
    // {
    //     if (!get_reading())
    //     {
    //         _sensor.status = AP_Strain::Status::NotConnected;
    //     }
    //     else if (_sensor.last_change_ms > TIMEOUT)
    //     {
    //         _sensor.status = AP_Strain::Status::NoData;
            
    //     }
    //     else
    //     {
    //         _sensor.status = AP_Strain::Status::Good;
    //     } 
    //     _dev->get_semaphore()->give();
    // }
    // else
    // {
    //     // Failed to get semaphore
    //     // Joe - Update last change time
    // }

    // // Joe - Check if last change is too large and if so set global armed flag and reset/calibrate
}

void AP_Strain_Backend::update_last_change_ms(bool reset, int32_t last_time)
{
    // If the boolean argument is true, reset the last change time to 0
    if (reset)
        _sensor.last_change_ms = 0;
    else
        _sensor.last_change_ms += AP_HAL::millis() - last_time;
}

//////////////////////////////////////// TODO - implement get reading and parse stream
// read - return sensor structure
bool AP_Strain_Backend::get_reading()
{

    // Joe - commenting out... old data now stored and checked in timer
    // int32_t data_last = _sensor.data[0];
    // int32_t time_last = _sensor.last_update_ms;

    // Create the buffer
    uint8_t buffer[_sensor.num_data*4];

    // Write "P" to sensor
    // uint8_t poll = 0x50;
    // const uint8_t* poll_a = &poll;
    if (!write_byte(0x50)) 
    {
        // Writing to sensor failed
        // Return false and deal with it in timer
        return false;
    }

    // Read bytes into the buffer
    if (!_dev->read(buffer, sizeof(buffer)))
    {
        // Reading from sensor failed
        // Return false and deal with it in timer
        return false;
    }

    // Buffer now stores all 40 bytes
    // Iterate the bytes 4 at a time and shift to form 32 bit signed integers
    for (uint8_t i = 0; i < _sensor.num_data; i++)
    {
        // For each sensor, combine the four bytes to form a 32 bit signed integer (assuming little endian)
        _sensor.data[i] = (int32_t) buffer[i*4]  |
                 ((int32_t) buffer[i*4+1] << 8)  | 
                 ((int32_t) buffer[i*4+2] << 16) | 
                 ((int32_t) buffer[i*4+3] << 24);
    }
    uint32_t current_time = AP_HAL::millis();
    _sensor.last_update_ms = current_time;

    // Joe - commenting out here... sensor last change time now updated in timer
    // if (data_last == _sensor.data[0])
    // {
    //     _sensor.last_change_ms += current_time - time_last;
    // }
    // else
    // {
    //     _sensor.last_change_ms = 0;
    // }
    return true;

    // uint8_t buffer[4]; // Buffer to hold the 4 bytes read from the I2C bus

    // // Attempt to read 4 bytes from the I2C device
    // if (!_dev->read(buffer, sizeof(buffer))) {
    //     return false; // Return false if the read operation fails
    // }

    // // Combine the 4 bytes into a single int32_t value
    // int32_t combined_value = (int32_t(buffer[0]) << 24) |
    //                          (int32_t(buffer[1]) << 16) |
    //                          (int32_t(buffer[2]) << 8)  |
    //                          int32_t(buffer[3]);
                             

    // // Write the combined value into the sensor's data field
    // sensor_arg.data[1] = combined_value;

    // return true; // Return true to indicate success
}

// AP_Strain_Backend *AP_Strain_Backend::detect(AP_Strain::sensor &_strain_arm ,AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
// {
//     if (!dev) {
//         return nullptr;
//     }

//     AP_Strain_Backend *sensor
//         = NEW_NOTHROW AP_Strain_Backend(_strain_arm, std::move(dev));

//     if (!sensor) {
//         return nullptr;
//     }

//     WITH_SEMAPHORE(sensor->_dev->get_semaphore());
//     if (!sensor->init()) {
//         delete sensor;
//         return nullptr;
//     }
//     return sensor;
// }

// set status and update valid count
void AP_Strain_Backend::set_status(AP_Strain::sensor &_strain_arg, AP_Strain::Status status)
{
    _strain_arg.status = status;
}

////////// public
// void AP_Strain_Backend::update(void)
// {
//     // nothing to do - its all done in the timer()
// }


// true if sensor is returning data
bool AP_Strain_Backend::has_data() const {
    return ((_sensor.status != AP_Strain::Status::NotConnected) &&
            (_sensor.status != AP_Strain::Status::NoData));
}


bool AP_Strain_Backend::calibrate()
{
    // Objective send the byte 'Z' to the sensor
    uint8_t zero = 0x5A;
    // const uint8_t* zero_a = &zero;
    
    bool has_sem = _dev->get_semaphore()->take(20);
    if (has_sem)
    {
        if (!write_byte(zero)) 
        {
            // Writing to sensor failed
            _dev->get_semaphore()->give(); 
            return false;
        }
        else
        {
            // Writing to sensor failed
            _dev->get_semaphore()->give(); 
            return true;
        }   
    }
    else
    {
        // Failed to get semaphore
        return false;
    }
}

bool AP_Strain_Backend::reset()
{
    // Objective send the byte 'R' to the sensor
    bool has_sem = _dev->get_semaphore()->take(50);
    if (has_sem)
    {
        uint8_t msg = 0x59;
        if (!write_byte(msg))
        {
            // Error writing bytes
            _dev->get_semaphore()->give();  
            return false;
        }      
        else
        {
            // Successfully wrote 'R' to sensor
            _dev->get_semaphore()->give();
            return true;
        }
    }
    else
    {
        // Error getting semaphore
        // What else to do here?
        return false;
    }

}

//////////////////////////////////////// TODO - implement calibrate
// void AP_Strain_Backend::calibrate();
// {
//     // TODO - implement
//     // write_bytes
//     // send the byte 'Z' to the sensor 
//     // zeroing happens on board the strain arm
// }

// true if sensor is returning data
// bool AP_Strain_Backend::has_data() const {
//     return ((_sensor.status != AP_Strain::Status::NotConnected) &&
//             (_sensor.status != AP_Strain::Status::NoData));
// }
