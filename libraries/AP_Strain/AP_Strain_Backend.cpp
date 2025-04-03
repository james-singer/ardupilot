#include "AP_Strain_Backend.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <stdio.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

// constructor
AP_Strain_Backend::AP_Strain_Backend(AP_Strain::sensor &_strain_arm, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev) : 
    _sensor(_strain_arm),
    _dev(std::move(dev)) {}

////////// private

bool AP_Strain_Backend::write_bytes(uint8_t *write_buf_u8, uint32_t len_u8)
{
    return _dev->transfer(write_buf_u8, len_u8, NULL, 0);
}


bool AP_Strain_Backend::init()
{

    DEV_PRINTF("I2C starting\n");

    // call timer() at 80Hz
    _dev->register_periodic_callback(12500, FUNCTOR_BIND_MEMBER(&AP_Strain_Backend::timer, void));
    
    uint8_t send_msg = 0x50;
    
    if (!write_bytes(&send_msg, 1)) {
        return false;
    }

    return true;
}

void AP_Strain_Backend::timer(void)
{

    if (get_reading(_sensor)) {
        set_status(AP_Strain::Status::Good);
    } else {
        set_status(AP_Strain::Status::NoData);
    }
}

//////////////////////////////////////// TODO - implement get reading and parse stream
// read - return sensor structure
bool AP_Strain_Backend::get_reading(AP_Strain::sensor &sensor_arg)
{
    uint8_t buffer[4]; // Buffer to hold the 4 bytes read from the I2C bus

    // Attempt to read 4 bytes from the I2C device
    if (!_dev->read(buffer, sizeof(buffer))) {
        return false; // Return false if the read operation fails
    }

    // Combine the 4 bytes into a single int32_t value
    int32_t combined_value = (int32_t(buffer[0]) << 24) |
                             (int32_t(buffer[1]) << 16) |
                             (int32_t(buffer[2]) << 8)  |
                             int32_t(buffer[3]);
                             

    // Write the combined value into the sensor's data field
    sensor_arg.data = combined_value;

    return true; // Return true to indicate success
}

AP_Strain_Backend *AP_Strain_Backend::detect(AP_Strain::sensor &_strain_arm ,AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    if (!dev) {
        return nullptr;
    }

    AP_Strain_Backend *sensor
        = NEW_NOTHROW AP_Strain_Backend(_strain_arm, std::move(dev));

    if (!sensor) {
        return nullptr;
    }

    WITH_SEMAPHORE(sensor->_dev->get_semaphore());
    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

// set status and update valid count
void AP_Strain_Backend::set_status(AP_Strain::sensor &_strain_arg, AP_Strain::Status status)
{
    _strain_arg.status = status;
}

////////// public
void AP_Strain_Backend::update(void)
{
    // nothing to do - its all done in the timer()
}


// true if sensor is returning data
bool AP_Strain_Backend::has_data() const {
    return ((_sensor.status != AP_Strain::Status::NotConnected) &&
            (_sensor.status != AP_Strain::Status::NoData));
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
