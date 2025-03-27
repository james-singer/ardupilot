#include "AP_Strain_Backend.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <stdio.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_HAL/utility/sem.h>

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
        
    return true;
}

void AP_Strain_Backend::timer(void)
{

    if (get_reading(_sensor)) {
        update_status();
    } else {
        set_status(AP_Strain::Status::NoData);
    }
}

//////////////////////////////////////// TODO - implement get reading and parse stream
// read - return sensor structure
bool AP_Strain_Backend::get_reading(AP_Strain::sensor &sensor_arg)
{



    /////////////////////////////////////////////////////////////////////////////////////////// line 344 AP_RangeFinder_LightWareI2C.cpp
    // be16_t val;

    // const uint8_t read_reg = LIGHTWARE_DISTANCE_READ_REG;

    // // read the high and low byte distance registers
    // if (_dev->transfer(&read_reg, 1, (uint8_t *)&val, sizeof(val))) {
    //     int16_t signed_val = int16_t(be16toh(val));
    //     if (signed_val < 0) {
    //         // some lidar firmwares will return 65436 for out of range
    //         reading_m = uint16_t(max_distance_cm() + LIGHTWARE_OUT_OF_RANGE_ADD_CM) * 0.01f;
    //     } else {
    //         reading_m = uint16_t(signed_val) * 0.01f;
    //     }
    //     return true;
    // }
    // return false;

    ///////////////////////////////////////////////////////////////////////////////////////// line 366 AP_RangeFinder_LightWareI2C.cpp
     // Parses up to 5 ASCII streams for LiDAR data.
    // If a parse fails, the stream measurement is not updated until it is successfully read in the future.
    // uint8_t stream[lx20_max_expected_stream_reply_len_bytes+1]; // Maximum response length for a stream ie "ldf,0:40.99" is 11 characters

    // uint8_t i = streamSequence[currentStreamSequenceIndex];
    // size_t num_processed_chars = 0;

    // /* Reads the LiDAR value requested during the last interrupt. */
    // if (!_dev->read(stream, sizeof(stream))) {
    //     return false;
    // }
    // stream[lx20_max_expected_stream_reply_len_bytes] = 0;

    // if (!sf20_parse_stream(stream, &num_processed_chars, parse_stream_id[i], sf20_stream_val[i])) {
    //     return false;
    // }

    // if (i==0) {
    //     reading_m = sf20_stream_val[0] * 0.01f;
    // }

    // // Increment the stream sequence
    // currentStreamSequenceIndex++;
    // if (currentStreamSequenceIndex >= numStreamSequenceIndexes) {
    //     currentStreamSequenceIndex = 0;
    // }
    // i = streamSequence[currentStreamSequenceIndex];

    // // Request the next stream in the sequence from the SF20
    // write_bytes((uint8_t*)init_stream_id[i], strlen(init_stream_id[i]));

    // return true;

}

bool AP_RangeFinder_LightWareI2C::strain_parse_stream(uint8_t *stream_buf,
    size_t *p_num_processed_chars,
    const char *string_identifier,
    uint16_t &val)
{
///////////////////////////////////////////////////////////////////////// line 400 AP_RangeFinder_LightWareI2C.cpp

// size_t string_identifier_len = strlen(string_identifier);

// for (uint32_t i = 0 ; i < string_identifier_len ; i++) {
//     if (stream_buf[*p_num_processed_chars] != string_identifier[i]) {
//         return false;
//     }
//     (*p_num_processed_chars)++;
// }

// /*
//   special case for being beyond maximum range, we receive a message like this:
//   ldl,1:-1.00
//   we will return max distance
//  */
// if (strncmp((const char *)&stream_buf[*p_num_processed_chars], "-1.00", 5) == 0) {
//     val = uint16_t(max_distance_cm() + LIGHTWARE_OUT_OF_RANGE_ADD_CM);
//     (*p_num_processed_chars) += 5;
//     return true;
// }

// /* Number is always returned in hundredths. So 6.33 is returned as 633. 6.3 is returned as 630.
//  * 6 is returned as 600.
//  * Extract number in format 6.33 or 123.99 (meters to be converted to centimeters).
//  * Percentages such as 100 (percent), are returned as 10000.
//  */
// uint32_t final_multiplier = 100;
// bool decrement_multiplier = false;
// bool number_found = false;
// uint16_t accumulator = 0;
// uint16_t digit_u16 = (uint16_t)stream_buf[*p_num_processed_chars];
// while ((((digit_u16 <= '9') &&
//          (digit_u16 >= '0')) ||
//         (digit_u16 == '.')) &&
//        (*p_num_processed_chars < lx20_max_reply_len_bytes)) {
//     (*p_num_processed_chars)++;
//     if (decrement_multiplier) {
//         final_multiplier /=10;
//     }
//     if (digit_u16 == '.') {
//         decrement_multiplier = true;
//         digit_u16 = (uint16_t)stream_buf[*p_num_processed_chars];
//         continue;
//     }
//     number_found = true;
//     accumulator *= 10;
//     accumulator += digit_u16 - '0';
//     digit_u16 = (uint16_t)stream_buf[*p_num_processed_chars];
// }
//
// accumulator *= final_multiplier;
// val = accumulator;
// return number_found;
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

////////// public

void AP_Strain_Backend::update(void)
{
    // nothing to do - its all done in the timer()
}


//////////////////////////////////////// TODO - implement calibrate
void AP_Strain_Backend::calibrate();
{
    // TODO - implement
    // write_bytes
    // send the byte 'Z' to the sensor 
    // zeroing happens on board the strain arm
}

// true if sensor is returning data
bool AP_Strain_Backend::has_data() const {
    return ((_sensor.status != AP_Strain::Status::NotConnected) &&
            (_sensor.status != AP_Strain::Status::NoData));
}
