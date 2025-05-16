/*
 *  RangeFinder test code
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_RangeFinder/AP_RangeFinder_Backend.h>
#include <GCS_MAVLink/GCS_Dummy.h>
// #include <AP_Strain/AP_Strain_Backend.h>


const struct AP_Param::GroupInfo        GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};
GCS_Dummy _gcs;

void setup();
void loop();

// extern const AP_HAL::HAL& hal;
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static AP_SerialManager serial_manager;
static RangeFinder sonar;

static AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev_temp = nullptr;
// static AP_Strain strain;
static uint8_t poll = 0x50;


void setup()
{
    // // print welcome message
    // hal.console->printf("Range Finder library test\n");

    // // setup for analog pin 13
    // AP_Param::set_object_value(&sonar, sonar.var_info, "_TYPE", (uint8_t)RangeFinder::Type::PLI2C);
    // AP_Param::set_object_value(&sonar, sonar.var_info, "_PIN", -1.0f);
    // AP_Param::set_object_value(&sonar, sonar.var_info, "_SCALING", 1.0f);

    // // initialise sensor, delaying to make debug easier
    // hal.scheduler->delay(2000);
    // // sonar.init(ROTATION_PITCH_270);
    // // hal.console->printf("RangeFinder: %d devices detected\n", sonar.num_sensors());


    // strain
    hal.scheduler->delay(100);
    hal.console->printf("Strain test\n");
    // strain.init();
    // strain.calibrate();
    dev_temp = hal.i2c_mgr->get_device(0, 0x09);
    dev_temp->set_retries(2);
    dev_temp->set_speed(AP_HAL::Device::SPEED_HIGH);
    

}

void loop()
{   
    hal.scheduler->delay(100);
    hal.console->printf("Strain test #############################################################################\n");
    bool has_own = dev_temp->get_semaphore()->take(100);
    // hal.console->printf("dev_temp = %d\n", dev_temp->get_bus_address());
    // int32_t data;
    const uint8_t* poll_a = &poll;
    if (has_own) {
            uint8_t buffer[32]; // Buffer to hold the 4 bytes read from the I2C bus
        if (dev_temp->transfer(poll_a, 1,NULL, 0)) {
            hal.console->printf("'P' sent\n");
        }
        else 
        {
            hal.console->printf("failed to send\n");
        }

        if (dev_temp->read(buffer, sizeof(buffer))) {

            int32_t combined_value = (int32_t(buffer[0]) ) |
                                    (int32_t(buffer[1]) << 8) |
                                    (int32_t(buffer[2]) << 16)  |
                                    (int32_t(buffer[3]) << 24 );
                                    
            hal.console->printf("Data: %ld\n", combined_value);
            
        }
        else
        {
            hal.console->printf("failed to read\n");
        }
    }
    else 
    {
        hal.console->printf("failed to get semaphore\n");
    }    

    
    dev_temp->get_semaphore()->give();
    
    // Write the combined value into the sensor's data field

//     // Delay between reads
//     hal.scheduler->delay(100);
//     sonar.update();

//     bool had_data = false;
//     for (uint8_t i=0; i<sonar.num_sensors(); i++) {
//         AP_RangeFinder_Backend *sensor = sonar.get_backend(i);
//         if (sensor == nullptr) {
//             continue;
//         }
//         if (!sensor->has_data()) {
//             continue;
//         }
//         hal.console->printf("All: device_%u type=%d status=%d distance=%f\n",
//                             i,
//                             (int)sensor->type(),
//                             (int)sensor->status(),
//                             sensor->distance());
//         had_data = true;
//     }
//     if (!had_data) {
//         hal.console->printf("All: no data on any sensor\n");
//     }

    // Delay between reads
    // hal.scheduler->delay(100);
    // strain.update();

    // bool had_data = false;
    // for (uint8_t i=0; i<1; i++) {
    //     AP_Strain_Backend *sensor = strain.get_backend(i);
    //     if (sensor == nullptr) {
    //         continue;
    //     }

    //     hal.console->printf("val=%f\n",
    //                         sensor->data());
    //     had_data = true;
    // }
    // if (!had_data) {
    //     hal.console->printf("All: no data on any sensor\n");
    // }
}
AP_HAL_MAIN();
