/*
 *  RangeFinder test code
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_RangeFinder/AP_RangeFinder_Backend.h>
#include <GCS_MAVLink/GCS_Dummy.h>
#include <AP_Strain/AP_Strain_Backend.h>


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
static AP_Strain strain;
// static uint8_t poll = 0x50;

int32_t count = 0;
void setup()
{

    // strain
    hal.scheduler->delay(100);
    hal.console->printf("Strain test\n");
    strain.init();
    strain.calibrate_all();
    // dev_temp = hal.i2c_mgr->get_device(0, 0x09);
    // dev_temp->set_retries(2);
    // dev_temp->set_speed(AP_HAL::Device::SPEED_HIGH);


    

}

void loop()
{  
    // if (count++ > 100)
    // {
    //     count = 0;
    //     if (strain.calibrate())
    //     {
    //         hal.console->printf("zeroing\n");
    //         hal.scheduler->delay(10);
    //     }
    //     else
    //     {
    //         hal.console->printf("failed to zeroing\n");
    //         hal.scheduler->delay(10);
    //     }
    // }

    int32_t* data = strain.get_data(0);
    uint32_t last_update = strain.get_last_update(0);

    hal.console->printf("----------------------------------\n");
    hal.console->printf("time: %ld\n", last_update);
    for (uint8_t i = 0; i < 10; i++)
    {
        hal.console->printf("Strain gauge %d: %ld\n", i+1, data[i]);

    }
    hal.console->printf("----------------------------------\n");
    hal.scheduler->delay(100);
    // hal.console->printf("Strain test #############################################################################\n");
    // bool has_own = dev_temp->get_semaphore()->take(100);
    // // hal.console->printf("dev_temp = %d\n", dev_temp->get_bus_address());
    // // int32_t data;
    // const uint8_t* poll_a = &poll;
    // if (has_own) {
    //         uint8_t buffer[32]; // Buffer to hold the 4 bytes read from the I2C bus
    //     if (dev_temp->transfer(poll_a, 1,NULL, 0)) {
    //         hal.console->printf("'P' sent\n");
    //     }
    //     else 
    //     {
    //         hal.console->printf("failed to send\n");
    //     }

    //     if (dev_temp->read(buffer, sizeof(buffer))) {

    //         int32_t combined_value = (int32_t(buffer[0]) ) |
    //                                 (int32_t(buffer[1]) << 8) |
    //                                 (int32_t(buffer[2]) << 16)  |
    //                                 (int32_t(buffer[3]) << 24 );
                                    
    //         hal.console->printf("Data: %ld\n", combined_value);
            
    //     }
    //     else
    //     {
    //         hal.console->printf("failed to read\n");
    //     }
    // }
    // else 
    // {
    //     hal.console->printf("failed to get semaphore\n");
    // }    

    
    // dev_temp->get_semaphore()->give();
    
    
}
AP_HAL_MAIN();
