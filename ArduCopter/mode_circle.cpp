#include "Copter.h"
#include <AP_Mount/AP_Mount.h>

// #if MODE_CIRCLE_ENABLED

/*
 * Init and run calls for circle flight mode
 */

// circle_init - initialise circle controller flight mode
bool ModeCircle::init(bool ignore_checks)
{
    
    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    return true;
//     speed_changing = false;

//     // set speed and acceleration limits
//     pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
//     pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
//     pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
//     pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

//     // initialise circle controller including setting the circle center based on vehicle speed
//     copter.circle_nav->init();

// #if HAL_MOUNT_ENABLED
//     // Check if the CIRCLE_OPTIONS parameter have roi_at_center
//     if (copter.circle_nav->roi_at_center()) {
//         const Vector3p &pos { copter.circle_nav->get_center() };
//         Location circle_center;
//         if (!AP::ahrs().get_location_from_origin_offset_NED(circle_center, pos * 0.01)) {
//             return false;
//         }
//         // point at the ground:
//         circle_center.set_alt_cm(0, Location::AltFrame::ABOVE_TERRAIN);
//         AP_Mount *s = AP_Mount::get_singleton();
//         s->set_roi_target(circle_center);
//     }
// #endif

//     // set auto yaw circle mode
//     auto_yaw.set_mode(AutoYaw::Mode::CIRCLE);

//     return true;
}

// circle_run - runs the circle flight mode
// should be called at 100hz or more
void ModeCircle::run()
{
        // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max_cd());

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate();

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

    // Alt Hold State Machine Determination
    AltHoldModeState althold_state = get_alt_hold_state(target_climb_rate);

    // Alt Hold State Machine
    switch (althold_state) {

    case AltHoldModeState::MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate(false);
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHoldModeState::Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHoldModeState::Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHoldModeState::Takeoff:
        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff(target_climb_rate);
        break;

    case AltHoldModeState::Flying:
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

#if AP_AVOIDANCE_ENABLED
        // apply avoidance
        copter.avoid.adjust_roll_pitch(target_roll, target_pitch, copter.aparm.angle_max);
#endif

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

#if AP_RANGEFINDER_ENABLED
        // update the vertical offset based on the surface measurement
        copter.surface_tracking.update_surface_offset();
#endif

        // Send the commanded climb rate to the position controller
        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
        break;
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // run the vertical position controller and set output throttle
    pos_control->update_z_controller_disturbance();
    // set speed and acceleration limits
//     pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
//     pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

//     // Check for any change in params and update in real time
//     copter.circle_nav->check_param_change();

//     // pilot changes to circle rate and radius
//     // skip if in radio failsafe
//     if (!copter.failsafe.radio && copter.circle_nav->pilot_control_enabled()) {
//         // update the circle controller's radius target based on pilot pitch stick inputs
//         const float radius_current = copter.circle_nav->get_radius();           // circle controller's radius target, which begins as the circle_radius parameter
//         const float pitch_stick = channel_pitch->norm_input_dz();               // pitch stick normalized -1 to 1
//         const float nav_speed = copter.wp_nav->get_default_speed_xy();          // copter WP_NAV parameter speed
//         const float radius_pilot_change = (pitch_stick * nav_speed) * G_Dt;     // rate of change (pitch stick up reduces the radius, as in moving forward)
//         const float radius_new = MAX(radius_current + radius_pilot_change,0);   // new radius target

//         if (!is_equal(radius_current, radius_new)) {
//             copter.circle_nav->set_radius_cm(radius_new);
//         }

//         // update the orbicular rate target based on pilot roll stick inputs
//         // skip if using transmitter based tuning knob for circle rate
//         if (g.radio_tuning != TUNING_CIRCLE_RATE) {
//             const float roll_stick = channel_roll->norm_input_dz();         // roll stick normalized -1 to 1

//             if (is_zero(roll_stick)) {
//                 // no speed change, so reset speed changing flag
//                 speed_changing = false;
//             } else {
//                 const float rate = copter.circle_nav->get_rate();           // circle controller's rate target, which begins as the circle_rate parameter
//                 const float rate_current = copter.circle_nav->get_rate_current(); // current adjusted rate target, which is probably different from _rate
//                 const float rate_pilot_change = (roll_stick * G_Dt);        // rate of change from 0 to 1 degrees per second
//                 float rate_new = rate_current;                              // new rate target
//                 if (is_positive(rate)) {
//                     // currently moving clockwise, constrain 0 to 90
//                     rate_new = constrain_float(rate_current + rate_pilot_change, 0, 90);

//                 } else if (is_negative(rate)) {
//                     // currently moving counterclockwise, constrain -90 to 0
//                     rate_new = constrain_float(rate_current + rate_pilot_change, -90, 0);

//                 } else if (is_zero(rate) && !speed_changing) {
//                     // Stopped, pilot has released the roll stick, and pilot now wants to begin moving with the roll stick
//                     rate_new = rate_pilot_change;
//                 }

//                 speed_changing = true;
//                 copter.circle_nav->set_rate(rate_new);
//             }
//         }
//     }

//     // get pilot desired climb rate (or zero if in radio failsafe)
//     float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());

//     // get avoidance adjusted climb rate
//     target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

//     // if not armed set throttle to zero and exit immediately
//     if (is_disarmed_or_landed()) {
//         make_safe_ground_handling();
//         return;
//     }

//     // set motors to full range
//     motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

// #if AP_RANGEFINDER_ENABLED
//     // update the vertical offset based on the surface measurement
//     copter.surface_tracking.update_surface_offset();
// #endif

//     copter.failsafe_terrain_set_status(copter.circle_nav->update(target_climb_rate));
//     pos_control->update_z_controller();

//     // call attitude controller with auto yaw
//     attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

uint32_t ModeCircle::wp_distance() const
{
    return copter.circle_nav->get_distance_to_target();
}

int32_t ModeCircle::wp_bearing() const
{
    return copter.circle_nav->get_bearing_to_target();
}

// #endif
