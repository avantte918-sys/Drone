#include "Copter.h"

#if MODE_GUIDED_NOGPS_ENABLED

// Local height-takeoff variant: MAV_CMD_USER_1 param1 is climb above start (m).
// This replaces the attitude-stream interface of mode 20.
bool ModeGuidedNoGPS::init(bool ignore_checks)
{
    // Enter on the ground only; an in-air re-entry would lose the height reference.
    if (motors->armed() && !copter.ap.land_complete) {
        return false;
    }
    _active = false;
    _reached = false;
    _airborne = false;
    _settled_ms = 0;
    takeoff.stop();
    pos_control->D_set_max_speed_accel_m(0.3f, 0.5f, 0.5f);
    pos_control->D_set_correction_speed_accel_m(0.3f, 0.5f, 0.5f);
    pos_control->D_init_controller();
    return true;
}

bool ModeGuidedNoGPS::height_feedback_ok() const
{
#if AP_RANGEFINDER_ENABLED
    nav_filter_status status {};
    return copter.rangefinder_state.enabled_and_healthy() &&
           !copter.rangefinder_state.data_stale() &&
           ahrs.get_filter_status(status) && status.flags.attitude &&
           status.flags.vert_pos && status.flags.vert_vel;
#else
    return false;
#endif
}

MAV_RESULT ModeGuidedNoGPS::start_height_takeoff(float height_m)
{
    if (!isfinite(height_m) || height_m < 0.2f || height_m > 2.0f) {
        return MAV_RESULT_DENIED;
    }
    if (!motors->armed() || !height_feedback_ok()) {
        return MAV_RESULT_TEMPORARILY_REJECTED;
    }
    // ACK retries must not move the target or restart the takeoff.
    if (_active) {
        return is_equal(height_m, _height_m) ? MAV_RESULT_ACCEPTED : MAV_RESULT_DENIED;
    }
    if (!copter.ap.land_complete || (copter.ap.using_interlock && !motors->get_interlock())) {
        return MAV_RESULT_DENIED;
    }
#if AP_RANGEFINDER_ENABLED
    if (copter.rangefinder_state.alt_m + height_m + 0.2f >
        copter.rangefinder.max_distance_orient(ROTATION_PITCH_270)) {
        return MAV_RESULT_DENIED;
    }
#endif
    if (fabsf(ahrs.get_roll_rad()) > radians(15.0f) ||
        fabsf(ahrs.get_pitch_rad()) > radians(15.0f)) {
        return MAV_RESULT_DENIED;
    }
    _height_m = height_m;
    _target_alt_m = pos_control->get_pos_estimate_U_m() + height_m;
    _attitude.from_euler(Vector3f{0.0f, 0.0f, ahrs.get_yaw_rad()});
    float reset_delta;
    _reset_ms = ahrs.getLastPosDownReset(reset_delta);
    _started_ms = AP_HAL::millis();
    _settled_ms = 0;
    _reached = false;
    _airborne = false;
    _active = true;
    copter.set_auto_armed(true);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NGH: climb %.2fm accepted", double(height_m));
    return MAV_RESULT_ACCEPTED;
}

void ModeGuidedNoGPS::exit()
{
    takeoff.stop();
    _active = false;
    _reached = false;
    _airborne = false;
}

void ModeGuidedNoGPS::run()
{
    if (!motors->armed()) {
        _active = false;
        _reached = false;
        takeoff.stop();
    }
    if (!_active) {
        make_safe_ground_handling(false);
        return;
    }

    float reset_delta;
    const bool reset = ahrs.getLastPosDownReset(reset_delta) != _reset_ms;
    const bool timeout = !_reached && AP_HAL::millis() - _started_ms > 20000U;
    if (!height_feedback_ok() || reset || timeout || (_airborne && copter.ap.land_complete)) {
        // Do not continue climbing with invalid feedback or a changed reference.
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "NGH: abort, feedback/reset/timeout");
        copter.set_mode_land_with_pause(ModeReason::EKF_FAILSAFE);
        return;
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    if (motors->get_spool_state() != AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
        attitude_control->reset_rate_controller_I_terms();
        pos_control->D_relax_controller(0.0f);
        pos_control->D_update_controller();
        attitude_control->input_quaternion(_attitude, Vector3f{});
        return;
    }

    if (copter.ap.land_complete && !takeoff.running()) {
        // Reuse AltHold's takeoff detection and throttle slew, without pilot input.
        takeoff.start_m(MAX(0.0f, _target_alt_m - pos_control->get_pos_estimate_U_m()));
    }
    if (takeoff.running()) {
        takeoff.do_pilot_takeoff_ms(0.0f);
    } else {
        pos_control->D_set_alt_target_with_slew_m(_target_alt_m);
    }
    attitude_control->input_quaternion(_attitude, Vector3f{});
    pos_control->D_update_controller();

    _airborne |= !copter.ap.land_complete;
    const bool settled = !copter.ap.land_complete && !takeoff.running() &&
        fabsf(pos_control->get_pos_estimate_U_m() - _target_alt_m) < 0.10f &&
        fabsf(pos_control->get_vel_estimate_U_ms()) < 0.15f;
    if (!settled) {
        _settled_ms = 0;
    } else if (_settled_ms == 0) {
        _settled_ms = AP_HAL::millis();
    } else if (!_reached && AP_HAL::millis() - _settled_ms >= 1000U) {
        _reached = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NGH: target reached, holding");
    }
}

#endif
