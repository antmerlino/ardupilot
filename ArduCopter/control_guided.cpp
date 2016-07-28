/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

/*
 * control_guided.pde - init and run calls for guided flight mode
 */

#ifndef GUIDED_LOOK_AT_TARGET_MIN_DISTANCE_CM
 # define GUIDED_LOOK_AT_TARGET_MIN_DISTANCE_CM     500     // point nose at target if it is more than 5m away
#endif

#define GUIDED_POSVEL_TIMEOUT_MS    3000    // guided mode's position-velocity controller times out after 3seconds with no new updates
#define GUIDED_SPLINE_MISSION_SIZE 	50		// guided spline's cmd size	
#define GUIDED_SPLINE_TIMEOUT_MS	10000	// guided spline's timeout period. If reached, it will reset the spline
#define GUIDED_ATTITUDE_TIMEOUT_MS  1000    // guided mode's attitude controller times out after 1 second with no new updates


static Vector3f posvel_pos_target_cm;
static Vector3f posvel_vel_target_cms;
static uint32_t posvel_update_time_ms;
static uint32_t vel_update_time_ms;

struct Guided_Limit {
    uint32_t timeout_ms;  // timeout (in seconds) from the time that guided is invoked
    float alt_min_cm;   // lower altitude limit in cm above home (0 = no limit)
    float alt_max_cm;   // upper altitude limit in cm above home (0 = no limit)
    float horiz_max_cm; // horizontal position limit in cm from where guided mode was initiated (0 = no limit)
    uint32_t start_time;// system time in milliseconds that control was handed to the external computer
    Vector3f start_pos; // start position as a distance from home in cm.  used for checking horiz_max limit
} guided_limit;

struct Guided_Spline_Manager {
	uint32_t update_time_ms;  // timeout (in seconds) from the time that guided is invoked
	uint16_t current_cmd_id; // Current Command Index
	uint16_t cmds_length; // Length of valid cmds	
	uint16_t reset; // Reset
	uint16_t started;
	uint16_t cmds_last; // Index of last added waypoint
	AP_Mission::Mission_Command cmds[GUIDED_SPLINE_MISSION_SIZE];
} guided_spline_manager;
static bool guided_spline_manager_initialized = false;

struct {
    uint32_t update_time_ms;
    float roll_cd;
    float pitch_cd;
    float yaw_cd;
    float climb_rate_cms;
} static guided_angle_state = {0,0.0f, 0.0f, 0.0f, 0.0f};

// guided_init - initialise guided controller
bool Copter::guided_init(bool ignore_checks)
{
#if FRAME_CONFIG == HELI_FRAME
    // do not allow helis to enter Alt Hold if the Rotor Runup is not complete and current control mode has manual throttle control,
    // as this will force the helicopter to descend.
    if (!ignore_checks && mode_has_manual_throttle(control_mode) && !motors.rotor_runup_complete()){
        return false;
    }
#endif

    if (position_ok() || ignore_checks) {
        // initialise yaw
        set_auto_yaw_mode(get_default_auto_yaw_mode(false));
        // start in position control mode
        guided_pos_control_start();
        return true;
    }else{
        return false;
    }
}


// guided_takeoff_start - initialises waypoint controller to implement take-off
void Copter::guided_takeoff_start(float final_alt_above_home)
{
    guided_mode = Guided_TakeOff;
    
    // initialise wpnav destination
    Vector3f target_pos = inertial_nav.get_position();
    target_pos.z = pv_alt_above_origin(final_alt_above_home);
    wp_nav.set_wp_destination(target_pos);

    // initialise yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);

    // clear i term when we're taking off
    set_throttle_takeoff();
}

// initialise guided mode's position controller
void Copter::guided_pos_control_start()
{
    // set to position control mode
    guided_mode = Guided_WP;

    // initialise waypoint and spline controller
    wp_nav.wp_and_spline_init();

    // initialise wpnav to stopping point at current altitude
    // To-Do: set to current location if disarmed?
    // To-Do: set to stopping point altitude?
    Vector3f stopping_point;
    stopping_point.z = inertial_nav.get_altitude();
    wp_nav.get_wp_stopping_point_xy(stopping_point);
    wp_nav.set_wp_destination(stopping_point);

    // initialise yaw
    set_auto_yaw_mode(get_default_auto_yaw_mode(false));
}

// initialise guided mode's velocity controller
void Copter::guided_vel_control_start()
{
    // set guided_mode to velocity controller
    guided_mode = Guided_Velocity;

    // initialize vertical speeds and leash lengths
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // initialise velocity controller
    pos_control.init_vel_controller_xyz();
}

// initialise guided mode's posvel controller
void Copter::guided_posvel_control_start()
{
    // set guided_mode to velocity controller
    guided_mode = Guided_PosVel;

    pos_control.init_xy_controller();

    // set speed and acceleration from wpnav's speed and acceleration
    pos_control.set_speed_xy(wp_nav.get_speed_xy());
    pos_control.set_accel_xy(wp_nav.get_wp_acceleration());

    const Vector3f& curr_pos = inertial_nav.get_position();
    const Vector3f& curr_vel = inertial_nav.get_velocity();

    // set target position and velocity to current position and velocity
    pos_control.set_xy_target(curr_pos.x, curr_pos.y);
    pos_control.set_desired_velocity_xy(curr_vel.x, curr_vel.y);

    // set vertical speed and acceleration
    pos_control.set_speed_z(wp_nav.get_speed_down(), wp_nav.get_speed_up());
    pos_control.set_accel_z(wp_nav.get_accel_z());

    // pilot always controls yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);
}

// Guided spline code
// Verge Aero

// initialize guide mode's spline start
void Copter::guided_spline_start(const Vector3f& destination, bool stopped_at_start, 
                               AC_WPNav::spline_segment_end_type seg_end_type, 
                               const Vector3f& next_destination)
{
    guided_mode = Guided_WP_Spline;
	
	// initialise waypoint and spline controller
    wp_nav.wp_and_spline_init();
	
	// Check if timeout has been reached
	if (millis() - guided_spline_manager.update_time_ms > GUIDED_SPLINE_TIMEOUT_MS) {
		reset_guided_spline_manager();
	}
	
	// Set Manager 
	guided_spline_manager.update_time_ms = millis();
	
    // initialise wpnav
    wp_nav.set_spline_destination(destination, stopped_at_start, seg_end_type, next_destination);
	
    // initialise yaw
    // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
    if (auto_yaw_mode != AUTO_YAW_ROI) {
        set_auto_yaw_mode(get_default_auto_yaw_mode(false));
    }
}

// End Guided spline code

// guided_set_destination - sets guided mode's target destination
void Copter::guided_set_destination(const Vector3f& destination)
{
    // ensure we are in position control mode
    if (guided_mode != Guided_WP) {
        guided_pos_control_start();
    }

    wp_nav.set_wp_destination(destination);
}

// Verge Aero
// guided_set_destination - sets guided mode's target destination
void Copter::guided_set_destination(const Vector3f& destination, bool fast)
{
    // ensure we are in position control mode
    if (guided_mode != Guided_WP) {
        guided_pos_control_start();
    }

    wp_nav.set_wp_destination(destination, fast);
}

// guided_set_velocity - sets guided mode's target velocity
void Copter::guided_set_velocity(const Vector3f& velocity)
{
    // check we are in velocity control mode
    if (guided_mode != Guided_Velocity) {
        guided_vel_control_start();
    }

    vel_update_time_ms = millis();

    // set position controller velocity target
    pos_control.set_desired_velocity(velocity);
}

// set guided mode posvel target
void Copter::guided_set_destination_posvel(const Vector3f& destination, const Vector3f& velocity) {
    // check we are in velocity control mode
    if (guided_mode != Guided_PosVel) {
        guided_posvel_control_start();
    }

    posvel_update_time_ms = millis();
    posvel_pos_target_cm = destination;
    posvel_vel_target_cms = velocity;

    pos_control.set_pos_target(posvel_pos_target_cm);
}

// guided_run - runs the guided controller
// should be called at 100hz or more
void Copter::guided_run()
{
    // call the correct auto controller
    switch (guided_mode) {

    case Guided_TakeOff:
        // run takeoff controller
        guided_takeoff_run();
        break;

    case Guided_WP:
        // run position controller
        guided_pos_control_run();
        break;

    case Guided_Velocity:
        // run velocity controller
        guided_vel_control_run();
        break;

    case Guided_PosVel:
        // run position-velocity controller
        guided_posvel_control_run();
        break;
		
	case Guided_WP_Spline:
		// run spline controller
		guided_spline_run();
		break;
    }
 }

// guided_takeoff_run - takeoff in guided mode
//      called by guided_run at 100hz or more
void Copter::guided_takeoff_run()
{
    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (!ap.auto_armed || !motors.get_interlock()) {
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(0, 0, 0, get_smoothing_gain());
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
#else   // multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);
    }

    // run waypoint controller
    wp_nav.update_wpnav();

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control.update_z_controller();

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
}

// guided_pos_control_run - runs the guided position controller
// called from guided_run
void Copter::guided_pos_control_run()
{
    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (!ap.auto_armed || !motors.get_interlock() || ap.land_complete) {
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(0, 0, 0, get_smoothing_gain());
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
#else   // multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // run waypoint controller
    wp_nav.update_wpnav();

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control.update_z_controller();

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control.angle_ef_roll_pitch_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), get_auto_heading(), true);
    }
}

// guided_vel_control_run - runs the guided velocity controller
// called from guided_run
void Copter::guided_vel_control_run()
{
    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (!ap.auto_armed || !motors.get_interlock() || ap.land_complete) {
        // initialise velocity controller
        pos_control.init_vel_controller_xyz();
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(0, 0, 0, get_smoothing_gain());
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
#else   // multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // set velocity to zero if no updates received for 3 seconds
    uint32_t tnow = millis();
    if (tnow - vel_update_time_ms > GUIDED_POSVEL_TIMEOUT_MS && !pos_control.get_desired_velocity().is_zero()) {
        pos_control.set_desired_velocity(Vector3f(0,0,0));
    }

    // call velocity controller which includes z axis controller
    pos_control.update_vel_controller_xyz(ekfNavVelGainScaler);

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(pos_control.get_roll(), pos_control.get_pitch(), target_yaw_rate);
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control.angle_ef_roll_pitch_yaw(pos_control.get_roll(), pos_control.get_pitch(), get_auto_heading(), true);
    }
}

// Guided spline code
// Verge Aero

// guided_posvel_control_run - runs the guided spline controller
// called from guided_run
void Copter::guided_posvel_control_run()
{
    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (!ap.auto_armed || !motors.get_interlock() || ap.land_complete) {
        // set target position and velocity to current position and velocity
        pos_control.set_pos_target(inertial_nav.get_position());
        pos_control.set_desired_velocity(Vector3f(0,0,0));
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(0, 0, 0, get_smoothing_gain());
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
#else   // multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;

    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // set velocity to zero if no updates received for 3 seconds
    uint32_t tnow = millis();
    if (tnow - posvel_update_time_ms > GUIDED_POSVEL_TIMEOUT_MS && !posvel_vel_target_cms.is_zero()) {
        posvel_vel_target_cms.zero();
    }

    // calculate dt
    float dt = pos_control.time_since_last_xy_update();

    // update at poscontrol update rate
    if (dt >= pos_control.get_dt_xy()) {
        // sanity check dt
        if (dt >= 0.2f) {
            dt = 0.0f;
        }

        // advance position target using velocity target
        posvel_pos_target_cm += posvel_vel_target_cms * dt;

        // send position and velocity targets to position controller
        pos_control.set_pos_target(posvel_pos_target_cm);
        pos_control.set_desired_velocity_xy(posvel_vel_target_cms.x, posvel_vel_target_cms.y);

        // run position controller
        pos_control.update_xy_controller(AC_PosControl::XY_MODE_POS_AND_VEL_FF, ekfNavVelGainScaler);
    }

    pos_control.update_z_controller();

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(pos_control.get_roll(), pos_control.get_pitch(), target_yaw_rate);
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control.angle_ef_roll_pitch_yaw(pos_control.get_roll(), pos_control.get_pitch(), get_auto_heading(), true);
    }
}

// Guided spline code
// Verge Aero

void Copter::guided_spline_run()
{
    // process pilot's yaw input
    float target_yaw_rate = 0;
	
    if (!failsafe.radio) {
        // get pilot's desired yaw rat
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }
	
	// Check if timeout has been reached
	if ((millis() - guided_spline_manager.update_time_ms > GUIDED_SPLINE_TIMEOUT_MS) && guided_spline_manager.reset==0) {
		reset_guided_spline_manager();
	}
	
	// Check if the cmd ID should loop
	if( guided_spline_manager.reset==0 && guided_spline_manager.started == 1
	        && guided_spline_manager.current_cmd_id==GUIDED_SPLINE_MISSION_SIZE
	        && guided_spline_manager.current_cmd_id!=guided_spline_manager.cmds_last){
		guided_spline_manager.current_cmd_id = 0;
	}
	
	// Updated Guided Spline
	if (wp_nav.reached_spline_destination()){
		// If reset is not equal to 0 the mission has started
		if (guided_spline_manager.reset==0 && guided_spline_manager.started == 1
		        && guided_spline_manager.current_cmd_id==guided_spline_manager.cmds_last) {

		    // Set Current Waypoint
            if(guided_spline_manager.cmds[guided_spline_manager.current_cmd_id-1].content.location.flags.relative_xy){
                guided_set_destination(Vector3f(1.0e-1f*guided_spline_manager.cmds[guided_spline_manager.current_cmd_id-1].content.location.x,
                        1.0e-1f*guided_spline_manager.cmds[guided_spline_manager.current_cmd_id-1].content.location.y,
                        pv_alt_above_origin(guided_spline_manager.cmds[guided_spline_manager.current_cmd_id-1].content.location.alt)));
            } else {
                // set wp_nav's destination
                Vector3f pos_or_vel = pv_location_to_vector(guided_spline_manager.cmds[guided_spline_manager.current_cmd_id-1].content.location);

                // Set Destination
                guided_set_destination(pos_or_vel);
            }

			// Reset the manager
			reset_guided_spline_manager();	
			
			gcs_send_text_P(SEVERITY_MEDIUM,PSTR("Spline Finished"));
		}		
		// If current_cmd_id < cmds_length then we have valid GPS positions to execute
		else if (guided_spline_manager.reset==0 && guided_spline_manager.started == 1){
			// Start the stored guided request
			do_guided(guided_spline_manager.cmds[guided_spline_manager.current_cmd_id++]);
		} 
	}

    // run waypoint controller
    wp_nav.update_spline();

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control.update_z_controller();

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control.angle_ef_roll_pitch_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), get_auto_heading(), true);
    }
}

// Start the mission list
// Verge Aero

bool Copter::start_guided_spline_mission(){
	// Make sure the mission has been reset
	if(!guided_spline_manager_initialized){
		reset_guided_spline_manager();		
		guided_spline_manager_initialized = true;
		return false;
	}	
	
	// Make sure there are enough waypoints to start
	if(guided_spline_manager.cmds_length<1){
		return false;
	}
	
	// Make sure there are enough waypoints to start
	if(guided_spline_manager.cmds_length==2 && guided_spline_manager.current_cmd_id<guided_spline_manager.cmds_length){
		// Make sure we send both the current and next waypoint
		do_guided(guided_spline_manager.cmds[guided_spline_manager.current_cmd_id++]);
		return true;
	}
	
	// Make sure that we haven't started already
	if(guided_spline_manager.started==1){		
		return true;
	}	
	
	// Initialize the wp and spline in wp nav
	wp_nav.wp_and_spline_init();
	
	// Set Update Time 
	guided_spline_manager.update_time_ms = millis();
	
	// Spline manager knows that the system is no longer reset
	guided_spline_manager.reset = 0;
	guided_spline_manager.started = 1;
	
	// Make sure we send both the current and next waypoint
	do_guided(guided_spline_manager.cmds[guided_spline_manager.current_cmd_id++]);
	return true;
}

// Add to mission list
// Verge Aero

void Copter::add_guided_spline_cmd(AP_Mission::Mission_Command cmd){
	
	// EXPERIMENTAL: Make sure we are not adding duplicate waypoints
	if (cmd.content.location.lat == guided_spline_manager.cmds[guided_spline_manager.cmds_last].content.location.lat && cmd.content.location.lng == guided_spline_manager.cmds[guided_spline_manager.cmds_last].content.location.lng){
		// Duplicate Waypoint	
		return;
	}
	
	// Check the size
	if (guided_spline_manager.cmds_last>=GUIDED_SPLINE_MISSION_SIZE && guided_spline_manager.cmds_length>=GUIDED_SPLINE_MISSION_SIZE){
		// Mission List is filled
		// Point to beginning
		guided_spline_manager.cmds_last = 0;
	} else if (guided_spline_manager.cmds_length<GUIDED_SPLINE_MISSION_SIZE){
		// Increment the Length
		guided_spline_manager.cmds_length++;
	}
	
	// Set Update Time 
	guided_spline_manager.update_time_ms = millis();
	
	// Reset the p1 value and reset
	cmd.p1 = MODIFY_GUIDED_SPLINE_WAYPOINT_WAYPOINT_PASS;
	
	// Add the mission to the mission list
	guided_spline_manager.cmds[guided_spline_manager.cmds_last++] = cmd;	
}

// Modify the mission list at a mission position
// Verge Aero

void Copter::modify_guided_spline_cmd(uint16_t mission_position, AP_Mission::Mission_Command cmd){
	// Check to make sure that the mission position is valid
	if (mission_position >= guided_spline_manager.cmds_length){
		return;
	}
	
	// Change the parameter value to an arbitrarily large number to tell the do_guided command
	// to GOTO spline_waypoint
	cmd.p1 = MODIFY_GUIDED_SPLINE_WAYPOINT_WAYPOINT_PASS;
	
	// Overwrite the CMD at the position specified
	guided_spline_manager.cmds[mission_position] = cmd;
}

// Delete a mission item at a mission position
// Verge Aero

void Copter::delete_guided_spline_cmd(uint16_t mission_position){
	// Variables
	uint16_t i = 0;
	AP_Mission::Mission_Command cmd;
	
	// Check to make sure that the mission position is valid
	if (mission_position >= guided_spline_manager.cmds_length){
		return;
	}
	
	// Mission position is valid
	
	// Shift all stored commands
	for (i = mission_position; i<guided_spline_manager.cmds_length-1; i++){
		guided_spline_manager.cmds[i] = guided_spline_manager.cmds[i+1];
	}
	
	// Clear the end of the cmds list
	for (i=guided_spline_manager.cmds_length-1; i<GUIDED_SPLINE_MISSION_SIZE;i++){
		guided_spline_manager.cmds[i] = cmd;
	}
	
	// Decrement the length
	guided_spline_manager.cmds_length--;
	
	//gcs_send_text_P(SEVERITY_MEDIUM,PSTR("Waypoint Deleted\n"));
}

// Insert a mission item at a mission position
// Verge Aero

void Copter::insert_guided_spline_cmd(uint16_t mission_position, AP_Mission::Mission_Command cmd){
	// Variables
	uint16_t i = 0;
	
	// Check to make sure that the mission position is valid
	if (mission_position >= guided_spline_manager.cmds_length){
		//gcs_send_text_P(SEVERITY_MEDIUM,PSTR("Mission position out of bounds\n"));
		return;
	}	
	
	// Check if we have room to add a mission position
	if (guided_spline_manager.cmds_length+1>GUIDED_SPLINE_MISSION_SIZE){
		// Mission List is filled
		return;
	}	
	
	// Shift all stored commands
	for (i = mission_position; i<=guided_spline_manager.cmds_length; i++){
		guided_spline_manager.cmds[i+1] = guided_spline_manager.cmds[i];
	}
	
	// Change the parameter value to an arbitrarily large number to tell the do_guided command
	// to GOTO spline_waypoint
	cmd.p1 = MODIFY_GUIDED_SPLINE_WAYPOINT_WAYPOINT_PASS;
	
	// Overwrite the CMD at the position specified
	guided_spline_manager.cmds[mission_position] = cmd;
	
	// Increment the length
	guided_spline_manager.cmds_length++;
}

// Reset the mission List
// Verge Aero

bool Copter::reset_guided_spline_manager(){
	int i = 0;
	AP_Mission::Mission_Command cmd;
	
	// Return if it has already been reset
	if (guided_spline_manager.reset==1){
		return false;
	}
		
	//print_spline_manager();
	
	// Reset Variables
	guided_spline_manager.update_time_ms 	= millis();
	guided_spline_manager.current_cmd_id 	= 0;
	guided_spline_manager.cmds_length 		= 0;
	guided_spline_manager.reset 			= 1;
	guided_spline_manager.cmds_last 		= 0;
	guided_spline_manager.started 			= 0;
	
	// Clear the cmds list
	for (i=0; i<GUIDED_SPLINE_MISSION_SIZE;i++){
		guided_spline_manager.cmds[i] = cmd;
	}
	
	return true;
}

// Returns the number of waypoint remaining to be executed
// Verge Aero

int Copter::get_number_waypoints_remaining(){
	int difference = 0;
	
	// Calculate the difference
	difference = guided_spline_manager.cmds_last - guided_spline_manager.current_cmd_id;
	
	// If the circular buffer has looped, make sure we grab a correct value
	if (difference<0){
		difference = guided_spline_manager.cmds_last + (GUIDED_SPLINE_MISSION_SIZE - guided_spline_manager.current_cmd_id);
	}
	
	return difference;
}

void Copter::print_spline_manager(){	
	gcs_send_text_fmt(PSTR("Update Time 	: %i\n"),guided_spline_manager.update_time_ms);
	gcs_send_text_fmt(PSTR("Current CMD ID 	: %i\n"),guided_spline_manager.current_cmd_id);
	gcs_send_text_fmt(PSTR("CMD Length 		: %i\n"),guided_spline_manager.cmds_length);
	gcs_send_text_fmt(PSTR("Rest Bool 		: %i\n"),guided_spline_manager.reset);
	gcs_send_text_fmt(PSTR("Started Bool 	: %i\n"),guided_spline_manager.started);
	gcs_send_text_fmt(PSTR("CMD Last 		: %i\n"),guided_spline_manager.cmds_last);
}

// End Guided Spline Code

// Guided Limit code

// guided_limit_clear - clear/turn off guided limits
void Copter::guided_limit_clear()
{
    guided_limit.timeout_ms = 0;
    guided_limit.alt_min_cm = 0.0f;
    guided_limit.alt_max_cm = 0.0f;
    guided_limit.horiz_max_cm = 0.0f;
}

// guided_limit_set - set guided timeout and movement limits
void Copter::guided_limit_set(uint32_t timeout_ms, float alt_min_cm, float alt_max_cm, float horiz_max_cm)
{
    guided_limit.timeout_ms = timeout_ms;
    guided_limit.alt_min_cm = alt_min_cm;
    guided_limit.alt_max_cm = alt_max_cm;
    guided_limit.horiz_max_cm = horiz_max_cm;
}

// guided_limit_init_time_and_pos - initialise guided start time and position as reference for limit checking
//  only called from AUTO mode's auto_nav_guided_start function
void Copter::guided_limit_init_time_and_pos()
{
    // initialise start time
    guided_limit.start_time = hal.scheduler->millis();

    // initialise start position from current position
    guided_limit.start_pos = inertial_nav.get_position();
}

// guided_limit_check - returns true if guided mode has breached a limit
//  used when guided is invoked from the NAV_GUIDED_ENABLE mission command
bool Copter::guided_limit_check()
{
    // check if we have passed the timeout
    if ((guided_limit.timeout_ms > 0) && (millis() - guided_limit.start_time >= guided_limit.timeout_ms)) {
        return true;
    }

    // get current location
    const Vector3f& curr_pos = inertial_nav.get_position();

    // check if we have gone below min alt
    if (!is_zero(guided_limit.alt_min_cm) && (curr_pos.z < guided_limit.alt_min_cm)) {
        return true;
    }

    // check if we have gone above max alt
    if (!is_zero(guided_limit.alt_max_cm) && (curr_pos.z > guided_limit.alt_max_cm)) {
        return true;
    }

    // check if we have gone beyond horizontal limit
    if (guided_limit.horiz_max_cm > 0.0f) {
        float horiz_move = pv_get_horizontal_distance_cm(guided_limit.start_pos, curr_pos);
        if (horiz_move > guided_limit.horiz_max_cm) {
            return true;
        }
    }

    // if we got this far we must be within limits
    return false;
}
