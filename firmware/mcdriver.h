#ifndef __MCDRIVER__H
#define __MCDRIVER__H

#include "fixed.h"
#include "types.h"
#include "communication.h"
#include "HardwareSerial.h"

typedef struct drive_cmd_t {
	fixed steering_pwm;
	fixed driving_pwm;
} drive_cmd_t;

typedef enum mc_driver_states_t {
	STATE_IDLE = 1, 
        STATE_NORMAL, 
        STATE_START_BACKING_1, 
        STATE_START_BACKING_2, 
        STATE_BACKING, 
        STATE_BRAKING
} mc_driver_states_t;

// Drive towards mass center of estimated IR reflection points
class MCDriver {
protected:
	mc_driver_states_t state;
        drive_cmd_t drive_cmd;
        fixed ticks_per_second;
        HardwareSerial* ser;
        
        bool maybe_stuck;
        fixed not_stuck_counter;
        
        fixed maybe_stuck_tick_nr;
        fixed backing_start_tick_nr;
        fixed last_speed_add;
        
        int speed_diff;
        int constant_speed_counter;
	
        point_t l, f, r;
	int min_front;

        fixed driving_max;
        fixed driving_norm_f;
        fixed driving_neutral;
        fixed driving_norm_b;
        fixed driving_min_b;
        
        fixed steering;
        fixed steering_neutral;
        fixed steering_deg_to_pwm;
        
	void  _calc_direction(bc_telemetry_packet_t& telemetry);
        fixed _calc_steering_pwm(fixed steering_direction_deg);

public:
	MCDriver(HardwareSerial &ser, fixed ticks_to_second, int min_steering, int neutral_steering, int max_steering, int range_steering_deg, int neutral_driving, int max_driving, int norm_driving_f, int norm_driving_b, int min_driving_b);
	drive_cmd_t& drive(bc_telemetry_packet_t& telemetry, int tick_nr);
};

#endif // __MCDRIVER__H


