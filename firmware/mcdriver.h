#ifndef __MCDRIVER__H
#define __MCDRIVER__H

#include "driver.h"

typedef enum mc_driver_states_t {
	STATE_IDLE = 1, 
        STATE_NORMAL, 
        STATE_START_BACKING_1, 
        STATE_START_BACKING_2, 
        STATE_BACKING, 
        STATE_BRAKING
} mc_driver_states_t;

// Drive towards mass center of estimated IR reflection points
class MCDriver: public Driver {
protected:
	mc_driver_states_t state;
	Timeouter stuck_timer, last_speed_add_timer;
	bool maybe_stuck;
        point_t l, f, r;

	fixed min_front;
        fixed last_speed_add;

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
	MCDriver(int min_steering, int neutral_steering, int max_steering, int range_steering_deg, int neutral_driving, int max_driving, int norm_driving_f, int norm_driving_b, int min_driving_b);
	drive_cmd_t& drive(bc_telemetry_packet_t& telemetry);
};

#endif // __MCDRIVER__H


