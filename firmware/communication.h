#ifndef __COMMUNICATION__H
#define __COMMUNICATION__H

#include "types.h"

// bc means from Buggy to Computer
// cb means from Computer to Buggy
enum {
	BC_TELEMETRY = 0x01,
	CB_MOTOR_COMMAND = 0x02
};

typedef struct
	__attribute__((packed)) {
		uint8_t header; // BC_TELEMETRY
		uint32_t time;
		fixed ir_left;
		fixed ir_right;
		fixed ir_front_left;
		fixed ir_front_right;
		fixed ir_front;
		point_t mc;
		fixed mc_dist;
		fixed mc_angle;

		fixed steering_pwm;
		fixed driving_pwm;
	} bc_telemetry_packet_t;

	typedef struct
		__attribute__((packed)) {
			uint8_t header; // CB_MOTOR_COMMAND
			uint8_t automatic;
			uint8_t steering_pwm;
			uint8_t drive_pwm;
		} cb_motor_command_packet_t;

#endif
