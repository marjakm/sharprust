#ifndef __DRIVER__H
#define __DRIVER__H

#include "communication.h"

typedef struct drive_cmd_t {
	fixed steering_pwm;
	fixed driving_pwm;
} drive_cmd_t;

// Abstract base class
class Driver {
protected:
	drive_cmd_t drive_cmd;

public:
	virtual drive_cmd_t& drive(bc_telemetry_packet_t& telemetry, int tick_nr) = 0;
};

#endif // __DRIVER__H
