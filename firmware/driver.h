#ifndef __DRIVER__H
#define __DRIVER__H

#include "communication.h"

typedef struct drive_cmd_t {
	fixed steering_pwm;
	fixed driving_pwm;
} drive_cmd_t;

class Timeouter {
private:
	uint32_t trig_time;

public:
	Timeouter() :
			trig_time(0) {
	}

	void start(uint32_t time, uint32_t timeout) {
		trig_time = time + timeout;
	}

	void stop() {
		trig_time = 0;
	}

	bool running() {
		return trig_time > 0;
	}

	bool triggered(uint32_t time) {
		return time > trig_time;
	}
};

// Abstract base class
class Driver {
protected:
	drive_cmd_t drive_cmd;

public:
	virtual drive_cmd_t& drive(bc_telemetry_packet_t& telemetry) = 0;
};

#endif // __DRIVER__H
