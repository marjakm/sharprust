#ifndef __TYPES__H
#define __TYPES__H

#include <stdint.h>
#include <math.h>
#include "fixed.h"

const fixed VAL_RAD_TO_DEG(57.295779513082320876798154814105);

class __attribute__((packed)) point_t {
public:
	fixed x, y;

	point_t() :
			x((int16_t)0), y((int16_t)0) {
	}

	point_t(const fixed& x, const fixed& y) :
			x(x), y(y) {
	}

	fixed get_distance() {
		// TODO: get rid of double and sqrt
		fixed dist(sqrt(double(x * x + y * y)));
		return dist;
	}

	fixed get_deg_angle() {
		// TODO: get rid of double and atan2
		fixed rads(atan2(double(y), double(x)));
		fixed angle(rads * VAL_RAD_TO_DEG);
		return angle;
	}
};

#define ABS(x) (x >= 0 ? x : -x)

#endif // __TYPES__H


