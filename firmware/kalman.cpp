// Idea from: http://pengu.student.utwente.nl/wordpress/?p=116

#include "kalman.h"

static fixed VAL_1(1);

// Single variable Kalman filter
SingleKalmanVar::SingleKalmanVar(const fixed& x, const fixed& P, const fixed& Q, const fixed& R) {
	this->x = x; // variable estimate
	this->P = P; // error (variance) estimate
	this->Q = Q; // process variance
	this->R = R; // measurement variance
}

fixed& SingleKalmanVar::stepKalman(fixed measurement) {
	// predict
	// predicted x is last x
	P_temp = P + R;

	// update
	K = P_temp / (P_temp + Q);
	x += K * (measurement - x);
	P = (VAL_1 - K) * P_temp;

	return x;
}

