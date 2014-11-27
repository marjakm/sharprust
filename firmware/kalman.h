// Idea from: http://pengu.student.utwente.nl/wordpress/?p=116

#ifndef __KALMAN__H
#define __KALMAN__H

#include "types.h"

class SingleKalmanVar {
private:
	fixed Q, R, P, x;
	fixed P_temp, K;

public:
	SingleKalmanVar(const fixed& x, const fixed& P, const fixed& Q, const fixed& R);
	fixed& stepKalman(fixed measurement);
};

#endif // __KALMAN__H


