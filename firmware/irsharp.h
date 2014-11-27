#ifndef __IRSHARP__H
#define __IRSHARP__H

#include "kalman.h"

class IRSharp {
public:
	IRSharp(int16_t irPin);
	fixed& distance();

private:
	int16_t irPin;
	SingleKalmanVar dist;
};

#endif // __IRSHARP__H


