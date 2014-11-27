#include "irsharp.h"

#include "Arduino.h"
#include "lookups.h"

IRSharp::IRSharp(int16_t irPin) :
irPin(irPin), dist(fixed(40), fixed(10), fixed(1), fixed(5)) {
	pinMode(irPin, INPUT);
}

// GP2Y0A21Y
fixed& IRSharp::distance() {
	return dist.stepKalman(irLookup[analogRead(irPin)]);
}


