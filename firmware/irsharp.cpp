#include "Arduino.h"
#include "IRSharp.h"

IRSharp::IRSharp(int irPin) : dist(40, 10, 5, 5), irPin(irPin)
{
  analogReference(DEFAULT);
}

// GP2Y0A21Y
int IRSharp::distance()
{
  return dist.stepKalman(irLookup[analogRead(_irPin)]);
}

