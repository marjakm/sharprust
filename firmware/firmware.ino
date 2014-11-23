#include "lookups.h"
#include "fixed.h"

#define SWEEP_STEP 100
#define SWEEP_PULSE_REPETITON_COUNT 10

#define SERVO_MIN_PULSE 1000
#define SERVO_MAX_PULSE 2000
#define SERVO_PWM_PIN 12

#define SENSOR_SWITCH_0 9
#define SENSOR_SWITCH_1 10

// Sharp measurement takes 38.3ms+-9.6ms
// Divide it by two and convert to us
#define STEP_DELAY 19150

uint8_t sensors[2][3] = {
  {A0, A2, A4}, // Group 0: left, center, right
  {A1, A3, A5}  // Group 1: left, center, right
};

IntervalTimer step_timer;
volatile int step_counter = 0;
volatile int started = 0;

fixed ir_left;
fixed ir_center = 40;
fixed ir_right;
volatile int sensor_group_in_use;


void setup(void) {
  for (int i=0; i<2; i++) {
    for (int j=0; j<3; j++){
      pinMode(sensors[i][j], INPUT);
    }
  }
  pinMode(SERVO_PWM_PIN,   OUTPUT);
  pinMode(SENSOR_SWITCH_0, OUTPUT);
  pinMode(SENSOR_SWITCH_1, OUTPUT);
  pinMode(LED_BUILTIN,     OUTPUT);
  analogReference(DEFAULT);
  analogReadAveraging(16);
  analogReadResolution(10);
  step_timer.begin(step_main, STEP_DELAY);
}

void step_main(void) {
  step_counter++;
  if (started == 0) {
     turn_on_sensors();
  } else {
    heartbeat();
    sensor_group_in_use = step_counter % 2;   
    ir_left   = irLookup[analogRead(sensors[sensor_group_in_use][0])];
    ir_center = irLookup[analogRead(sensors[sensor_group_in_use][1])];
    ir_right  = irLookup[analogRead(sensors[sensor_group_in_use][2])];
  }
}

void heartbeat(void) {
  if (step_counter % 25 == 0) {
    if (step_counter % 50 == 0) {
      digitalWrite(LED_BUILTIN, LOW);
    } else {
      digitalWrite(LED_BUILTIN, HIGH);
    }
  }
}

void turn_on_sensors(void) {
  if (step_counter == 1) {
    digitalWrite(SENSOR_SWITCH_0, HIGH);
  } else {
    digitalWrite(SENSOR_SWITCH_1, HIGH);
    started = 1;
  }
}

volatile int pulse_duration   = SERVO_MIN_PULSE;
void sweep_servo(void) {
    if (step_counter % SWEEP_PULSE_REPETITON_COUNT == 0) { 
      pulse_duration += SWEEP_STEP;
    }
    if (pulse_duration > SERVO_MAX_PULSE or pulse_duration < SERVO_MIN_PULSE) {
      pulse_duration = SERVO_MIN_PULSE;
    }
    digitalWrite(SERVO_PWM_PIN, HIGH);
    delayMicroseconds(pulse_duration);
    digitalWrite(SERVO_PWM_PIN, LOW);    
}

void loop(void) {
  delay(10000);
}
