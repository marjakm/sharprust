#include "lookups.h"
#include "fixed.h"
#include "communication.h"
#include "driver.h"
#include "mcdriver.h"

#define SWEEP_STEP 100
#define SWEEP_PULSE_REPETITON_COUNT 10

#define DRIVING_MIN_PULSE 1000
#define DRIVING_NORM_B 1200
#define DRIVING_NEUTRAL 1500
#define DRIVING_NORM_F 1800
#define DRIVING_MAX_PULSE 2000
#define DRIVING_PWM_PIN 11

#define STEERING_MIN_PULSE 1000
#define STEERING_NEUTRAL 1500
#define STEERING_MAX_PULSE 2000
#define STEERING_PWM_PIN 12

#define SENSOR_SWITCH_0 9
#define SENSOR_SWITCH_1 10

// Sharp measurement takes 38.3ms+-9.6ms
// Divide it by two and convert to us
#define STEP_DELAY 19150

uint8_t sensors[2][3] = {
  {A0, A2, A4}, // Group 0: left, front, right
  {A1, A3, A5}  // Group 1: left, front, right
};

IntervalTimer step_timer;
volatile int step_counter = 0;
volatile int started = 0;

bc_telemetry_packet_t telemetry;
volatile int sensor_group_in_use;
MCDriver driver(STEERING_NEUTRAL, DRIVING_NEUTRAL, DRIVING_MAX_PULSE, DRIVING_NORM_F, DRIVING_NORM_B, DRIVING_MIN_PULSE);


void setup(void) {
  telemetry.header = BC_TELEMETRY;
  telemetry.ir_front_left        = 0;
  telemetry.ir_front_right = 0;
  
  for (int i=0; i<2; i++) {
    for (int j=0; j<3; j++){
      pinMode(sensors[i][j], INPUT);
    }
  }
  pinMode(STEERING_PWM_PIN,OUTPUT);
  pinMode(DRIVING_PWM_PIN,   OUTPUT);
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
    telemetry.time = millis();
    telemetry.ir_left   = irLookup[analogRead(sensors[sensor_group_in_use][0])];
    telemetry.ir_front  = irLookup[analogRead(sensors[sensor_group_in_use][1])];
    telemetry.ir_right  = irLookup[analogRead(sensors[sensor_group_in_use][2])];
    drive_cmd_t& drive_cmd = driver.drive(telemetry);
    send_pwm_command(DRIVING_PWM_PIN,  DRIVING_MIN_PULSE,  DRIVING_MAX_PULSE,  drive_cmd.driving_pwm);
    send_pwm_command(STEERING_PWM_PIN, STEERING_MIN_PULSE, STEERING_MAX_PULSE, drive_cmd.steering_pwm);
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

void send_pwm_command(int pin, int min_pulse, int max_pulse, int pulse_duration) {
  if (pulse_duration > max_pulse){
    pulse_duration = max_pulse;
  } else {
   if (pulse_duration < min_pulse) {
      pulse_duration = min_pulse;
    }
  }
  digitalWrite(pin, HIGH);
  delayMicroseconds(pulse_duration);
  digitalWrite(pin, LOW);
}

volatile int sweep_pulse_duration = 0;
void sweep_servo(int pin, int max_pulse, int min_pulse) {
    if (step_counter % SWEEP_PULSE_REPETITON_COUNT == 0) { 
      sweep_pulse_duration += SWEEP_STEP;
    }
    send_pwm_command(pin, min_pulse, max_pulse, sweep_pulse_duration);
}

void loop(void) {
  delay(10000);
}
