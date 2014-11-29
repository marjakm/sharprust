#ifndef __FIRMWARE__H
#define __FIRMWARE__H

#define SWEEP_STEP 10
#define SWEEP_PULSE_REPETITON_COUNT 50

// Bigger number is forwards
#define DRIVING_MIN_PULSE 1400
#define DRIVING_NORM_B 1400
#define DRIVING_NEUTRAL 1500
#define DRIVING_NORM_F 1630
#define DRIVING_MAX_PULSE 1630
#define DRIVING_PWM_PIN 11

// Bigger number turns left
#define STEERING_MIN_PULSE 1340
#define STEERING_NEUTRAL 1480
#define STEERING_MAX_PULSE 1560
#define STEERING_RANGE_DEG 10
#define STEERING_PWM_PIN 12

#define SENSOR_SWITCH_0 9
#define SENSOR_SWITCH_1 10
#define START_BUTTON 8
#define BATTERY_VOLTAGE_PIN A6

// Sharp measurement takes 38.3ms+-9.6ms
// Divide it by two and convert to us
#define STEP_DELAY 19150
#define REQ_BUTTON_PRESS_SEC 0.1


typedef enum main_loop_states {
        STATE_STARTING_SENSORS,
	STATE_WAIT_BUTTON,  
        STATE_RUNNING,
        STATE_SWEEP_MOTOR,
        STATE_SWEEP_STEERING,
} main_loop_states_t;

#endif
