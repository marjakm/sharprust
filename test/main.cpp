#include "stdio.h"
#include "math.h"
#include "mcdriver.h"
#include "communication.h"
#include "fixed.h"

// Bigger number is forwards
// 1400 - 1600
#define DRIVING_MIN_PULSE 1450
#define DRIVING_NORM_B 1400
#define DRIVING_NEUTRAL 1500
#define DRIVING_NORM_F 1550
#define DRIVING_MAX_PULSE 1550
#define DRIVING_PWM_PIN 11

// Bigger number turns left
#define STEERING_MIN_PULSE 1340
#define STEERING_NEUTRAL 1480
#define STEERING_MAX_PULSE 1560
#define STEERING_RANGE_DEG 10
#define STEERING_PWM_PIN 12

#define SENSOR_SWITCH_0 9
#define SENSOR_SWITCH_1 10

// Sharp measurement takes 38.3ms+-9.6ms
// Divide it by two and convert to us
#define STEP_DELAY 19150

MCDriver driver(STEP_DELAY, STEERING_MIN_PULSE , STEERING_NEUTRAL, STEERING_MAX_PULSE, STEERING_RANGE_DEG, DRIVING_NEUTRAL, DRIVING_MAX_PULSE, DRIVING_NORM_F, DRIVING_NORM_B, DRIVING_MIN_PULSE);
bc_telemetry_packet_t telemetry;

int main() {
    telemetry.time           = 0;
    telemetry.ir_left        = 0;
    telemetry.ir_right       = 0;
    telemetry.ir_front_left  = 0;
    telemetry.ir_front_right = 0;
    telemetry.ir_front       = 94;
    int step_counter = 0;

    for ( int val = 0; val<95; val++ ) {
        telemetry.time += 10;
        telemetry.ir_left     = sqrt(2)*val;
        telemetry.ir_right    = sqrt(2)*(94-val);
        step_counter++;
        drive_cmd_t& dc = driver.drive(telemetry, step_counter);

        printf("l %3d r %3d fl %3d fr %3d steer %3d drive %3d\n",
            int(telemetry.ir_left),
            int(telemetry.ir_right),
            int(telemetry.ir_front_left),
            int(telemetry.ir_front_right),
            int(dc.steering_pwm)-STEERING_NEUTRAL,
            int(dc.driving_pwm)-DRIVING_NEUTRAL
        );
    }


}
