#include "mcdriver.h"

static const fixed VAL_SQRT_1_DIV_2(0.70710678118654752440084436210485);
static const fixed VAL_SQRT_3_DIV_2(0.86602540378443864676372317075294);
static const fixed VAL_1_DIV_45(0.02222222222222222222222222222222);

static const fixed VAL_0_5(0.5);
static const fixed VAL_1(1);
static const fixed VAL_2(2);
static const fixed VAL_3_5(3.5);
static const fixed VAL_5_5(5.5);

#ifdef DEBUG
  #include "stdio.h"
#endif

MCDriver::MCDriver(int min_steering, int neutral_steering, int max_steering, int range_steering_deg, int neutral_driving, int max_driving, int norm_driving_f, int norm_driving_b, int min_driving_b) {
	state = STATE_IDLE;
	maybe_stuck = false;

        driving_max    = max_driving;
        driving_norm_f = norm_driving_f;
        driving_neutral= neutral_driving;
        driving_norm_b = norm_driving_b;
        driving_min_b  = min_driving_b;
        
        steering_neutral    = neutral_steering;
        steering_deg_to_pwm = (min_steering - max_steering)/range_steering_deg;
        
        steering = 0;
	drive_cmd.steering_pwm = neutral_steering;
	drive_cmd.driving_pwm  = neutral_driving;
	
	last_speed_add_timer.start(1, 1);
}

void MCDriver::_calc_direction(bc_telemetry_packet_t& telemetry) {
	// left and right 45 degrees from center (y-axis)
	// front_left and front_right 30 degrees from center (y-axis)
	fixed a1 = telemetry.ir_left * VAL_SQRT_1_DIV_2;
	l.x = -(a1 + VAL_3_5);
	l.y = a1 - VAL_1;
	f.y = telemetry.ir_front + VAL_5_5; // f.x always 0
	fixed a2 = telemetry.ir_right * VAL_SQRT_1_DIV_2;
	r.x = a2 + VAL_3_5;
	r.y = a2 - VAL_1;

	min_front = f.y;

	// Fill missing telemetry values
	fixed inv_wsum = 1 / (telemetry.ir_left + telemetry.ir_front + telemetry.ir_right);
	telemetry.mc.x = (telemetry.ir_left * l.x + telemetry.ir_right * r.x) * inv_wsum;
	telemetry.mc.y = telemetry.ir_front * f.y * inv_wsum;

	telemetry.mc_dist  = telemetry.mc.get_distance();
	telemetry.mc_angle = telemetry.mc.get_deg_angle();

#ifdef DEBUG
	printf("%2d  %2d  |  x=%4d  y=%4d  d=%9d  a=%9d  |  ",
			a1, a2,
			telemetry.mc.x,
			telemetry.mc.y,
			telemetry.mc.dist,
			telemetry.mc.angle
	);
#endif
}

fixed MCDriver::_calc_steering_pwm(fixed steering_direction_deg) {
  return steering_deg_to_pwm*steering_direction_deg+steering_neutral;
}

drive_cmd_t& MCDriver::drive(bc_telemetry_packet_t& telemetry) {
	fixed turn, speed_add, front_fact, angle_fact;

	_calc_direction(telemetry);
	maybe_stuck = (telemetry.mc_dist < 10) || (min_front < 20);
	turn = telemetry.mc_angle - 90;
	steering = 2 * int(turn);

	switch (state) {
		case STATE_NORMAL:
			// steering calculations
			drive_cmd.steering_pwm = _calc_steering_pwm(-steering);

			// speed calculations
			speed_add = fixed(driving_max - driving_norm_f);

			// normal operation
			front_fact = (45 - turn.abs()) * VAL_1_DIV_45; // correct speed by turn angle
			angle_fact = (telemetry.ir_front - 20) * 0.01; // correct speed by front distance
			if (front_fact < 0) {
				front_fact = 0;
			}
			else if (front_fact > 1) {
				front_fact = 1;
			}
			if (angle_fact < 0) {
				angle_fact = 0;
			}
			else if (angle_fact > 1) {
				angle_fact = 1;
			}
			speed_add = angle_fact * (front_fact * speed_add);
			drive_cmd.driving_pwm = driving_norm_f + speed_add;

			// for abrupt lowering of speed go to braking state
			if (speed_add < last_speed_add - 5) {
				stuck_timer.stop();
				last_speed_add = speed_add;
				drive_cmd.driving_pwm = driving_min_b;
				state = STATE_BRAKING;
				break;
			}
			last_speed_add = speed_add;

			// stuck countdown
			if (maybe_stuck) {
				if (!stuck_timer.running()) {
					stuck_timer.start(telemetry.time, 1000);
				}
				else if (stuck_timer.triggered(telemetry.time)) {
					stuck_timer.stop();
					state = STATE_BACKING;
				}
			}
			else {
				stuck_timer.stop();
			}
			break;

		case STATE_BACKING:
			drive_cmd.steering_pwm = _calc_steering_pwm(steering);
			drive_cmd.driving_pwm = driving_norm_b;

			if (!stuck_timer.running()) {
				stuck_timer.start(telemetry.time, 2000);
			}
			if (!maybe_stuck || stuck_timer.triggered(telemetry.time)) {
				stuck_timer.stop();
				state = STATE_NORMAL;
			}
			break;

		case STATE_BRAKING:
			drive_cmd.steering_pwm = _calc_steering_pwm(-steering);
			drive_cmd.driving_pwm = driving_norm_b;

			if (!stuck_timer.running()) {
				stuck_timer.start(telemetry.time, 100);
			}
			else if (stuck_timer.triggered(telemetry.time)) {
				stuck_timer.stop();
				state = STATE_NORMAL;
			}
			break;

		case STATE_IDLE:
		default:
			state = STATE_NORMAL;
			break;
	}

	telemetry.steering_pwm = drive_cmd.steering_pwm;
	telemetry.driving_pwm  = drive_cmd.driving_pwm;

	return drive_cmd;
}
