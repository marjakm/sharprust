#include "mcdriver.h"
#include "firmware.h"

static const fixed VAL_SQRT_1_DIV_2(0.70710678118654752440084436210485);
static const fixed VAL_1_DIV_45(0.02222222222222222222222222222222);
static const fixed VAL_1_DIV_150(0.00666666666666666666666666666666);

#ifdef DEBUG
  #include "stdio.h"
#endif

MCDriver::MCDriver(HardwareSerial &serial, fixed ticks_to_second, int min_steering, 
                   int neutral_steering,   int max_steering,      int range_steering_deg, 
                   int neutral_driving,    int max_driving,       int norm_driving_f, 
                   int norm_driving_b,     int min_driving_b) {
                     
	state       = STATE_IDLE;
	maybe_stuck = false;

        driving_max      = max_driving;
        driving_norm_f   = norm_driving_f;
        driving_neutral  = neutral_driving;
        driving_norm_b   = norm_driving_b;
        driving_min_b    = min_driving_b;
        ticks_per_second = ticks_to_second;
        ser = &serial;
        
        maybe_stuck_tick_nr = 0;
        not_stuck_counter   = 0;
        last_speed_add      = 0;
        last_turn           = 0;
        
        constant_speed_counter       = 0;
        constant_speed_counter_hyst  = 0;
        constant_turn_counter        = 0;
        constant_turn_counter_hyst   = 0;
        turn_direction_counter       = 0;
        turn_direction_counter_hyst  = 0;
        
        steering_neutral    = neutral_steering;
        steering_deg_to_pwm = (min_steering - max_steering)/range_steering_deg;
        
        steering = 0;
	drive_cmd.steering_pwm = neutral_steering;
	drive_cmd.driving_pwm  = neutral_driving;
}

void MCDriver::_calc_direction(bc_telemetry_packet_t& telemetry) {
	// left and right 45 degrees from center (y-axis)
	// front_left and front_right 30 degrees from center (y-axis)
	fixed a1 = telemetry.ir_left * VAL_SQRT_1_DIV_2;
	l.x = -a1;
	l.y = a1;
	f.y = telemetry.ir_front; // f.x always 0
	fixed a2 = telemetry.ir_right * VAL_SQRT_1_DIV_2;
	r.x = a2;
	r.y = a2;

	min_front = int(f.y);

	// Fill missing telemetry values
	fixed inv_wsum = 1 / (telemetry.ir_left + telemetry.ir_front + telemetry.ir_right);
	telemetry.mc.x = (telemetry.ir_left * l.x + telemetry.ir_right * r.x) * inv_wsum;
	telemetry.mc.y = telemetry.ir_front * f.y * inv_wsum;

	telemetry.mc_dist  = telemetry.mc.get_distance();
	telemetry.mc_angle = telemetry.mc.get_deg_angle();

#ifdef DEBUG
	printf("%2d  %2d  |  x=%4d  y=%4d  d=%9d  t=%9d  |  ",
			int(a1), int(a2),
			int(telemetry.mc.x),
			int(telemetry.mc.y),
			int(telemetry.mc_dist),
			2*(int(telemetry.mc_angle)-90)
	);
#endif
}

fixed MCDriver::_calc_steering_pwm(fixed steering_direction_deg) {
  return steering_neutral-steering_deg_to_pwm*steering_direction_deg;
}

drive_cmd_t& MCDriver::drive(bc_telemetry_packet_t& telemetry, int tick_nr) {   
	_calc_direction(telemetry);
	maybe_stuck = (int(telemetry.mc_dist) < 10) || (int(min_front) < 30) || ((int(telemetry.ir_left)==80) && (int(telemetry.ir_right)==80));
	turn = telemetry.mc_angle - 90;
	steering = 2 * int(turn);

        #ifdef USE_SERIAL
          ser->printf("drive l %3d,%3d  f %3d,%3d r %3d,%3d mc_dist %3d min_front %3d maybe_stuck %d\n", 
            int(l.x),int(l.y), int(f.x),int(f.y), int(r.x),int(r.y), int(telemetry.mc_dist), int(min_front), maybe_stuck
          );
        #endif
        

	switch (state) {
		case STATE_NORMAL:
			// steering calculations
			drive_cmd.steering_pwm = _calc_steering_pwm(-steering);

			// normal operation
			front_fact = (45 - turn.abs()) * VAL_1_DIV_45; // correct speed by turn angle
                        if (tick_nr % 2 == 1) { //150 cm senor
                          angle_fact = telemetry.ir_front * VAL_1_DIV_150;
                        } else {
                          angle_fact = (telemetry.ir_front-20) * 0.01;
                        }
			
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
			speed_add = angle_fact * (front_fact * fixed(driving_max - driving_norm_f));
                        speed_diff = int(last_speed_add - speed_add);
                        last_speed_add = speed_add;
                        
                        if ((speed_diff < 15) and (speed_diff > -15)) {
                          constant_speed_counter++;
                          constant_speed_counter_hyst = 0;
                          drive_cmd.driving_pwm = driving_norm_f + speed_add;
                        } else {
                          if (tick_nr % 2 == 0) {
                            if (constant_speed_counter_hyst > 2){
                              constant_speed_counter      = 0;
                              constant_speed_counter_hyst = 0;
                            } else {
                              constant_speed_counter_hyst++;
                            }
                          }
                          if ((speed_diff > -25) and (speed_diff < 30)) {
                            drive_cmd.driving_pwm = driving_norm_f + 2*speed_add;
                          }
                          else if (speed_diff < -25) {
                            drive_cmd.driving_pwm = driving_neutral-40;
                          } else {
                            drive_cmd.driving_pwm = driving_max;
                          }
                        }
                        
                        if (tick_nr % 2 == 0) {
                          if (int(turn)-last_turn < 10 and int(turn)-last_turn > -10) {
                            constant_turn_counter++;
                            constant_turn_counter_hyst=0;
                          } else {
                            if (constant_turn_counter_hyst > 2){
                              constant_turn_counter = 0;
                              constant_turn_counter_hyst=0;
                            } else {
                              constant_turn_counter_hyst++;
                            }
                          }
                        }
                        last_turn = turn;
                        
                        
                          
                       if (constant_speed_counter > 30 || constant_turn_counter > 40 ) {
                            state = STATE_BACKING;
                            backing_start_tick_nr  = tick_nr;
                            maybe_stuck_tick_nr    = 0;
                            constant_speed_counter = 0;
                            constant_speed_counter_hyst = 0;
                            constant_turn_counter  = 0;
                            constant_turn_counter_hyst=0;
                            break;
                       }
                        
			// stuck countdown
			if (maybe_stuck) {
                          not_stuck_counter = 0;
                          if (maybe_stuck_tick_nr == 0) {
                            maybe_stuck_tick_nr = tick_nr;            
                          } else {
                            if (tick_nr - maybe_stuck_tick_nr >= int(STUCK_BEFORE_BACKING_SECONDS*ticks_per_second)) {
                              state = STATE_BACKING;
                              backing_start_tick_nr = tick_nr;
                              maybe_stuck_tick_nr = 0;
			      }   
                          }
			} else {
                          not_stuck_counter++;
                          if (not_stuck_counter >= 5) {
                            maybe_stuck_tick_nr = 0;
                          }
			}
			break;

		case STATE_BACKING:
                        #ifdef USE_SERIAL
                          ser->printf("backing\n");
                        #endif
			drive_cmd.steering_pwm = _calc_steering_pwm(steering/2);
			drive_cmd.driving_pwm  = driving_norm_b;
                        if (tick_nr - backing_start_tick_nr >= int(BACKING_SECONDS*ticks_per_second)) {
                          state = STATE_NORMAL;
                        }
			break;

                case STATE_CLIMB_MOUNTAIN:
                  drive_cmd.steering_pwm = _calc_steering_pwm(0);
	          drive_cmd.driving_pwm  = driving_max;
                  if (tick_nr - backing_start_tick_nr >= int(MOUNTAIN_CLIMB_SECONDS*ticks_per_second)) {
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

