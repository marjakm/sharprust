#include "lookups.h"
#include "fixed.h"
#include "communication.h"
#include "mcdriver.h"
#include "firmware.h"

fixed ticks_per_second = 1000000./STEP_DELAY;

uint8_t sensors[2][3] = {
  {A0, A2, A4}, // Group 0: left, front, right
  {A1, A3, A5}  // Group 1: left, front, right
};

volatile main_loop_states_t main_loop_state;

IntervalTimer step_timer;
volatile int  step_counter = 0;
volatile int  button_first_down_step = 0;
volatile int  first_start_delay_step = 0;

bc_telemetry_packet_t telemetry;
volatile int sensor_group_in_use;

MCDriver *driver;
drive_cmd_t drive_cmd;

void setup(void) {
  #ifdef USE_SERIAL
    SERIALDEV.begin(115200);
  #endif
  driver = new MCDriver(SERIALDEV, ticks_per_second, STEERING_MIN_PULSE , STEERING_NEUTRAL, STEERING_MAX_PULSE, STEERING_RANGE_DEG, DRIVING_NEUTRAL, DRIVING_MAX_PULSE, DRIVING_NORM_F, DRIVING_NORM_B, DRIVING_MIN_PULSE);
  
  telemetry.header = BC_TELEMETRY;
  telemetry.ir_front_left  = 0;
  telemetry.ir_front_right = 0;
  main_loop_state  = STATE_STARTING_SENSORS;
  
  for (int i=0; i<2; i++) {
    for (int j=0; j<3; j++){
      pinMode(sensors[i][j], INPUT);
    }
  }
 
  pinMode(STEERING_PWM_PIN,OUTPUT);
  pinMode(DRIVING_PWM_PIN, OUTPUT);
  pinMode(SENSOR_SWITCH_0, OUTPUT);
  pinMode(SENSOR_SWITCH_1, OUTPUT);
  pinMode(LED_BUILTIN,     OUTPUT);
  pinMode(START_BUTTON,    INPUT);
  pinMode(BATTERY_VOLTAGE_PIN, INPUT);
  
  analogReference(DEFAULT);
  analogReadAveraging(16);
  analogReadResolution(10);
  step_timer.begin(step_main, STEP_DELAY);
}

void step_main(void) {
  noInterrupts();
  step_counter++;
  switch (main_loop_state) {
    case STATE_SWEEP_MOTOR:
      sweep_servo(DRIVING_PWM_PIN, DRIVING_MIN_PULSE, DRIVING_MAX_PULSE);
      break;   
    case STATE_SWEEP_STEERING:
      sweep_servo(STEERING_PWM_PIN, STEERING_MIN_PULSE, STEERING_MAX_PULSE);
      break;   
    case STATE_STARTING_SENSORS:
      turn_on_sensors();
      break;
    case STATE_WAIT_BUTTON:
      heartbeat(40);
      //drive_cmd = do_measurements();
      check_button(STATE_WAIT_DELAY);
      break;
    case STATE_WAIT_DELAY:
      heartbeat(10);
      if (first_start_delay_step == 0) {
        first_start_delay_step = step_counter;
      } else if (step_counter-first_start_delay_step >= int(ticks_per_second*START_DELAY_SEC)) {
        main_loop_state = STATE_RUNNING;
      }
      break;
    case STATE_RUNNING:
      heartbeat(20);
      check_button(STATE_WAIT_BUTTON);
      drive_cmd = do_measurements();
      send_pwm_command(DRIVING_PWM_PIN,  DRIVING_MIN_PULSE,  DRIVING_MAX_PULSE,  int(drive_cmd.driving_pwm));
      send_pwm_command(STEERING_PWM_PIN, STEERING_MIN_PULSE, STEERING_MAX_PULSE, int(drive_cmd.steering_pwm));
      break;
  }
  interrupts();
}

void check_button(main_loop_states_t next_state) {
  if (digitalRead(START_BUTTON) == LOW) {
    if (button_first_down_step == 0) {
      button_first_down_step = step_counter;     
    }
  } else {
    if ((button_first_down_step != 0) and (step_counter - button_first_down_step >= int(ticks_per_second*REQ_BUTTON_PRESS_SEC))) {
      main_loop_state = next_state;
    }
    button_first_down_step = 0;
  }
}

drive_cmd_t do_measurements(void) {
  sensor_group_in_use = step_counter % 2;
  telemetry.ir_left   = ir80Lookup[analogRead(sensors[sensor_group_in_use][0])];
  if (sensor_group_in_use == 0){
    telemetry.ir_front  = ir80Lookup[analogRead(sensors[sensor_group_in_use][1])];
  } else {
    telemetry.ir_front  = ir150Lookup[analogRead(sensors[sensor_group_in_use][1])];
  }
  telemetry.ir_right  = ir80Lookup[analogRead(sensors[sensor_group_in_use][2])];
  return driver->drive(telemetry, step_counter);
}

void heartbeat(int interval_step_count) {
  if (step_counter % interval_step_count == 0) {
    if (step_counter % (2*interval_step_count) == 0) {
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
    main_loop_state = STATE_WAIT_BUTTON;
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
volatile int sweep_direction = 1;
void sweep_servo(int pin, int min_pulse, int max_pulse) {
    if (step_counter % SWEEP_PULSE_REPETITON_COUNT == 0) { 
      if (sweep_direction == 1) {
        sweep_pulse_duration += SWEEP_STEP;
      } else {
        sweep_pulse_duration -= SWEEP_STEP;
      }
    }
    if (sweep_pulse_duration >= max_pulse){
      sweep_pulse_duration = max_pulse;
      sweep_direction = 0;
    } else {
     if (sweep_pulse_duration <= min_pulse) {
        sweep_pulse_duration = min_pulse;
        sweep_direction = 1;
      }
    }
    send_pwm_command(pin, min_pulse, max_pulse, sweep_pulse_duration);
}

void loop(void) {
  delay(10000);
}

