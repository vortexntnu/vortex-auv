#include <Arduino.h>

#define servo_pwm_pin 6
#define dc_motor_pwm_pin 5
#define ks_pin 2
#define green_led 8
#define red_led 9
#define pot_pin A1

#define pwm_full_duty 1860
#define pwm_zero_duty 1060


uint16_t control_val = 0;
unsigned long dc_motor_pwm_timer = 0;
uint16_t duty = 0;

// put function declarations here:
// bool ks_active();

void dc_motor_pwm_control(int duty);
void check_initial_duty();


void setup() {
  // put your setup code here, to run once:
  pinMode(green_led, OUTPUT);
  pinMode(red_led, OUTPUT);
  pinMode(dc_motor_pwm_pin, OUTPUT);
  pinMode(servo_pwm_pin, OUTPUT);
  pinMode(ks_pin, INPUT);

  check_initial_duty();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (ks_pin) {
    control_val = analogRead(pot_pin);
    dc_motor_pwm_control(duty)
  }
}

/*
bool KS_not_triggered() {
  if (ks_pin = HIGH) {
      return true
  }

}
*/
void dc_motor_pwm_control(int duty) {
if (dc_motor_pwm_timer + duty + 1060 > micros())
}

void check_initial_duty() {
  control_val = analogRead(pot_pin);
  while (control_val >= 1) {
    control_val = analogRead(pot_pin);
    digitalWrite(red_led, HIGH);
    digitalWrite(green_led, LOW);
  }
  digitalWrite(green_led, HIGH);
  digitalWrite(red_led, LOW);
}

/*
ADC converter for potmeter

pwm dc_motor

pwm_servo

bool KS_active() {

}

*/
