#include <Arduino.h>

// define pins used
#define servo_pwm_pin 6
#define dc_motor_pwm_pin 5
#define ks_pin 2
#define green_led 8
#define red_led 9
#define pot_pin A1

// defining constants f
#define pwm_full_duty 1860
#define pwm_zero_duty 1060

// global variables 
uint16_t control_val = 0;
unsigned long dc_motor_pwm_timer = 0;
uint16_t duty_cycle = 0;
const float adc_to_percent = 100/1023;
bool ks_off = 1;

// function declarations
uint8_t adc_to_duty_percent(int adc_val);
uint16_t duty_percent_to_period(uint8_t duty_percent);
void dc_motor_pwm_control(uint16_t duty);
void check_duty_low();

void setup() {
  // Setup pins for inn and output
  pinMode(green_led, OUTPUT);
  pinMode(red_led, OUTPUT);
  pinMode(dc_motor_pwm_pin, OUTPUT);
  pinMode(servo_pwm_pin, OUTPUT);
  pinMode(pot_pin, INPUT);
  pinMode(ks_pin, INPUT);
  // dont start program unless dc control is at 0
  check_duty_low();
}
// Main loop
void loop() {
  // put your main code here, to run repeatedly:
  ks_off = digitalRead(ks_pin);
  if (ks_off == 1) {
    control_val = analogRead(pot_pin);
    dc_motor_pwm_control(duty_percent_to_period(adc_to_duty_percent(control_val)));
  }
  else {
    check_duty_low();
  }
}

/*
* 
*/
uint8_t adc_to_duty_percent(int adc_val) {
  uint8_t duty_percent = adc_val*(adc_to_percent);
  return duty_percent;
}

uint16_t duty_percent_to_period(uint8_t duty_percent) {
  uint16_t duty = duty_percent*8;
  return duty;
}

void dc_motor_pwm_control(uint16_t duty_cycle) {
  if (dc_motor_pwm_timer + duty_cycle + pwm_zero_duty > micros()) {
    pinMode(dc_motor_pwm_pin, LOW);
    if (dc_motor_pwm_timer + pwm_full_duty > micros()) {
      dc_motor_pwm_timer = micros();
      pinMode(dc_motor_pwm_pin, HIGH);
    }
  }
}

void check_duty_low() {
  control_val = analogRead(pot_pin);
  while (control_val > 20) {
    control_val = analogRead(pot_pin);
    digitalWrite(red_led, HIGH);
    digitalWrite(green_led, LOW);
  }
  digitalWrite(green_led, HIGH);
  digitalWrite(red_led, LOW);
}