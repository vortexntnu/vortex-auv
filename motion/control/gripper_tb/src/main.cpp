#include <Arduino.h>
#include <SoftwareSerial.h>

// define pins used
#define servo_pwm_pin 7
#define dc_motor_pwm_pin 6
#define ks_pin 3
#define green_led 8
#define red_led 9
#define pot_pin A1

// Setup for serial to pc
SoftwareSerial testPort(1, 0);

// defining constants f
unsigned long pwm_full_duty = 1860;
unsigned long pwm_zero_duty = 1040;

// global variables
uint16_t control_val = 0;
unsigned long dc_motor_pwm_count = 0;
uint16_t duty_cycle = 0;
const double adc_to_duty =
    820.0 / 1023.0; // (pwm_full_duty - pwm_zero_duty) / adc_resolution

// function declarations
uint16_t adc_to_duty_percent(int adc_val);
uint16_t duty_percent_to_period(uint8_t duty_percent);
void dc_motor_pwm_control(uint16_t duty);
void check_duty_low();
void arm_esc();

void setup() {
  /*
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  // Start each software serial port
  testPort.begin(9600);
  testPort.listen();
  */
  // Serial.println(adc_to_duty);
  //  Setup pins for inn and output
  pinMode(green_led, OUTPUT);
  pinMode(red_led, OUTPUT);
  pinMode(dc_motor_pwm_pin, OUTPUT);
  // pinMode(servo_pwm_pin, OUTPUT);
  pinMode(pot_pin, INPUT);
  pinMode(ks_pin, INPUT);
  /* dont start program unless dc control is at 0 and kill switch is not
   * activated */
  check_duty_low();
  arm_esc();
}
// Main loop
void loop() {
  // put your main code here, to run repeatedly:
  if (digitalRead(ks_pin)) { /* Kill switch not activated */
    control_val = analogRead(pot_pin);

    dc_motor_pwm_control(adc_to_duty_percent(control_val));
  } else { /* Kill switch activated */
    /* dont continue program again unless dc control is at 0 and kill switch is
     * not activated */
    check_duty_low();
  }
}

/* dont continue unless dc control is at 0 and kill switch is not activated */
void check_duty_low() {
  control_val = analogRead(pot_pin);
  while ((control_val > 20) || (digitalRead(ks_pin) == 0)) {
    control_val = analogRead(pot_pin);
    digitalWrite(red_led, HIGH);
    digitalWrite(green_led, LOW);
    digitalWrite(dc_motor_pwm_pin, LOW);
  }
  digitalWrite(green_led, HIGH);
  digitalWrite(red_led, LOW);
}
/*
 *
 */
uint16_t adc_to_duty_percent(int adc_val) {
  uint16_t duty_percent = 0;
  if (adc_val > 20) {
    duty_percent = adc_val * adc_to_duty;
  }
  return duty_percent;
}

void dc_motor_pwm_control(uint16_t duty_cycle) {
  // digitalWrite(red_led, HIGH);
  unsigned long dc_duty = duty_cycle + pwm_zero_duty;
  if (dc_motor_pwm_count + dc_duty <= micros()) {
    digitalWrite(dc_motor_pwm_pin, LOW);
    // Serial.println(dc_duty);
    if (dc_motor_pwm_count + pwm_full_duty <= micros()) {
      dc_motor_pwm_count = micros();
      digitalWrite(dc_motor_pwm_pin, HIGH);
    }
  }
}

void arm_esc() {
  digitalWrite(dc_motor_pwm_pin, LOW);
  delayMicroseconds(1860 * 3);
}