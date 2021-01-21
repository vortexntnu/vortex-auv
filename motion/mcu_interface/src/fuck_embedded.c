#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

/* Defines to quicly change between the thrusters. Could use something like this in the future */
#define THR0 0x98
#define THR1 0x9A
#define THR2 0x9C
#define THR3 0xA8
#define THR4 0xAA
#define THR5 0xAC
#define THR6 0x128
#define THR7 0x12A

/* Setting clock */
/* 16MHz */
CKSEL = 1111;
SUT = 11;
__no_operation();


/* See tables 22-10 to 22-12 for baudrate */
/* Setting baudrate to 38400 Hz */
UBBR = 0x19;
U2X = 0;


/* Clear ports */
DDRE = 0;
DDRH = 0;
DDRL = 0;


/* Setting ports as output */
DDRE |= (1 << PE3) | (1 << PE4) | (1 << PE5); 
DDRH |= (1 << PH1) | (1 << PH3) | (1 << PH4) | (1 << PH5);
DDRL |= (1 << PL3) | (1 << PL4);


/* Initializing ports as pull-down */
PORTE &= !((1 << PE3) | (1 << PE4) | (1 << PE5));  
PORTH &= !((1 << PH2) | (1 << PH1) | (1 << PH0));
PORTL &= !((1 << PL3) | (1 << PL4));


/* Init duty-cycles to 0 */
OCR3A = 0; /* Duty-cycle THR0 */
OCR3B = 0; /* Duty-cycle THR1 */ 
OCR3C = 0; /* Duty-cycle THR2 */ 

OCR4A = 0; /* Duty-cycle THR3 */ 
OCR4B = 0; /* Duty-cycle THR4 */ 
OCR4C = 0; /* Duty-cycle THR5 */ 

OCR5A = 0; /* Duty-cycle THR6 */ 
OCR5B = 0; /* Duty-cycle THR7 */


/* Setting A to Fast PWM, clearing OCnA, OCnB and OCnC on compare match */
TCCR3A = (1 << COM3A1) | (1 << COM3B1) | (1 << COM3C1) | (1 << WGM32) | (1 << WGM31) | (1 << WGM30);
TCCR4A = (1 << COM4A1) | (1 << COM4B1) | (1 << COM4C1) | (1 << WGM42) | (1 << WGM41) | (1 << WGM40);
TCCR5A = (1 << COM5A1) | (1 << COM5B1) | (1 << COM5C1) | (1 << WGM52) | (1 << WGM51) | (1 << WGM50);


/* Setting clock-source to 1/1024 * clk. See table 17-6 */
TCCR3B = (1 << CS32) | (1 << CS30);
TCCR4B = (1 << CS42) | (1 << CS40);
TCCR4B = (1 << CS52) | (1 << CS50);


/* Enabling timers */
PRR1 &= !((1 << PRTIM3) | (1 << PRTIM4) | (1 << PRTIM5));


/* Enabling interrupt */
SREG |= (1 << I);


/* Init timers */
TCNT3 = 0;
TCNT4 = 0;
TCNT5 = 0;


/* Inserting nop for system-sync */
__no_operation();

/* Function to map the wanted Duty-cycle to counting-variable */
/* Variables used: 
clock_speed = 16MHz 
clk_divisior = 1024

OCRnx could be set to at most 0xFF

It is required to develop a function to translate the value
into the required duty-cycle
*/
uint16_t offset = 0;
uint8_t calculate_OCRnx_signal(const uint16_t& value){
  return (uint8_t)((value - offset) / 0xFF);
}


/* Test to set the duty cycle */
int main(void){
  while(1){
    uint16_t base_duty_cycle = 10;
    uint16_t diff_duty_cycle = 100;
    
    for(int i = 0; i < 100; i++){
      OCR3A = base_duty_cycle + i * diff_duty_cycle;
      
      _delay_ms(250);
    }
  }
  return 0;
}