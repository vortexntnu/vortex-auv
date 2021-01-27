#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

/**
 * @brief This function does a linear transformation of a 16 bit control signal
 * in the range [1000, 2000] to a 8 bit control signal in the range [127, 255] 
 * 
 * 
 * From linear calculations: 
 *      thruster_control_signal = 1000 => pwm_signal = 127
 *      thruster_control_signal = 2000 => pwm_signal = 255
 * 
 * we get the function f: R -> R given by
 * pwm_signal := f(thruster_control_signal) = 0.128 * thruster_control_signal - 1  
 * 
 * The calculated pwm_signal - value is casted to uint8_t as we desire values between
 * 0x7F and 0xFF
 *
 * @warning If the given value is outside the specified range, the thrust is set to idle
 * 
 * @warning This linear transformation assumes that the clock-scaler is 
 * set to 1/64
 * 
 * @warning May be a redundant code, due to using a 8 bit MCU. The thruster-commands
 * should therefore be specified to be within 0x00 and 0xFF
 */
uint8_t calculate_pwm_counter(const uint16_t& thruster_control_signal){
  /**
   * Checking if the signal is valid
   * 
   * If not valid (outside the range), the thruster is set to idle thrust
   */
  if(thruster_control_signal < 1000 || thruster_control_signal > 2000)
    return (uint8_t) ((0xFF - 0x7F) / 2 + 0x7F);
  
  return (uint8_t) (0.128 * thruster_control_signal - 1);
}


void setup() {  
  /* Clear ports */
  DDRE = 0;
  DDRH = 0;
  DDRL = 0;
  DDRF = 0;


  /* Set MOSI and SCK output, all others input */
  DDR_SPI = (1 << DD_MOSI) | (1 << DD_SCK);
  
  /* Enable SPI, Master, set clock rate fck/16 */
  SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
  
  
  /* Setting ports as output */
  DDRE |= (1 << PE3) | (1 << PE4) | (1 << PE5); 
  DDRH |= (1 << PH1) | (1 << PH3) | (1 << PH4) | (1 << PH5);
  DDRL |= (1 << PL3) | (1 << PL4);
  
  
  /* Initializing ports as pull-down */
  PORTE &= !((1 << PE3) | (1 << PE4) | (1 << PE5));  
  PORTH &= !((1 << PH2) | (1 << PH1) | (1 << PH0));
  PORTL &= !((1 << PL3) | (1 << PL4));


  /* Init upper count-limit to 0 */
  OCR3A = 0; /* Count THR0 */
  OCR3B = 0; /* Count THR1 */ 
  OCR3C = 0; /* Count THR2 */ 
  
  OCR4A = 0; /* Count THR3 */ 
  OCR4B = 0; /* Count THR4 */ 
  OCR4C = 0; /* Count THR5 */ 
  
  OCR5A = 0; /* Count THR6 */ 
  OCR5B = 0; /* Count THR7 */
  
  
  /* Setting A to Fast PWM, clearing OCnA and OCnB on compare match */
  /* | COMnA1 | COMnA0 | COMnB1 | COMnB0 | WGMn3 | WMGn2 | WMGn1 | WMGn0 | = | 1 | 0 | 1 | 0 | 0 | 1 | 1 | 0 | */
  TCCR3A = (1 << COM3A1) | (1 << COM3B1) | (1 << COM3C1) | (1 << WGM32) | (1 << WGM31);
  TCCR4A = (1 << COM4A1) | (1 << COM4B1) | (1 << COM4C1) | (1 << WGM42) | (1 << WGM41);
  TCCR5A = (1 << COM5A1) | (1 << COM5B1) | (1 << COM5C1) | (1 << WGM52) | (1 << WGM51);
  
  
  /* Setting clock-source to 1/64 * clk. See table 17-6 */
  TCCR3B = (1 << CS31) | (1 << CS30);
  TCCR4B = (1 << CS41) | (1 << CS40);
  TCCR5B = (1 << CS51) | (1 << CS50);
  
  
  /* Init timers */
  TCNT3 = 0;
  TCNT4 = 0;
  TCNT5 = 0;


  /* Enabling global interrupt */
  SREG = (1 << 7);
  

  /* Enable LEDs */
  DDRF |= (1 << PF0) | (1 << PF1);


  /* Wait 1 ms before starting any other code */
  _delay_ms(1);
}

void loop() {
  /* Setting LED D3 high and LED D2 low */
  PORTF = 0xAA;

  /* Testing PWM-signals on THR7 */
  OCR5B = 127;
  _delay_ms(100);
  
  /* Setting LED D2 high and LED D3 low */
  PORTF = 0x55;
  _delay_ms(100);
}