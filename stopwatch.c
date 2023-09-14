/*
 * stopwatch.c
 *
 *  Created on: Sep 13, 2023
 *      Author: Mark Nagy Georgy
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/*
 * Fcpu = 1 MHz ,N =1024
 * Ftimer = 1 MHz/1024 = 976.5625 Hz
 * Ttimer = 1 / 976.5625 = 1.024 msec
 *
 * Tcompare = value choose to be in compare register x Ttimer  =250 x 1.024 msec = 256 msec
 * interrupts or compare matches per second = 1000 msec/ 256 msec = 4
 *
 */

#define COMPARE_MATCHES_PER_SECOND 4
#define VALUE_OF_COMPARE_REG 250

/* declaring and initializing seconds, minutes,hours variables as global variables */
unsigned char g_seconds = 0;

unsigned char g_minutes = 0;

unsigned char g_hours = 0;

unsigned char g_tick = 0;


/* timer 1 Interrupt handler */
ISR(TIMER1_COMPA_vect){

	g_tick++;
	if(g_tick == COMPARE_MATCHES_PER_SECOND){
		g_tick = 0;
		g_seconds++;
	}
	if(g_seconds == 60){
		g_seconds = 0;
		g_minutes ++;
	}
	if(g_minutes == 60){
		g_minutes =0;
		g_hours ++;
	}
	GIFR |= ( 1 << OCF1A); // clearing interrupt flag (auto done at end of ISR but for correctness)
}

/* INT0 Interrupt handler --> reset stop watch*/
ISR(INT0_vect){
	g_seconds = 0;
	g_minutes = 0;
	g_hours = 0;
	GIFR |= ( 1 << INTF0);
}

/* INT1 Interrupt handler --> pause stop watch*/
ISR(INT1_vect){
	TCCR1B &= ~(1<<CS10) & ~(1<<CS11) & ~(1<<CS12); //turn off clock which means stop timer (pause)
	GIFR |= ( 1 << INTF1);
}

/* INT2 Interrupt handler --> resume stop watch*/
ISR(INT2_vect){
	TCCR1B = (1<<CS12) | (1<<CS10) | (1<<WGM12); //enable clock again at N=1024 to continue timer
	GIFR |= ( 1 << INTF2);

}


void Timer1_CTC_Init(void){

	TCNT1 = 0; //initial value of timer
	TCCR1A =  (1<<FOC1A) | (1<<FOC1B) | (1<<WGM12); //non-PWM mode
	OCR1A = VALUE_OF_COMPARE_REG; //compare value
	SREG |=(1<<7); //enable i-bit
	TIMSK |= (1<<OCIE1A); //the Timer/Counter1 Output Compare A match interrupt is enabled
	TCCR1B = (1<<CS12) | (1<<CS10) | (1<<WGM12); // choosing N = 1024 and enable clock (start timer)

}



void display(){
 /* delay at end of each display to prevent blinking
  *
  * every time we select a display to enable using PORTA connected to the multiplexer and
  * then show corresponding counter that should be displayed using PORTC
  * '%'is used to find unit and '/' is used to find the tenth
  *
  * */

	PORTA = (PORTA & 0xC0) | 0x01;
	PORTC = (PORTC & 0xF0) | (g_seconds % 10);
	_delay_ms(1);

	PORTA = (PORTA & 0xC0)| 0x02;
	PORTC = (PORTC & 0xF0) | (g_seconds / 10);
	_delay_ms(1);

	PORTA = (PORTA & 0xC0)| 0x04;
	PORTC = (PORTC & 0xF0) | (g_minutes % 10);
	_delay_ms(1);

	PORTA = (PORTA & 0xC0)| 0x08;
	PORTC = (PORTC & 0xF0) | (g_minutes / 10);
	_delay_ms(1);

	PORTA = (PORTA & 0xC0)| 0x10;
	PORTC = (PORTC & 0xF0) | (g_hours % 10);
	_delay_ms(1);

	PORTA = (PORTA & 0xC0)| 0x20;
	PORTC = (PORTC & 0xF0) | (g_hours / 10);
	_delay_ms(1);

}

void INT0_Init(void){

	DDRD &= ~(1<<PD2); //configuring PD2 asinput pin
	PORTD |= (1<<PD2); //internal pull up resistor

	/* fire interrupt at falling edge caused by push button as it's connected pull up connection */
	MCUCR  |= (1<<ISC01);
	MCUCR &= ~(1<<ISC00);

	GICR |= (1<<INT0); // External Interrupt Request 0 Enable


}

void INT1_Init(){

 DDRD &= ~(1<<PD3); //configuring PD3 as input pin (external push button)
 MCUCR |= (1<<ISC11)| (1<<ISC10); // The rising edge of INT1 generates an interrupt request.
 GICR |= (1<<INT1); // External Interrupt Request 1 Enable

}

void INT2_Init(){

	DDRB &= ~(1<<PB2); //configuring PB2 as input pin
	PORTB |= (1<<PB2); //internal pull up resistor

	/* fire interrupt at falling edge caused by push button as it's connected pull up connection */
	MCUCR  |= (1<<ISC01);
	MCUCR &= ~(1<<ISC00);

	GICR |= (1<<INT2); // External Interrupt Request 2 Enable

}


int main(void){
	DDRC |= 0x0F; //configuring first 4 pins of PORTC as outputs
	DDRA |= 0x3F;  //configuring first 6 pins of PORTA as outputs

	Timer1_CTC_Init();
	INT0_Init();
	INT1_Init();
	INT2_Init();


	while(1)
	{
		display();
	}

}

