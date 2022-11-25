/*
 Author: Philip Narteh
 Embedded Systems Final Project
*/

#ifndef FINAL_PROJECT_HEADERS_H
#define FINAL_PROJECT_HEADERS_H

#include <mkl25z4.h>
#include <stdio.h>

#define RX	1		//PTA1
#define TX	2		//PTA2
#define OSR 15		//over sample rate (like a pre-scaler)
#define BAUD_RATE  	9600	//my communication rate on BT
#define SYS_CLOCK	20971520u	//
#define MASK(x) (1UL << (x))
#define RED_LED (18)		// on port B
#define GREEN_LED (19)	// on port B
#define BLUE_LED (1)		// on port D
#define BUZZER 7

#define W1 			(9)	// on port B
#define W2	 		(8)		// on port B
#define X1 			(0)	// on port B
#define X2	 		(1)
#define BACK_LIT    (10)
#define FRONT_LIT   (3)

#define TRIGGER		(12)		//PTA12 associated with TPM1 CH0		//PWM trigger.
#define ECHO		(13)	   //PTA13

#define serviceWDOG()	\
SIM->SRVCOP = 0x55;		\
SIM->SRVCOP = 0xAA;

volatile char rxChar;
static int repeat_check = 0;
static int speed = 0;

volatile unsigned long counter=0;
unsigned long  millis(void);
void SysTick_Handler(void);
void ultrasonic_timer();
void Init_UART(void);
void TPM0_IRQHandler();
void TPM1_IRQHandler();
void TPM2_IRQHandler();
void UART0_IRQHandler(void);
unsigned long  millis(void);

void init_pins(void);
void Init_RGB_LEDs(void);
void self_test();
void buzzer(int repeat);
void control_Motor();
void movement_control();
void Control_RGB_LEDs(unsigned int red_on, unsigned int green_on, unsigned int blue_on);
void init_motor(void);
void init_motor_Timer(void);
void flashRRRBBB(void);
void slow(int pin);

int g_dist=0;		//global variable to hold distance.
static unsigned int poten_val=0U;

#endif
