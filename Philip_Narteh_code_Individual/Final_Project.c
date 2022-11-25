/*
 Author: Philip Narteh
 Embedded Systems Final Project
 */

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

int main (void) {
	init_pins();
	ultrasonic_timer();
	Init_UART();
	Init_RGB_LEDs();
	init_motor();
	init_motor_Timer();
	while (1) {
		flashRRRBBB();
		control_Motor();
		serviceWDOG();
		ADC0->SC1[0] = ADC_SC1_ADCH(0)  | ADC_SC1_COCO_MASK;
		poten_val = ADC0->R[0];
		poten_val &=~MASK(7);
	}
}

void init_motor(void) {
	// Enable clock to ports B and D
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK ;
	// Make 3 pins GPIO
	PORTB->PCR[W1] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[W1] |= PORT_PCR_MUX(1);
	PORTB->PCR[W2] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[W2] |= PORT_PCR_MUX(1);
	PORTB->PCR[X1] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[X1] |= PORT_PCR_MUX(1);
	PORTB->PCR[X2] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[X2] |= PORT_PCR_MUX(1);
	PORTB->PCR[BACK_LIT] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[BACK_LIT] |= PORT_PCR_MUX(1);
	PORTB->PCR[FRONT_LIT] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[FRONT_LIT] |= PORT_PCR_MUX(1);
	// Set ports to outputs
	PTB->PDDR |= MASK(W1) | MASK(W2) | MASK(X1) | MASK(X2) | MASK(BACK_LIT) | MASK(FRONT_LIT);
}

void init_motor_Timer(void){
	//Clock gate
	SIM->SCGC6 |=SIM_SCGC6_TPM2_MASK;	//***TPM2 channel 0
	//Select clock source in SIM_SOPT
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);	//1- MCGPLLCLK/2, or MCGFLLCLK 01 (ext?), 2-ext OSCERCLK, 3-internal MCGIRCLK
	//Configure registers
	TPM2->MOD= 0xCCCD;  //20ms or 50Hz period

	//working wth TPM2_C0SC
						//output compare + edge aligned PWM MSBA: 10, ELSBA:10
	TPM2->CONTROLS[0].CnSC |= TPM_CnSC_MSB(1) | TPM_CnSC_ELSB(1) ;
	TPM2->CONTROLS[0].CnSC |= TPM_CnSC_CHF_MASK;  //clear spurious interrupts
	TPM2->CONTROLS[0].CnV =0xCCCC;  //0x147B;
	TPM2->SC |=  TPM_SC_TOF_MASK | TPM_SC_PS(3) | TPM_SC_TOIE_MASK  ;

	TPM2->SC |= TPM_SC_CMOD(1); //enable internal clock to run

	SIM->SCGC6 |=SIM_SCGC6_TPM0_MASK;	//***TPM2 channel 0
		//Select clock source in SIM_SOPT
		SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);	//1- MCGPLLCLK/2, or MCGFLLCLK 01 (ext?), 2-ext OSCERCLK, 3-internal MCGIRCLK
		//Configure registers
		TPM2->MOD= 0xCCCD;  //20ms or 50Hz period

		//working wth TPM2_C0SC
							//output compare + edge aligned PWM MSBA: 10, ELSBA:10
		TPM0->CONTROLS[1].CnSC |= TPM_CnSC_MSB(1) | TPM_CnSC_ELSB(1) ;
		TPM0->CONTROLS[1].CnSC |= TPM_CnSC_CHF_MASK;  //clear spurious interrupts
		TPM0->CONTROLS[1].CnV =0xCCCC;  //0x147B;
		TPM0->SC |=  TPM_SC_TOF_MASK | TPM_SC_PS(3) | TPM_SC_TOIE_MASK;

		TPM0->SC |= TPM_SC_CMOD(1); //enable internal clock to run

}



void slow(int pin){
	static int ctr=0;
		if(pin == W1){
			movement_control(1,0,0,0);

		}else if(pin == W2){
			movement_control(0,1,0,0);
		}

		static enum stages{ S1,S2,S3}next_stage=S1;
		if (TPM2->SC & TPM_SC_TOF_MASK){	//count overflow
			ctr++;
			if (ctr>=250){  //wait for 5secs
				ctr=0;
				switch (next_stage){
				case S1:

					TPM2->CONTROLS[0].CnV =0x6CCD; //half power
					next_stage=S2;
					break;
				case S2:
					TPM2->CONTROLS[0].CnV =0x9CCD; //3/4 power
					next_stage=S3;
					break;

				case S3:
					TPM2->CONTROLS[0].CnV =0xCCCD; // full power
					next_stage=S1;
					break;
				default:
					TPM2->CONTROLS[0].CnV =0xCCCD;
					next_stage=S1;
					break;
				}

			}

		}
}


void init_pins(void){
	//Clock gate port A  --- motors
	SIM->SCGC5 |= SIM_SCGC5_PORTA(1);
	PORTA->PCR[RX] &= ~PORT_PCR_MUX_MASK;  	//clear mux
	PORTA->PCR[RX] |=  PORT_PCR_MUX(2); 	//set for UART0 RX
	PORTA->PCR[TX] &= ~PORT_PCR_MUX_MASK;	//clear
	PORTA->PCR[TX] |=  PORT_PCR_MUX(2); 	//set for UART0 TX

	SIM->SCGC5 |=SIM_SCGC5_PORTA_MASK;
	PORTA->PCR[TRIGGER] &=~PORT_PCR_MUX_MASK;
	PORTA->PCR[TRIGGER] |=PORT_PCR_MUX(3);

	SIM->SCGC5 |=SIM_SCGC5_PORTA_MASK;
	PORTA->PCR[ECHO] &=~PORT_PCR_MUX_MASK;
	PORTA->PCR[ECHO] |=PORT_PCR_MUX(3);

}


void Init_UART(void){
	//select clock for uart0 (disabled by default), MCGFLLCLK/system clk as UART0 clock
	SIM->SOPT2 |= SIM_SOPT2_UART0SRC(1);
	// clock gate UART0
	SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;		//clock gate UART0
	//compute set baud rate (SBR), choosing baud rate of 9600 for BT
	uint8_t sbr = (uint16_t)((SYS_CLOCK)/((OSR+1) *BAUD_RATE ));	//default OSR is 15, 	sbr=136.5 if SYS_CLOCK =20971520u
	//BAUD_RATE = (SYS_CLOCK)/[(OSR+1) * BR]
	//UART0->BDH |=((sbr>>8) & 0x1F);	//generic. set only bottom 5 bits
	UART0->BDH =0;			//0x0 for this calculation, other fields are default 0.
	UART0->BDL=sbr;			//0x88 (or 136) for this calculation
	// Rx Interrupt enabled, only RX
	UART0->C2  |= UART_C2_RIE_MASK | UART_C2_RE_MASK ;// | UART_C2_TE_MASK ;
	//note: default is 8N1 if uart0->C1=0


	NVIC_SetPriority(UART0_IRQn, 3);
	NVIC_ClearPendingIRQ(UART0_IRQn);
	NVIC_EnableIRQ(UART0_IRQn);
}



void ultrasonic_timer(){	//use channel0 for signal generation,channel 1 for input capture
	//Clock gate
	SIM->SCGC6 |=SIM_SCGC6_TPM1_MASK;	//***TPM1 channel 0
	//Select clock source in SIM_SOPT		//system clock
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1) ;
	//Timeout is in 38ms
	//TPM1->MOD=0x6666;		//160ms timeout  (160^-3*20971520/128 =26,214   = 0x6666	//both work
	TPM1->MOD= 8192;	//50ms =8192 or 0x2000

	//Channel 0  PWM: MSB-A==10 ELSB-A  10
	TPM1->CONTROLS[0].CnSC |= TPM_CnSC_MSB(1) |TPM_CnSC_ELSB(1)  ; //PWM output, interrupts not needed.
	TPM1->CONTROLS[0].CnV |= 3 ;		//For trigger of 20us, CnV=3.2  10us is 1.6 Also tested with CnV=100

	//input capture: MSB-A==00 ELSB-A  11
	TPM1->CONTROLS[1].CnSC |= TPM_CnSC_ELSA(1) |TPM_CnSC_ELSB(1)  ; //rising & falling edge
	TPM1->CONTROLS[1].CnSC |= TPM_CnSC_CHF_MASK | TPM_CnSC_CHIE_MASK ;  //enable interrupts
	TPM1->SC |=  TPM_SC_TOF_MASK | TPM_SC_PS(7) | TPM_SC_TOIE_MASK  ;
	TPM1->SC |= TPM_SC_CMOD(1); //enable internal clock to run

	NVIC_ClearPendingIRQ(TPM1_IRQn);
	NVIC_SetPriority(TPM1_IRQn, 3);
	NVIC_EnableIRQ(TPM1_IRQn);
}


void Init_RGB_LEDs(void) {
	// Enable clock to ports B and D
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTD_MASK;

	// Make 3 pins GPIO
	PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1);
	PORTB->PCR[GREEN_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[GREEN_LED] |= PORT_PCR_MUX(1);
	PORTD->PCR[BLUE_LED] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[BLUE_LED] |= PORT_PCR_MUX(1);

	PORTD->PCR[BUZZER] &= ~PORT_PCR_MUX_MASK;	//Clear mux
	PORTD->PCR[BUZZER] |= PORT_PCR_MUX(1);

	// Set ports to outputs
	PTB->PDDR |= MASK(RED_LED) | MASK(GREEN_LED);
	PTD->PDDR |= MASK(BLUE_LED) | MASK(BUZZER);

	PTB->PSOR |= MASK(RED_LED) | MASK(GREEN_LED);
	PTD->PSOR |= MASK(BLUE_LED);
	PTD->PCOR |= MASK(BUZZER);



	SysTick->LOAD = (20971520u/1000u)-1 ;  //configure for every milli sec restart.
		    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk |
		    		SysTick_CTRL_ENABLE_Msk |SysTick_CTRL_TICKINT_Msk;
}


void self_test(){
	enum clockState{S1, S2, S3, S4, S5, S6, S7, S8};
	unsigned long current_time =0u;
	static unsigned long last_run=0u;
	static enum clockState next_state = S1;
	unsigned long interval = 5000;

	switch(next_state){
	case S1: movement_control(1,0,0,0); next_state = S2; break;
	case S2: current_time= millis();
		if((current_time-last_run) >= interval){
			last_run=current_time;
			next_state= S3;
		}break;
	case S3: movement_control(0,0,0,0); movement_control(0,1,0,0); next_state = S4; break;
	case S4: current_time= millis();
		if((current_time-last_run) >= interval){
			last_run=current_time;
			next_state= S5;
		}break;
	case S5: movement_control(0,0,0,0); movement_control(0,0,1,0); next_state = S6; break;
		case S6: current_time= millis();
			if((current_time-last_run) >= interval){
				last_run=current_time;
				next_state= S7;
			}break;
		case S7: movement_control(0,0,0,0); movement_control(0,0,0,1); next_state = S8; break;
		case S8: current_time= millis();
			if((current_time-last_run) >= interval){
				last_run=current_time;
				next_state= S1;
			}break;

		default: next_state = S1; break;

	}

}

void buzzer(int repeat){
	enum clockState{S1, S2, S3, S4};
			unsigned long current_time =0u;
			static unsigned long last_run=0u;
			static enum clockState nextState = S1;

			if(!repeat){
				if(repeat_check <2 ){
			switch(nextState){
				case S1:PTD->PSOR = MASK(BUZZER); nextState = S2; break;
				case S2: current_time= millis();
						if((current_time-last_run) >= 500){
							last_run=current_time;
							nextState= S3;
						}break;

				case S3: PTD->PCOR = MASK(BUZZER); nextState = S4; break;
				case S4: current_time= millis();
						if((current_time-last_run) >= 500){
							last_run=current_time;
							nextState= S1;
							repeat_check ++;
						}break;
				default: nextState = S1; break;
			}
			}
	}else {
		switch(nextState){
						case S1:PTD->PSOR = MASK(BUZZER); nextState = S2; break;
						case S2: current_time= millis();
								if((current_time-last_run) >= 500){
									last_run=current_time;
									nextState= S3;
								}break;

						case S3: PTD->PCOR = MASK(BUZZER); nextState = S4; break;
						case S4: current_time= millis();
								if((current_time-last_run) >= 500){
									last_run=current_time;
									nextState= S1;
								}break;
						default: nextState = S1; break;
					}
	}
}

void movement_control(unsigned int forward, unsigned int backward, unsigned int left, unsigned int right) {

	if (forward) {
		if(g_dist <= 250){
			buzzer(0);

			PTB->PCOR |=MASK(W1);
			PTB->PCOR |=MASK(W2);
			PTB->PCOR |=MASK(X1);
			PTB->PCOR |=MASK(X2);
			}else{
			PTB->PCOR |= MASK(BACK_LIT);
			PTB->PSOR |= MASK(FRONT_LIT);
			PTD->PCOR = MASK(BUZZER);
			PTB->PCOR |=MASK(W1);
			PTB->PSOR |=MASK(W2);
			PTB->PCOR |=MASK(X1);
			PTB->PSOR |=MASK(X2);
			if(repeat_check == 2)
			repeat_check =  repeat_check-3;
		}}
	if (backward) {

		buzzer(1);
		PTB->PSOR |=MASK(W1);
		PTB->PSOR |= MASK(BACK_LIT);
		PTB->PCOR |= MASK(FRONT_LIT);
		PTB->PCOR |=MASK(W2);
		PTB->PSOR |=MASK(X1);
		PTB->PCOR |=MASK(X2);
	}
	if (left) {
		PTB->PCOR |= MASK(BACK_LIT);
		PTB->PCOR |= MASK(FRONT_LIT);
		PTD->PCOR = MASK(BUZZER);
		PTB->PCOR |=MASK(W1);
		PTB->PCOR |=MASK(W2);
		PTB->PCOR |=MASK(X1);
		PTB->PSOR |=MASK(X2);
	}

	if(right) {
		PTB->PCOR |= MASK(BACK_LIT);
		PTB->PCOR |= MASK(FRONT_LIT);
		PTD->PCOR = MASK(BUZZER);
		PTB->PCOR |=MASK(W1);
		PTB->PSOR |=MASK(W2);
		PTB->PCOR |=MASK(X1);
		PTB->PCOR |=MASK(X2);
	}

	if(!right && !left && !forward && !backward) {
		    PTB->PCOR |= MASK(BACK_LIT);
			PTB->PCOR |= MASK(FRONT_LIT);
		    PTD->PCOR = MASK(BUZZER);
			PTB->PCOR |=MASK(W1);
			PTB->PCOR |=MASK(W2);
			PTB->PCOR |=MASK(X1);
			PTB->PCOR |=MASK(X2);
		}
}

void control_Motor(void){
	switch (rxChar){
		case 'f': movement_control(1,0,0,0);
				break;
		case 'b':  movement_control(0,1,0,0);
				break;
		case 'l':  movement_control(0,0,1,0);
				break;
		case 'r':  movement_control(0,0,0,1);
				break;
		case 's':  movement_control(0,0,0,0);
						break;
		case 'z':  self_test();
				break;
		case 'p': speed = 1; break ;
		case 'k': speed = 0; break;
		default:
			  movement_control(1,0,0,0); break;
	}
}

void TPM1_IRQHandler(){
	static int ctr=0;
	static unsigned int previous=0;
	unsigned int current=0;
	static unsigned int interval=0;		//make this global so another function can use it
		if (TPM1->STATUS & TPM_STATUS_CH1F_MASK){		//any input capture?
			current=TPM1->CONTROLS[1].CnV;
			current |= (ctr <<16);// add the no. of overflows. Each ctr tick is 2^16
			interval = current-previous;
			previous=current;
			TPM1->CONTROLS[1].CnSC |=TPM_CnSC_CHF_MASK;  	//clear flag
	}

	if (TPM1->SC & TPM_SC_TOF_MASK){
		ctr++;		//a timer overflow occurRED_LED.
		TPM1->SC |= TPM_SC_TOF_MASK ; //clear the interrupt
		if (!( ctr %10)){		//check every ___s
			 g_dist=interval;		//needed to multiply bt 1.047. Just aproximating



			 g_dist=interval;		//needed to multiply bt 1.047. Just aproximating

		}
	}
}



void flashRRRBBB(void){
	static enum green_states{S1, S2, S3, S4, S5,S6,S7,S8,S9,S10,S11,S12} next_state = S1;
	unsigned int current_time;
	static unsigned long last_run=0u;
	unsigned int interval = poten_val;

	current_time= millis();	//current time
	switch (next_state){
	case S1: //first ON
		PTB->PCOR |= MASK(RED_LED);  //turn on, move to default to run only once
		if((current_time-last_run) >= interval){
			last_run=current_time;
			PTB->PSOR |= MASK(RED_LED);  //turn off
			next_state= S2;
		}
		break;
	case S2:
		if((current_time-last_run) >= interval){
			last_run=current_time;
			PTB->PCOR |= MASK(RED_LED);  //turn on
			next_state= S3;
		}
		break;
	case S3:
		if((current_time-last_run) >= interval){
			last_run=current_time;
			PTB->PSOR |= MASK(RED_LED);  //turn off
			next_state= S4;
		}
		break;
	case S4: //off
		if((current_time-last_run) >= interval){
			last_run=current_time;
			PTB->PCOR |= MASK(RED_LED);  //turn on
			next_state= S5;
		}
		break;

	case S5: //first ON
			if((current_time-last_run) >= interval){
				last_run=current_time;
				PTB->PSOR |= MASK(RED_LED);  //turn off
				next_state= S6;
			}
			break;
		case S6:
			if((current_time-last_run) >= interval){
				last_run=current_time;
				PTD->PCOR |= MASK(BLUE_LED);  //turn on
				next_state= S7;
			}
			break;
		case S7:
			if((current_time-last_run) >= interval){
				last_run=current_time;
				PTD->PSOR |= MASK(BLUE_LED);  //turn off
				next_state= S8;
			}
			break;
		case S8: //off
			if((current_time-last_run) >= interval){
				last_run=current_time;
				PTD->PCOR |= MASK(BLUE_LED);
				next_state= S9;
			}
			break;

		case S9: //first ON
				if((current_time-last_run) >= interval){
					last_run=current_time;
					PTD->PSOR |= MASK(BLUE_LED);  //turn off
					next_state= S10;
				}
				break;
			case S10:
				if((current_time-last_run) >= interval){
					last_run=current_time;
					PTD->PCOR |= MASK(BLUE_LED);  //turn on
					next_state= S11;
				}
				break;
			case S11:
				if((current_time-last_run) >= interval){
					last_run=current_time;
					PTD->PSOR |= MASK(BLUE_LED);  //turn off
					next_state= S12;
				}
				break;
			case S12: //off
				if((current_time-last_run) >= interval){
					last_run=current_time;
					next_state= S1;
				}
				break;

	default:
		PTB->PCOR = MASK(RED_LED);  //turn on		//added but not expected to run yet.
		break;

	}

}



void SysTick_Handler(void){
	counter++;
}
void UART0_IRQHandler(void){
	uint8_t ch;

	if (UART0->S1 & (UART_S1_OR_MASK |UART_S1_NF_MASK |
		UART_S1_FE_MASK | UART_S1_PF_MASK)) {
			// clear the error flags
			UART0->S1 |= UART0_S1_OR_MASK | UART0_S1_NF_MASK |
			UART0_S1_FE_MASK | UART0_S1_PF_MASK;
			// read the data register to clear RDRF
			ch = UART0->D;
	}
	if (UART0->S1 & UART0_S1_RDRF_MASK) {
		// received a character
		ch = UART0->D;
		rxChar=ch;		//to enable me take some action
			//deal with input here or in main.
	}
}
unsigned long  millis(void){
	return (unsigned long)counter;
}


void TPM2_IRQHandler(){
 if(speed){
	slow(W1);
 TPM2->SC |= TPM_SC_TOF_MASK ; //clear the interrupt
}
}

void TPM0_IRQHandler(){
 if(speed){
	slow(W1);
 TPM0->SC |= TPM_SC_TOF_MASK ; //clear the interrupt
}
}

