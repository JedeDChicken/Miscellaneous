/**
 * @file	main.c
 * @brief	Week 7: Universal Asynchronous Receiver/Transmitter
 *
 * @author	Alberto de Villa <abdevilla@up.edu.ph>
 * @date	29 Nov 2022
 * @copyright
 * Copyright (C) 2022. This source code was created as part of the author's
 * official duties with the Electrical and Electronics Engineering Institute,
 * University of the Philippines <https://eee.upd.edu.ph>
 * 
 * TODO: Modify the information here as appropriate!
 */

/*
 * System configuration/build:
 * 	- Clock source == HSI (~16 MHz)
 * 		- No AHB & APB1/2 prescaling
 *	- Inputs:
 * 		- Active-LO NO-wired pushbutton @ PC13
 * 		- USART Input @ PA3 (USART2_RX)
 * 	- Outputs:
 * 		- Active-HI LED @ PA5
 *		- USART Output @ PA2 (USART2_TX)
 *
 * NOTE: This project uses the CMSIS standard for ARM-based microcontrollers;
 * 	 This allows register names to be used without regard to the exact
 * 	 addresses.
 */

#include "usart.h"
#include <stdint.h>	// C standard header; contains uint32_t, for example
#include <stm32f4xx.h>	// Header for the specific device family

#include <stdio.h>	// Needed for snprintf()
#include <string.h>	// Needed for memcmp()

#include <math.h>
unsigned int key_pressed = 0;
unsigned int counter = 0;
unsigned int counter2 = 0;
unsigned int FIFO_idx = 0;
unsigned int FIFO_idx2 = 0;
unsigned int FIFO_idxsub = 5;
int FIFO[5];

////////////////////////////////////////////////////////////////////////////

/*
 * IRQ data shared between the handlers and main()
 *
 * As asynchronous access is possible, all members are declared volatile.
 */
volatile struct
{
	/*
	 * If set, the button has been pressed.
	 * 
	 * This should be cleared in main().
	 */
	unsigned int pressed;
	
	/*
	 * Number of system ticks elapsed
	 */
	unsigned int nr_tick;
	
} irq_data;

/*
 * Handler for the interrupt
 *
 * This function name is special -- this name is used by the startup code (*.s)
 * to indicate the handler for this interrupt vector.
 */
void EXTI15_10_IRQHandler(void)
{
	/*
	 * The hardware setup has PC13 being active-low. This must be taken
	 * into consideration to maintain logical consistency with the
	 * rest of the code.
	 */
	if (!(GPIOC->IDR & 0x2000))
		irq_data.pressed = 1;

	// Re-enable reception of interrupts on this line.
	EXTI->PR = (1 << 13);
}

// Handler for the system tick
void SysTick_Handler(void)
{
	irq_data.nr_tick += 1;
	SysTick->VAL = 0;
}

////////////////////////////////////////////////////////////////////////////

// Function to initialize the system; called only once on device reset
static void do_sys_config(void)
{
	// See the top of this file for assumptions used in this initialization.

	
	////////////////////////////////////////////////////////////////////

	RCC->AHB1ENR |= (1 << 0);	// Enable GPIOA

	GPIOA->MODER &= ~(0b11 << 10);	// Set PA5 as input...
	GPIOA->MODER |=  (0b10 << 10);	// ... then set it as alternate function.
	GPIOA->OTYPER &= ~(1 << 5);	// PA5 = push-pull output
	GPIOA->OSPEEDR &= ~(0b11 << 10);	// Fast mode (needed for PWM)
	GPIOA->OSPEEDR |=  (0b10 << 10);

	/*
	 * For PA5 -- where the LED on the Nucleo board is -- only TIM2_CH1
	 * is PWM-capable, per the *device* datasheet. This corresponds to
	 * AF01.
	 */
	GPIOA->AFR[0] &= ~(0x00F00000);	// TIM2_CH1 on PA5 is AF01
	GPIOA->AFR[0] |=  (0x00100000);

	////////////////////////////////////////////////////////////////////

	/*
	 * In this setup, TIM2 is used for brightness control (see reason
	 * above).
	 *
	 * The internal clock for TIM2 is the same as the APB1 clock;
	 * in turn, this clock is assumed the same as the system (AHB) clock
	 * without prescaling, which results to the same frequency as HSI
	 * (~16 MHz).
	 */
	RCC->APB1ENR	|= (1 << 0);	// Enable TIM2

	/*
	 * Classic PWM corresponds to the following:
	 * 	- Edge-aligned	(CMS  = CR1[6:5] = 0b00)
	 * 	- Upcounting	(DIR  = CR1[4:4] = 0)
	 *	- Repetitive	(OPM  = CR1[3:3] = 0)
	 * 	- PWM Mode #1	(CCxS[1:0] = 0b00, OCMx = 0b110)
	 * 		- These are in CCMRy; y=1 for CH1 & CH2, and y=2 for
	 * 		  CH3 & CH4.
	 */
	TIM2->CR1 &= ~(0b1111 << 0);
	TIM2->CR1 &= ~(1 << 0);		// Make sure the timer is off
	TIM2->CR1 |=  (1 << 7);		// Preload ARR (required for PWM)
	TIM2->CCMR1 = 0x0068;		// Channel 1 (TIM2_CH1)

	/*
	 * Per the Nyquist sampling theorem (from EEE 147), to appear
	 * continuous the PWM frequency must be more than twice the sampling
	 * frequency. The human eye (the sampler) can usually go up to 24Hz;
	 * some, up to 40-60 Hz. To cover all bases, let's use 500 Hz.
	 *
	 * In PWM mode, the effective frequency changes to
	 *
	 * (f_clk) / (ARR*(PSC+1))
	 *
	 * since each period must be able to span all values on the interval
	 * [0, ARR). For obvious reasons, ARR must be at least equal to one.
	 */
	TIM2->ARR	= 100;		// Integer percentages; interval = [0,100]
	TIM2->PSC	= (320 - 1);	// (16MHz) / (ARR*(TIM2_PSC + 1)) = 500 Hz

	// Let main() set the duty cycle. We initialize at zero.
	TIM2->CCR1	= 0;

	/*
	 * The LED is active-HI; thus, its polarity bit must be cleared. Also,
	 * the OCEN bit must be enabled to actually output the PWM signal onto
	 * the port pin.
	 */
	TIM2->CCER	= 0x0001;

	////////////////////////////////////////////////////////////////////

	// Pushbutton configuration
	RCC->AHB1ENR |= (1 << 2);	// Enable GPIOC

	GPIOC->MODER &= ~(0b11 << 26);	// Set PC13 as input...
	GPIOC->PUPDR &= ~(0b11 << 26);	// ... without any pull-up/pull-down (provided externally).

	////////////////////////////////////////////////////////////////////
	
	/*
	 * Enable the system-configuration controller. If disabled, interrupt
	 * settings cannot be configured.
	 */
	RCC->APB2ENR |= (1 << 14);

	/*
	 * SYSCFG_EXTICR is a set of 4 registers, each controlling 4 external-
	 * interrupt lines. Within each register, 4 bits are used to select
	 * the source connected to each line:
	 * 	0b0000 = Port A
	 * 	0b0001 = Port B
	 * 	...
	 * 	0b0111 = Port H
	 *
	 * For the first EXTICR register:
	 *	Bits 0-3   correspond to Line 0;
	 * 	Bits 4-7   correspond to Line 1;
	 *	Bits 8-11  correspond to Line 2; and
	 * 	Bits 12-15 correspond to Line 3.
	 *
	 * The 2nd EXTICR is for Lines 4-7; and so on. Also, the line numbers
	 * map 1-to-1 to the corresponding bit in each port. Thus, for example,
	 * a setting of EXTICR2[11:8] == 0b0011 causes PD6 to be tied to
	 * Line 6.
	 *
	 * For this system, PC13 would end up on Line 13; thus, the
	 * corresponding setting is EXTICR4[7:4] == 0b0010. Before we set it,
	 * mask the corresponding interrupt line.
	 */
	EXTI->IMR &= ~(1 << 13);		// Mask the interrupt
	SYSCFG->EXTICR[3] &= (0b1111 << 4);	// Select PC13 for Line 13
	SYSCFG->EXTICR[3] |= (0b0010 << 4);

	/*
	 * Per the hardware configuration, pressing the button causes a
	 * falling-edge event to be triggered, and a rising-edge on release.
	 * Since we are only concerned with presses, just don't trigger on
	 * releases.
	 */
	EXTI->RTSR &= ~(1 << 13);
	EXTI->FTSR |=  (1 << 13);

	/*
	 * Nothing more from the SCC, disable it to prevent accidental
	 * remapping of the interrupt lines.
	 */
	RCC->APB2ENR &= ~(1 << 14);

	////////////////////////////////////////////////////////////////////
	
	/*
	 *
	 * Per the STM32F411xC/E family datasheet, the handler for EXTI_15_10
	 * is at 40.
	 *
	 * Per the STM32F4 architecture datasheet, the NVIC IP registers
	 * each contain 4 interrupt indices; 0-3 for IPR[0], 4-7 for IPR[1],
	 * and so on. Each index has 8 bits denoting the priority in the top
	 * 4 bits; lower number = higher priority.
	 *
	 * Position 40 in the NVIC table would be at IPR[10][7:0]; or,
	 * alternatively, just IP[40].
	 */
	NVIC->IP[40] = (1 << 4);
	NVIC->IP[6]  = (0b1111 << 4);	// SysTick; make it least-priority
	
	/*
	 * Per the STM32F4 architecture datasheet, the NVIC ISER/ICER registers
	 * each contain 32 interrupt indices; 0-31 for I{S/C}ER[0], 32-63 for
	 * I{S/C}ER[1], and so on -- one bit per line.
	 *
	 * ISER is written '1' to enable a particular interrupt, while ICER is
	 * written '1' to disable the same interrupt. Writing '0' has no
	 * effect.
	 *
	 * Position 25 in the NVIC table would be at I{S/C}ER[0][25:25]; while
	 * position 27 would be at I{S/C}ER[0][27:27].
	 */
	NVIC->ISER[0] = (1 << 6);	// Note: Writing '0' is a no-op
	NVIC->ISER[1] = (1 << 8);	// Note: Writing '0' is a no-op
	EXTI->IMR |= (1 << 13);		// Unmask the interrupt on Line 13
	TIM2->EGR |= (1 << 0);		// Trigger an update on TIM2
	TIM2->CR1 |= (1 << 0);		// Activate both timers
	irq_data.pressed = 0;
	
	/*
	 * Enable tick counting; the idea is to allow main() to perform
	 * periodic tasks.
	 */
	SysTick->LOAD = (20000-1);	// Target is 100 Hz with 2MHz clock
	SysTick->VAL  = 0;
	SysTick->CTRL &= ~(1 << 2);	// Clock base = 16MHz / 8 = 2MHz
	SysTick->CTRL &= ~(1 << 16);
	SysTick->CTRL |= (0b11 << 0);	// Enable the tick
	
	// Do the initialization of USART last.
	usart2_init();
}

/////////////////////////////////////////////////////////////////////////////

/*
 * Copyright message printed upon reset
 * 
 * Displaying author information is optional; but as always, must be present
 * as comments at the top of the source file for copyright purposes.
 * 
 * FIXME: Modify this prompt message to account for additional instructions.
 */
static const char banner_msg[] =
"\033[0m\033[2J\033[1;1H"
"-------------------------------------------------------------------\r\n"
"EEE 158: Electrical and Electronics Engineering Laboratory V\r\n"
"         Academic Year 2022-2023, Semester 1\r\n"
"\r\n"
"Week 07: (Universal Synchronous/Asynchronous Receiver/Transmitter)\r\n"
"\r\n"				// [BEGIN] Author info
"Name:    <your name here>\r\n"
"Section: <your section here>\r\n"
"Date:    <date here>\r\n"	// [END] Author info
"-------------------------------------------------------------------\r\n"
"\r\n"
"Press the following keys to control the on-board LED:\r\n"
"\t- <Up>/W/w      Turn ON the LED\r\n"
"\t- <Down>/S/s    Turn OFF the LED\r\n"
"\t- <Space>       Toggle the LED; no effect while blinking\r\n"
"\t- <Left>/A/a    Dim the LED, if ON\r\n"
"\t- <Right>/D/d   Brighten the LED, if ON\r\n"
"\t- <F5>          LED blinks roughly 5x/s\r\n"
"\t- <F6>          LED blinks 2x/s\r\n"
"\t- <F7>          LED blinks 1x/2s\r\n"
"\t- <F8>          LED blinks 1x/s\r\n"
"\r\n";

/*
 * For readability reasons, the ANSI control sequences and the data string are
 * kept separate; the compiler treats these as one continuous string.
 * 
 * cbstr_0 is the message shown upon initialization. These fragments do the
 * following:
 * 	- Moves the cursor to Line 19, Column 1 ("\033[19;1H")
 * 	- Sets the color scheme to default ("\033[0m")
 * 	- Clears the contents from the cursor (inclusive) to the end of the screen ("\033[0J")
 * 	- Shows the actual message, with its own color codes if any
 * 	- Sets the color scheme to default ("\033[0m")
 * 
 * FIXME: Update "Line 19" to match the new end of the banner after additional
 *        instructions are added.
 */
static const char cbstr_0[] = "\033[23;1H\033[0m\033[0J" "Electrical" "\033[0m";
static const char cbstr_1[] = "\033[23;1H\033[41m\033[37m\033[0J" "and" "\033[41m\033[37m";
static const char cbstr_2[] = "\033[23;1H\033[42m\033[30m\033[0J" "Electronics" "\033[42m\033[30m";
static const char cbstr_3[] = "\033[23;1H\033[44m\033[37m\033[0J" "Engineering" "\033[44m\033[37m";
static const char cbstr_4[] = "\033[23;1H\033[46m\033[30m\033[0J" "Institute" "\033[46m\033[30m";
static const char cbstr_5[] = "\033[23;1H\033[45m\033[0J" "UP" "\033[45m";
static const char cbstr_6[] = "\033[23;1H\033[43m\033[0J" "Diliman" "\033[43m";

/*
 * Array of strings to show up
 * 
 * TODO: For additional messages, define a character array like cbstr_0, and
 *       add a new entry in this array describing the new string
 */
struct bstr_desc_type {
	const char *str;
	unsigned int size;
};
static const struct bstr_desc_type cbstr_desc[] = {
	{
		.str  = cbstr_0,
		.size = sizeof(cbstr_0)-1
	},
	// TODO: Add more entries here
	{
		.str  = cbstr_1,
		.size = sizeof(cbstr_1)-1
	},
	{
		.str  = cbstr_2,
		.size = sizeof(cbstr_2)-1
	},
	{
		.str  = cbstr_3,
		.size = sizeof(cbstr_3)-1
	},
	{
		.str  = cbstr_4,
		.size = sizeof(cbstr_4)-1
	},
	{
		.str  = cbstr_5,
		.size = sizeof(cbstr_5)-1
	},
	{
		.str  = cbstr_6,
		.size = sizeof(cbstr_6)-1
	}
};


// The heart of the program
int main(void)
{
	unsigned int usart_evt = 0;	// USART event, also used for scratchwork outside
	unsigned int led_brightness = 0, led_on = 0, nr_tick = 0;
	
	// Variable to avoid changing colors too frequently
	unsigned int msg_index = 0;
	
	// Buffer for transmitting data
//	char txb_data[48];
	unsigned int txb_size = 0;
	const char *txb_ptr = 0;
	
	/*
	 * Buffer for storing data from the USART; necessary to properly parse
	 * ESC sequences
	 */
	char		rxb_data[64];
	unsigned int	rxb_idx  = 0;
	unsigned int	rxb_size = 0;
	
	// Configure the system
	do_sys_config();
		
	// Print the banner first
	txb_ptr  = banner_msg;
	txb_size = sizeof(banner_msg) - 1;
	for (; txb_size > 0; ++txb_ptr, --txb_size) {
		usart2_tx_send_char(*txb_ptr);
	}
	
	// Print the first message
	msg_index = 0;
	txb_ptr  = cbstr_desc[msg_index].str;
	txb_size = cbstr_desc[msg_index].size;
	for (; txb_size > 0; ++txb_ptr, --txb_size) {
		usart2_tx_send_char(*txb_ptr);
	}
	txb_ptr = 0;
	
	/*
	 * Microcontroller main()'s are expected to never return; hence, the
	 * infinite loop.
	 */
	led_brightness = 100;		// Start at dim brightness
	led_on = 0;			// LED initially OFF
	for (;;) {
		
		/////////////////////////////////////////////////////////////
		
		// Check for any data received via USART2.
		do {
			usart_evt = usart2_rx_try_get_event();
			if (usart_evt == USART_RX_EVT_NONE)
				// Nothing to do here
				break;
			
			if (usart_evt & USART_RX_EVT_ERR_MASK)
				// Data has errors; ignore
				break;
			
			/*
			 * [1] If an IDLE is detected, update the size.
			 *     
			 * [2] If no data is present, we're done.
			 */
			if (usart_evt & USART_RX_EVT_IDLE) {
				rxb_size = rxb_idx;
				break;
			} else if (!(usart_evt & USART_RX_EVT_DATA_VALID)) {
				break;
			}
			
			// Store the data
			if (rxb_idx >= sizeof(rxb_data)) {
				rxb_size = rxb_idx;
				break;
			}
			rxb_data[rxb_idx++] = usart_evt & USART_RX_EVT_DATA_MASK;
			break;
		} while (0);
		
		// Clear out any backlogs in the TX queue
		if (txb_size > 0 && txb_ptr) {
			if (usart2_tx_try_send_char(*txb_ptr)) {
				// Successfully enqueued
				--txb_size;
				++txb_ptr;
				if (txb_size == 0)
					txb_ptr = 0;
			}
		}
		
		/////////////////////////////////////////////////////////////
		
		/*
		 * Since we are operating on a per-character basis, if rxb_size
		 * is non-zero we immediately process the data.
		 */
		if (rxb_size > 0) {
			if (rxb_data[0] == 0x1B) {

/*				 * ESC sequence
				 *
				 * The rest of the buffer must be examined to
				 * determine what this code is.
				 *
				 * TODO: If applicable, place here escape
				 *       codes for most other keys, like
				 *       Alt+ESC.
				 *
				 * NOTE: The vast majority of escape sequences
				 *       sent by modern PCs follow either the
				 *       VT1xx or xterm standards.*/

/*				if (rxb_size == 3) {
					if (!memcmp(rxb_data+1, "[A", 2)) {
						// Up arrow pressed
						led_on = 1;
						key_pressed = 0;
					} else if (!memcmp(rxb_data+1, "[B", 2)) {
						// Down arrow pressed
						led_on = 0;
						key_pressed = 0;
					} else if (!memcmp(rxb_data+1, "[C", 2)) {
						// Right arrow pressed
						led_brightness = 100;
					} else if (!memcmp(rxb_data+1, "[D", 2)) {
						// Left arrow pressed
						led_brightness = 20;
					}
				}
// 				else if (rxb_size == 7 && !memcmp(rxb_data+1, "[24;3~", 6)) {
// 					// Alt+F12 pressed
// 				}
				else if (rxb_size == 5) {
					if (!memcmp(rxb_data+1, "[15~", 4)) {
						//F5
					}
					else if (!memcmp(rxb_data+1, "[17~", 4)) {
						//F6
						key_pressed = 2;
					}
					else if (!memcmp(rxb_data+1, "[18~", 4)) {
						//F7
						key_pressed = 3;
					}
					else if (!memcmp(rxb_data+1, "[19~", 4)) {
						//F8
						key_pressed = 4;
					}
				}*/
			}

			else {

/*				 * Just a plain character
				 * 
				 * TODO: If applicable, place here other
				 *       actions that use only plain
				 *       characters.*/
				if (FIFO_idx < 5) {
					switch (rxb_data[0]) {
						case 'A':
						case 'a':
							//
							FIFO[FIFO_idx] = 1;
							FIFO_idx += 1;
							break;
						case 'B':
						case 'b':
							FIFO[FIFO_idx] = 2;
							FIFO_idx += 1;
							break;
						case 'C':
						case 'c':
							FIFO[FIFO_idx] = 3;
							FIFO_idx += 1;
							break;
						case 'D':
						case 'd':
							FIFO[FIFO_idx] = 4;
							FIFO_idx += 1;
							break;
						case 'E':
						case 'e':
							FIFO[FIFO_idx] = 5;
							FIFO_idx += 1;
							break;
						case 'F':
						case 'f':
							//
							FIFO[FIFO_idx] = 6;
							FIFO_idx += 1;
							break;
						case 'G':
						case 'g':
							FIFO[FIFO_idx] = 7;
							FIFO_idx += 1;
							break;
						case 'H':
						case 'h':
							FIFO[FIFO_idx] = 8;
							FIFO_idx += 1;
							break;
						case 'I':
						case 'i':
							FIFO[FIFO_idx] = 9;
							FIFO_idx += 1;
							break;
						case 'J':
						case 'j':
							FIFO[FIFO_idx] = 10;
							FIFO_idx += 1;
							break;
						case 'K':
						case 'k':
							//
							FIFO[FIFO_idx] = 11;
							FIFO_idx += 1;
							break;
						case 'L':
						case 'l':
							FIFO[FIFO_idx] = 12;
							FIFO_idx += 1;
							break;
						case 'M':
						case 'm':
							FIFO[FIFO_idx] = 13;
							FIFO_idx += 1;
							break;
						case 'N':
						case 'n':
							FIFO[FIFO_idx] = 14;
							FIFO_idx += 1;
							break;
						case 'O':
						case 'o':
							FIFO[FIFO_idx] = 15;
							FIFO_idx += 1;
							break;
						case 'P':
						case 'p':
							//
							FIFO[FIFO_idx] = 16;
							FIFO_idx += 1;
							break;
						case 'Q':
						case 'q':
							FIFO[FIFO_idx] = 17;
							FIFO_idx += 1;
							break;
						case 'R':
						case 'r':
							FIFO[FIFO_idx] = 18;
							FIFO_idx += 1;
							break;
						case 'S':
						case 's':
							FIFO[FIFO_idx] = 19;
							FIFO_idx += 1;
							break;
						case 'T':
						case 't':
							FIFO[FIFO_idx] = 20;
							FIFO_idx += 1;
							break;
						case 'U':
						case 'u':
							//
							FIFO[FIFO_idx] = 21;
							FIFO_idx += 1;
							break;
						case 'V':
						case 'v':
							FIFO[FIFO_idx] = 22;
							FIFO_idx += 1;
							break;
						case 'W':
						case 'w':
							FIFO[FIFO_idx] = 23;
							FIFO_idx += 1;
							break;
						case 'X':
						case 'x':
							FIFO[FIFO_idx] = 24;
							FIFO_idx += 1;
							break;
						case 'Y':
						case 'y':
							FIFO[FIFO_idx] = 25;
							FIFO_idx += 1;
							break;
						case 'Z':
						case 'z':
							//
							FIFO[FIFO_idx] = 26;
							FIFO_idx += 1;
							break;

						case '1':
							FIFO[FIFO_idx] = 31;
							FIFO_idx += 1;
							break;
						case '2':
							FIFO[FIFO_idx] = 32;
							FIFO_idx += 1;
							break;
						case '3':
							FIFO[FIFO_idx] = 33;
							FIFO_idx += 1;
							break;
						case '4':
							FIFO[FIFO_idx] = 34;
							FIFO_idx += 1;
							break;
						case '5':
							//
							FIFO[FIFO_idx] = 35;
							FIFO_idx += 1;
							break;
						case '6':
							FIFO[FIFO_idx] = 36;
							FIFO_idx += 1;
							break;
						case '7':
							FIFO[FIFO_idx] = 37;
							FIFO_idx += 1;
							break;
						case '8':
							FIFO[FIFO_idx] = 38;
							FIFO_idx += 1;
							break;
						case '9':
							FIFO[FIFO_idx] = 39;
							FIFO_idx += 1;
							break;
						case '0':
							//
							FIFO[FIFO_idx] = 30;
							FIFO_idx += 1;
							break;

/*						case ' ':
							// Toggle the LED
							if ((led_on == 1) && (key_pressed == 0))
								led_on = 0;
							else if ((led_on == 0) && (key_pressed == 0))
								led_on = 1;
							break;*/
					}
				}
			}
			
			// "Clear" the buffer.
			rxb_size = rxb_idx = 0;
		}
		
/*		while (rxb_size < 5) {
			switch (rxb_data[rxb_idx]) {
				case 'A':
				case 'a':
				//
					rxb_data[rxb_idx] = 1;
					rxb_idx += 1;
					rxb_size += 1;
					break;
				case 'B':
				case 'b':
					rxb_data[rxb_idx] = 2;
					rxb_idx += 1;
					rxb_size += 1;
					break;
			}
		}*/

		/////////////////////////////////////////////////////////////
		
		/*
		 * Change the brightness, as appropriate.
		 * 
		 * NOTE: We can use 'usart_evt' as a scratch variable, since
		 *       its data is no longer relevant here.
		 */
		switch (led_on) {
			case 0:
				// Solid-OFF
				TIM2->CCR1 = 0;
				break;
			case 1:
				// Solid-ON
				TIM2->CCR1 = led_brightness;
				break;
		}
		
		/////////////////////////////////////////////////////////////
		
		// Handle the pushbutton press here
		if (irq_data.pressed) {
			irq_data.pressed = 0;
			
			// [BEGIN] User code
			/*
			 * TODO: Insert action(s) to be done when the button
			 *       is pressed
			 */
			// [END] User code

/*			if (counter2 > 100) {
				if (msg_index < 6) {
					msg_index += 1;
				}
				else {
					msg_index = 0;
				}
				txb_ptr  = cbstr_desc[msg_index].str;
				txb_size = cbstr_desc[msg_index].size;
				for (; txb_size > 0; ++txb_ptr, --txb_size) {
					usart2_tx_send_char(*txb_ptr);
				}
				txb_ptr = 0;
				counter2 = 0;
			}*/
		}
		
		/////////////////////////////////////////////////////////////
		
		/*
		 * Handle periodic ticks here
		 * 
		 * Based on the configuration for SysTick in do_sys_config(),
		 * ticks occur at a nominal rate of 100 Hz. irq_data.nr_tick
		 * can be greater than one, in case some events were missed.
		 */
		if ((nr_tick = irq_data.nr_tick) > 0) {
			irq_data.nr_tick = 0;
			
			// [BEGIN] User code
			/*
			 * TODO: Any tasks periodically done nominally every
			 *       10 ms (= 1/100Hz) should be added here.
			 */
			// [END] User code

			counter2 += 1;

/*			if (FIFO_idx == 5) {
				for (int i=0; i<5; i++) {
					if(FIFO[i] == 1) {						//A
						counter += 1;
						if (counter == 1) {					//Dot
							led_on = 1;
						}
						else if (counter == 50) {			//Space
							led_on = 0;
						}
						else if (counter == 100) {			//Dash
							led_on = 1;
						}
						else if (counter == 250) {			//Off
							led_on = 0;
						}
						else if (counter == 400) {
							counter = 0;
							FIFO_idx -= 1;
						}
						break;
					}
					else if (FIFO[i] == 2) {				//B
						counter += 1;
						if (counter == 1) {					//Dash
							led_on = 1;
						}
						else if ((counter == 150)) {		//Space
							led_on = 0;
						}
						else if ((counter == 200)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 250)) {		//Space
							led_on = 0;
						}
						else if ((counter == 300)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 350)) {		//Space
							led_on = 0;
						}
						else if ((counter == 400)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 450)) {		//Off
							led_on = 0;
						}
						else if (counter == 600) {
							counter = 0;
							FIFO_idx -= 1;
						}
						break;
					}
				}
			}*/

			if (FIFO_idx == 5) {
				if (FIFO_idxsub > 0) {
					key_pressed = FIFO[FIFO_idx2];

					if (key_pressed == 1) {					//A
						counter += 1;
						if (counter == 1) {					//Dot
							led_on = 1;
						}
						else if (counter == 50) {			//Space
							led_on = 0;
						}
						else if (counter == 100) {			//Dash
							led_on = 1;
						}
						else if (counter == 250) {			//Space
							led_on = 0;
						}
						else if (counter == 400) {			//Finish
							counter = 0;
							FIFO_idx2 += 1;
							FIFO_idxsub -= 1;
						}
					}

					else if (key_pressed == 2) {			//B
						counter += 1;
						if (counter == 1) {					//Dash
							led_on = 1;
						}
						else if ((counter == 150)) {		//Space
							led_on = 0;
						}
						else if ((counter == 200)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 250)) {		//Space
							led_on = 0;
						}
						else if ((counter == 300)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 350)) {		//Space
							led_on = 0;
						}
						else if ((counter == 400)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 450)) {		//Space
							led_on = 0;
						}
						else if ((counter == 600)) {		//Finish
							counter = 0;
							FIFO_idx2 += 1;
							FIFO_idxsub -= 1;
						}
					}

					else if (key_pressed == 3) {			//C
						counter += 1;
						if (counter == 1) {					//Dash
							led_on = 1;
						}
						else if ((counter == 150)) {		//Space
							led_on = 0;
						}
						else if ((counter == 200)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 250)) {		//Space
							led_on = 0;
						}
						else if ((counter == 300)) {		//Dash
							led_on = 1;
						}
						else if ((counter == 450)) {		//Space
							led_on = 0;
						}
						else if ((counter == 500)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 550)) {		//Space
							led_on = 0;
						}
						else if ((counter == 700)) {		//Finish
							counter = 0;
							FIFO_idx2 += 1;
							FIFO_idxsub -= 1;
						}
					}

					else if (key_pressed == 4) {			//D
						counter += 1;
						if (counter == 1) {					//Dash
							led_on = 1;
						}
						else if (counter == 150) {			//Space
							led_on = 0;
						}
						else if (counter == 200) {			//Dot
							led_on = 1;
						}
						else if (counter == 250) {			//Space
							led_on = 0;
						}
						else if (counter == 300) {			//Dot
							led_on = 1;
						}
						else if (counter == 350) {			//Space
							led_on = 0;
						}
						else if (counter == 500) {			//Finish
							counter = 0;
							FIFO_idx2 += 1;
							FIFO_idxsub -= 1;
						}
					}

					else if (key_pressed == 5) {			//E
						counter += 1;
						if (counter == 1) {					//Dot
							led_on = 1;
						}
						else if (counter == 50) {			//Space
							led_on = 0;
						}
						else if (counter == 200) {			//Finish
							counter = 0;
							FIFO_idx2 += 1;
							FIFO_idxsub -= 1;
						}
					}

					else if (key_pressed == 6) {			//F
						counter += 1;
						if (counter == 1) {					//Dot
							led_on = 1;
						}
						else if ((counter == 50)) {			//Space
							led_on = 0;
						}
						else if ((counter == 100)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 150)) {		//Space
							led_on = 0;
						}
						else if ((counter == 200)) {		//Dash
							led_on = 1;
						}
						else if ((counter == 350)) {		//Space
							led_on = 0;
						}
						else if ((counter == 400)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 450)) {		//Space
							led_on = 0;
						}
						else if ((counter == 600)) {		//Finish
							counter = 0;
							FIFO_idx2 += 1;
							FIFO_idxsub -= 1;
						}
					}

					else if (key_pressed == 7) {			//G
						counter += 1;
						if (counter == 1) {					//Dash
							led_on = 1;
						}
						else if (counter == 150) {			//Space
							led_on = 0;
						}
						else if (counter == 200) {			//Dash
							led_on = 1;
						}
						else if (counter == 350) {			//Space
							led_on = 0;
						}
						else if (counter == 400) {			//Dot
							led_on = 1;
						}
						else if (counter == 450) {			//Space
							led_on = 0;
						}
						else if (counter == 600) {			//Finish
							counter = 0;
							FIFO_idx2 += 1;
							FIFO_idxsub -= 1;
						}
					}

					else if (key_pressed == 8) {			//H
						counter += 1;
						if (counter == 1) {					//Dot
							led_on = 1;
						}
						else if ((counter == 50)) {			//Space
							led_on = 0;
						}
						else if ((counter == 100)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 150)) {		//Space
							led_on = 0;
						}
						else if ((counter == 200)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 250)) {		//Space
							led_on = 0;
						}
						else if ((counter == 300)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 350)) {		//Space
							led_on = 0;
						}
						else if ((counter == 500)) {		//Finish
							counter = 0;
							FIFO_idx2 += 1;
							FIFO_idxsub -= 1;
						}
					}

					else if (key_pressed == 9) {			//I
						counter += 1;
						if (counter == 1) {					//Dot
							led_on = 1;
						}
						else if (counter == 50) {			//Space
							led_on = 0;
						}
						else if (counter == 100) {			//Dot
							led_on = 1;
						}
						else if (counter == 150) {			//Space
							led_on = 0;
						}
						else if (counter == 300) {			//Finish
							counter = 0;
							FIFO_idx2 += 1;
							FIFO_idxsub -= 1;
						}
					}

					else if (key_pressed == 10) {			//J
						counter += 1;
						if (counter == 1) {					//Dot
							led_on = 1;
						}
						else if ((counter == 50)) {			//Space
							led_on = 0;
						}
						else if ((counter == 100)) {		//Dash
							led_on = 1;
						}
						else if ((counter == 250)) {		//Space
							led_on = 0;
						}
						else if ((counter == 300)) {		//Dash
							led_on = 1;
						}
						else if ((counter == 450)) {		//Space
							led_on = 0;
						}
						else if ((counter == 500)) {		//Dash
							led_on = 1;
						}
						else if ((counter == 550)) {		//Space
							led_on = 0;
						}
						else if ((counter == 700)) {		//Finish
							counter = 0;
							FIFO_idx2 += 1;
							FIFO_idxsub -= 1;
						}
					}

					else if (key_pressed == 11) {			//K
						counter += 1;
						if (counter == 1) {					//Dash
							led_on = 1;
						}
						else if (counter == 150) {			//Space
							led_on = 0;
						}
						else if (counter == 200) {			//Dot
							led_on = 1;
						}
						else if (counter == 250) {			//Space
							led_on = 0;
						}
						else if (counter == 300) {			//Dash
							led_on = 1;
						}
						else if (counter == 450) {			//Space
							led_on = 0;
						}
						else if (counter == 600) {			//Finish
							counter = 0;
							FIFO_idx2 += 1;
							FIFO_idxsub -= 1;
						}
					}

					else if (key_pressed == 12) {			//L
						counter += 1;
						if (counter == 1) {					//Dot
							led_on = 1;
						}
						else if ((counter == 50)) {			//Space
							led_on = 0;
						}
						else if ((counter == 100)) {		//Dash
							led_on = 1;
						}
						else if ((counter == 250)) {		//Space
							led_on = 0;
						}
						else if ((counter == 300)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 350)) {		//Space
							led_on = 0;
						}
						else if ((counter == 400)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 450)) {		//Space
							led_on = 0;
						}
						else if ((counter == 600)) {		//Finish
							counter = 0;
							FIFO_idx2 += 1;
							FIFO_idxsub -= 1;
						}
					}

					else if (key_pressed == 13) {			//M
						counter += 1;
						if (counter == 1) {					//Dash
							led_on = 1;
						}
						else if (counter == 150) {			//Space
							led_on = 0;
						}
						else if (counter == 200) {			//Dash
							led_on = 1;
						}
						else if (counter == 350) {			//Space
							led_on = 0;
						}
						else if (counter == 500) {			//Finish
							counter = 0;
							FIFO_idx2 += 1;
							FIFO_idxsub -= 1;
						}
					}

					else if (key_pressed == 14) {			//N
						counter += 1;
						if (counter == 1) {					//Dash
							led_on = 1;
						}
						else if (counter == 150) {			//Space
							led_on = 0;
						}
						else if (counter == 200) {			//Dot
							led_on = 1;
						}
						else if (counter == 250) {			//Space
							led_on = 0;
						}
						else if (counter == 400) {			//Finish
							counter = 0;
							FIFO_idx2 += 1;
							FIFO_idxsub -= 1;
						}
					}

					else if (key_pressed == 15) {			//O
						counter += 1;
						if (counter == 1) {					//Dash
							led_on = 1;
						}
						else if (counter == 150) {			//Space
							led_on = 0;
						}
						else if (counter == 200) {			//Dash
							led_on = 1;
						}
						else if (counter == 350) {			//Space
							led_on = 0;
						}
						else if (counter == 400) {			//Dash
							led_on = 1;
						}
						else if (counter == 550) {			//Space
							led_on = 0;
						}
						else if (counter == 700) {			//Finish
							counter = 0;
							FIFO_idx2 += 1;
							FIFO_idxsub -= 1;
						}
					}

					else if (key_pressed == 16) {			//P
						counter += 1;
						if (counter == 1) {					//Dot
							led_on = 1;
						}
						else if ((counter == 50)) {			//Space
							led_on = 0;
						}
						else if ((counter == 100)) {		//Dash
							led_on = 1;
						}
						else if ((counter == 250)) {		//Space
							led_on = 0;
						}
						else if ((counter == 300)) {		//Dash
							led_on = 1;
						}
						else if ((counter == 450)) {		//Space
							led_on = 0;
						}
						else if ((counter == 500)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 550)) {		//Space
							led_on = 0;
						}
						else if ((counter == 700)) {		//Finish
							counter = 0;
							FIFO_idx2 += 1;
							FIFO_idxsub -= 1;
						}
					}

					else if (key_pressed == 17) {			//Q
						counter += 1;
						if (counter == 1) {					//Dash
							led_on = 1;
						}
						else if ((counter == 150)) {		//Space
							led_on = 0;
						}
						else if ((counter == 200)) {		//Dash
							led_on = 1;
						}
						else if ((counter == 350)) {		//Space
							led_on = 0;
						}
						else if ((counter == 400)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 450)) {		//Space
							led_on = 0;
						}
						else if ((counter == 500)) {		//Dash
							led_on = 1;
						}
						else if ((counter == 650)) {		//Space
							led_on = 0;
						}
						else if ((counter == 800)) {		//Finish
							counter = 0;
							FIFO_idx2 += 1;
							FIFO_idxsub -= 1;
						}
					}

					else if (key_pressed == 18) {			//R
						counter += 1;
						if (counter == 1) {					//Dot
							led_on = 1;
						}
						else if (counter == 50) {			//Space
							led_on = 0;
						}
						else if (counter == 100) {			//Dash
							led_on = 1;
						}
						else if (counter == 250) {			//Space
							led_on = 0;
						}
						else if (counter == 300) {			//Dot
							led_on = 1;
						}
						else if (counter == 350) {			//Space
							led_on = 0;
						}
						else if (counter == 500) {			//Finish
							counter = 0;
							FIFO_idx2 += 1;
							FIFO_idxsub -= 1;
						}
					}

					else if (key_pressed == 19) {			//S
						counter += 1;
						if (counter == 1) {					//Dot
							led_on = 1;
						}
						else if (counter == 50) {			//Space
							led_on = 0;
						}
						else if (counter == 100) {			//Dot
							led_on = 1;
						}
						else if (counter == 150) {			//Space
							led_on = 0;
						}
						else if (counter == 200) {			//Dot
							led_on = 1;
						}
						else if (counter == 250) {			//Space
							led_on = 0;
						}
						else if (counter == 400) {			//Finish
							counter = 0;
							FIFO_idx2 += 1;
							FIFO_idxsub -= 1;
						}
					}

					else if (key_pressed == 20) {			//T
						counter += 1;
						if (counter == 1) {					//Dash
							led_on = 1;
						}
						else if (counter == 150) {			//Space
							led_on = 0;
						}
						else if (counter == 300) {			//Finish
							counter = 0;
							FIFO_idx2 += 1;
							FIFO_idxsub -= 1;
						}
					}

					else if (key_pressed == 21) {			//U
						counter += 1;
						if (counter == 1) {					//Dot
							led_on = 1;
						}
						else if (counter == 50) {			//Space
							led_on = 0;
						}
						else if (counter == 100) {			//Dot
							led_on = 1;
						}
						else if (counter == 150) {			//Space
							led_on = 0;
						}
						else if (counter == 200) {			//Dash
							led_on = 1;
						}
						else if (counter == 350) {			//Space
							led_on = 0;
						}
						else if (counter == 500) {			//Finish
							counter = 0;
							FIFO_idx2 += 1;
							FIFO_idxsub -= 1;
						}
					}

					else if (key_pressed == 22) {			//V
						counter += 1;
						if (counter == 1) {					//Dot
							led_on = 1;
						}
						else if ((counter == 50)) {			//Space
							led_on = 0;
						}
						else if ((counter == 100)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 150)) {		//Space
							led_on = 0;
						}
						else if ((counter == 200)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 250)) {		//Space
							led_on = 0;
						}
						else if ((counter == 300)) {		//Dash
							led_on = 1;
						}
						else if ((counter == 450)) {		//Space
							led_on = 0;
						}
						else if ((counter == 600)) {		//Finish
							counter = 0;
							FIFO_idx2 += 1;
							FIFO_idxsub -= 1;
						}
					}

					else if (key_pressed == 23) {			//W
						counter += 1;
						if (counter == 1) {					//Dot
							led_on = 1;
						}
						else if (counter == 50) {			//Space
							led_on = 0;
						}
						else if (counter == 100) {			//Dash
							led_on = 1;
						}
						else if (counter == 250) {			//Space
							led_on = 0;
						}
						else if (counter == 300) {			//Dash
							led_on = 1;
						}
						else if (counter == 450) {			//Space
							led_on = 0;
						}
						else if (counter == 600) {			//Finish
							counter = 0;
							FIFO_idx2 += 1;
							FIFO_idxsub -= 1;
						}
					}

					else if (key_pressed == 24) {			//X
						counter += 1;
						if (counter == 1) {					//Dash
							led_on = 1;
						}
						else if ((counter == 150)) {		//Space
							led_on = 0;
						}
						else if ((counter == 200)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 250)) {		//Space
							led_on = 0;
						}
						else if ((counter == 300)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 350)) {		//Space
							led_on = 0;
						}
						else if ((counter == 400)) {		//Dash
							led_on = 1;
						}
						else if ((counter == 550)) {		//Space
							led_on = 0;
						}
						else if ((counter == 700)) {		//Finish
							counter = 0;
							FIFO_idx2 += 1;
							FIFO_idxsub -= 1;
						}
					}

					else if (key_pressed == 25) {			//Y
						counter += 1;
						if (counter == 1) {					//Dash
							led_on = 1;
						}
						else if ((counter == 150)) {		//Space
							led_on = 0;
						}
						else if ((counter == 200)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 250)) {		//Space
							led_on = 0;
						}
						else if ((counter == 300)) {		//Dash
							led_on = 1;
						}
						else if ((counter == 450)) {		//Space
							led_on = 0;
						}
						else if ((counter == 500)) {		//Dash
							led_on = 1;
						}
						else if ((counter == 650)) {		//Space
							led_on = 0;
						}
						else if ((counter == 800)) {		//Finish
							counter = 0;
							FIFO_idx2 += 1;
							FIFO_idxsub -= 1;
						}
					}

					else if (key_pressed == 26) {			//Z
						counter += 1;
						if (counter == 1) {					//Dash
							led_on = 1;
						}
						else if ((counter == 150)) {		//Space
							led_on = 0;
						}
						else if ((counter == 200)) {		//Dash
							led_on = 1;
						}
						else if ((counter == 350)) {		//Space
							led_on = 0;
						}
						else if ((counter == 400)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 450)) {		//Space
							led_on = 0;
						}
						else if ((counter == 500)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 550)) {		//Space
							led_on = 0;
						}
						else if ((counter == 700)) {		//Finish
							counter = 0;
							FIFO_idx2 += 1;
							FIFO_idxsub -= 1;
						}
					}

					else if (key_pressed == 31) {			//1
						counter += 1;
						if (counter == 1) {					//Dot
							led_on = 1;
						}
						else if ((counter == 50)) {			//Space
							led_on = 0;
						}
						else if ((counter == 100)) {		//Dash
							led_on = 1;
						}
						else if ((counter == 250)) {		//Space
							led_on = 0;
						}
						else if ((counter == 300)) {		//Dash
							led_on = 1;
						}
						else if ((counter == 450)) {		//Space
							led_on = 0;
						}
						else if ((counter == 500)) {		//Dash
							led_on = 1;
						}
						else if ((counter == 650)) {		//Space
							led_on = 0;
						}
						else if ((counter == 700)) {		//Dash
							led_on = 1;
						}
						else if ((counter == 850)) {		//Space
							led_on = 0;
						}
						else if ((counter == 1000)) {		//Finish
							counter = 0;
							FIFO_idx2 += 1;
							FIFO_idxsub -= 1;
						}
					}

					else if (key_pressed == 32) {			//2
						counter += 1;
						if (counter == 1) {					//Dot
							led_on = 1;
						}
						else if ((counter == 50)) {			//Space
							led_on = 0;
						}
						else if ((counter == 100)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 150)) {		//Space
							led_on = 0;
						}
						else if ((counter == 200)) {		//Dash
							led_on = 1;
						}
						else if ((counter == 350)) {		//Space
							led_on = 0;
						}
						else if ((counter == 400)) {		//Dash
							led_on = 1;
						}
						else if ((counter == 550)) {		//Space
							led_on = 0;
						}
						else if ((counter == 600)) {		//Dash
							led_on = 1;
						}
						else if ((counter == 750)) {		//Space
							led_on = 0;
						}
						else if ((counter == 900)) {		//Finish
							counter = 0;
							FIFO_idx2 += 1;
							FIFO_idxsub -= 1;
						}
					}

					else if (key_pressed == 33) {			//3
						counter += 1;
						if (counter == 1) {					//Dot
							led_on = 1;
						}
						else if ((counter == 50)) {			//Space
							led_on = 0;
						}
						else if ((counter == 100)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 150)) {		//Space
							led_on = 0;
						}
						else if ((counter == 200)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 250)) {		//Space
							led_on = 0;
						}
						else if ((counter == 300)) {		//Dash
							led_on = 1;
						}
						else if ((counter == 450)) {		//Space
							led_on = 0;
						}
						else if ((counter == 500)) {		//Dash
							led_on = 1;
						}
						else if ((counter == 650)) {		//Space
							led_on = 0;
						}
						else if ((counter == 800)) {		//Finish
							counter = 0;
							FIFO_idx2 += 1;
							FIFO_idxsub -= 1;
						}
					}

					else if (key_pressed == 34) {			//4
						counter += 1;
						if (counter == 1) {					//Dot
							led_on = 1;
						}
						else if ((counter == 50)) {			//Space
							led_on = 0;
						}
						else if ((counter == 100)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 150)) {		//Space
							led_on = 0;
						}
						else if ((counter == 200)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 250)) {		//Space
							led_on = 0;
						}
						else if ((counter == 300)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 350)) {		//Space
							led_on = 0;
						}
						else if ((counter == 400)) {		//Dash
							led_on = 1;
						}
						else if ((counter == 550)) {		//Space
							led_on = 0;
						}
						else if ((counter == 700)) {		//Finish
							counter = 0;
							FIFO_idx2 += 1;
							FIFO_idxsub -= 1;
						}
					}

					else if (key_pressed == 35) {			//5
						counter += 1;
						if (counter == 1) {					//Dot
							led_on = 1;
						}
						else if ((counter == 50)) {			//Space
							led_on = 0;
						}
						else if ((counter == 100)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 150)) {		//Space
							led_on = 0;
						}
						else if ((counter == 200)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 250)) {		//Space
							led_on = 0;
						}
						else if ((counter == 300)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 350)) {		//Space
							led_on = 0;
						}
						else if ((counter == 400)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 450)) {		//Space
							led_on = 0;
						}
						else if ((counter == 600)) {		//Finish
							counter = 0;
							FIFO_idx2 += 1;
							FIFO_idxsub -= 1;
						}
					}

					else if (key_pressed == 36) {			//6
						counter += 1;
						if (counter == 1) {					//Dash
							led_on = 1;
						}
						else if ((counter == 150)) {		//Space
							led_on = 0;
						}
						else if ((counter == 200)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 250)) {		//Space
							led_on = 0;
						}
						else if ((counter == 300)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 350)) {		//Space
							led_on = 0;
						}
						else if ((counter == 400)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 450)) {		//Space
							led_on = 0;
						}
						else if ((counter == 500)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 550)) {		//Space
							led_on = 0;
						}
						else if ((counter == 700)) {		//Finish
							counter = 0;
							FIFO_idx2 += 1;
							FIFO_idxsub -= 1;
						}
					}

					else if (key_pressed == 37) {			//7
						counter += 1;
						if (counter == 1) {					//Dash
							led_on = 1;
						}
						else if ((counter == 150)) {		//Space
							led_on = 0;
						}
						else if ((counter == 200)) {		//Dash
							led_on = 1;
						}
						else if ((counter == 350)) {		//Space
							led_on = 0;
						}
						else if ((counter == 400)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 450)) {		//Space
							led_on = 0;
						}
						else if ((counter == 500)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 550)) {		//Space
							led_on = 0;
						}
						else if ((counter == 600)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 650)) {		//Space
							led_on = 0;
						}
						else if ((counter == 800)) {		//Finish
							counter = 0;
							FIFO_idx2 += 1;
							FIFO_idxsub -= 1;
						}
					}

					else if (key_pressed == 38) {			//8
						counter += 1;
						if (counter == 1) {					//Dash
							led_on = 1;
						}
						else if ((counter == 150)) {		//Space
							led_on = 0;
						}
						else if ((counter == 200)) {		//Dash
							led_on = 1;
						}
						else if ((counter == 350)) {		//Space
							led_on = 0;
						}
						else if ((counter == 400)) {		//Dash
							led_on = 1;
						}
						else if ((counter == 550)) {		//Space
							led_on = 0;
						}
						else if ((counter == 600)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 650)) {		//Space
							led_on = 0;
						}
						else if ((counter == 700)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 750)) {		//Space
							led_on = 0;
						}
						else if ((counter == 900)) {		//Finish
							counter = 0;
							FIFO_idx2 += 1;
							FIFO_idxsub -= 1;
						}
					}

					else if (key_pressed == 39) {			//9
						counter += 1;
						if (counter == 1) {					//Dash
							led_on = 1;
						}
						else if ((counter == 150)) {		//Space
							led_on = 0;
						}
						else if ((counter == 200)) {		//Dash
							led_on = 1;
						}
						else if ((counter == 350)) {		//Space
							led_on = 0;
						}
						else if ((counter == 400)) {		//Dash
							led_on = 1;
						}
						else if ((counter == 550)) {		//Space
							led_on = 0;
						}
						else if ((counter == 600)) {		//Dash
							led_on = 1;
						}
						else if ((counter == 750)) {		//Space
							led_on = 0;
						}
						else if ((counter == 800)) {		//Dot
							led_on = 1;
						}
						else if ((counter == 850)) {		//Space
							led_on = 0;
						}
						else if ((counter == 1000)) {		//Finish
							counter = 0;
							FIFO_idx2 += 1;
							FIFO_idxsub -= 1;
						}
					}

					else if (key_pressed == 30) {			//0
						counter += 1;
						if (counter == 1) {					//Dash
							led_on = 1;
						}
						else if ((counter == 150)) {		//Space
							led_on = 0;
						}
						else if ((counter == 200)) {		//Dash
							led_on = 1;
						}
						else if ((counter == 350)) {		//Space
							led_on = 0;
						}
						else if ((counter == 400)) {		//Dash
							led_on = 1;
						}
						else if ((counter == 550)) {		//Space
							led_on = 0;
						}
						else if ((counter == 600)) {		//Dash
							led_on = 1;
						}
						else if ((counter == 750)) {		//Space
							led_on = 0;
						}
						else if ((counter == 800)) {		//Dash
							led_on = 1;
						}
						else if ((counter == 950)) {		//Space
							led_on = 0;
						}
						else if ((counter == 1100)) {		//Finish
							counter = 0;
							FIFO_idx2 += 1;
							FIFO_idxsub -= 1;
						}
					}

/*					if (finish == 1) {
						FIFO_idx2 += 1;
						FIFO_idx -= 1;
						finish = 0;
						counter = 0;
					}*/

				}

				else if (FIFO_idxsub == 0) {
					FIFO_idx = 0;
					FIFO_idx2 = 0;
					FIFO_idxsub = 5;
				}

			// "Clear" the buffer.
			// FIFO_idx2 = FIFO_idx = 0;
			// memset{FIFO, '\0', sizeof(FIFO)};

		}
	}
	}

	// This line is supposed to never be reached.
	return 1;
}
