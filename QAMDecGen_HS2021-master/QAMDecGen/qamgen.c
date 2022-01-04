/*
* qamgen.c
*
* Created: 05.05.2020 16:24:59
*  Author: Chaos
*/ 
#include "avr_compiler.h"
#include "pmic_driver.h"
#include "TC_driver.h"
#include "clksys_driver.h"
#include "sleepConfig.h"
#include "port_driver.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "semphr.h"
#include "stack_macros.h"

#include "mem_check.h"

#include "qaminit.h"
#include "qamgen.h"
#include "rtos_buttonhandler.h"


// Uart defines
#define USART_SERIAL_EXAMPLE            &USARTC0
#define USART_SERIAL_EXAMPLE_BAUDRATE   9600
#define USART_SERIAL_CHAR_LENGTH        USART_CHSIZE_8BIT_gc
#define USART_SERIAL_PARITY             USART_PMODE_DISABLED_gc
#define USART_SERIAL_STOP_BIT           false

/*
// USART options.
static usart_rs232_options_t USART_SERIAL_OPTIONS = {
	.baudrate = USART_SERIAL_EXAMPLE_BAUDRATE,
	.charlength = USART_SERIAL_CHAR_LENGTH,
	.paritytype = USART_SERIAL_PARITY,
	.stopbits = USART_SERIAL_STOP_BIT
};
*/


// test sinus lookup table for 2 periods (=> 2 DMA Channels)
// 1 DMA channel sends data to DAC
// => second DMA channel refill with datas
// const sinus lookup table, do not use it, send idlebits instead
const int16_t sinLookup100[NR_OF_SAMPLES*2] = {	0x0,0x27,0x9c,0x159,0x258,0x38e,0x4f0,0x670,
												0x800,0x990,0xb10,0xc72,0xda8,0xea7,0xf64,0xfd9,
												0xFFF,0xfd9,0xf64,0xea7,0xda8,0xc72,0xb10,0x990,
												0x800,0x670,0x4f0,0x38e,0x258,0x159,0x9c,0x27,
												0x0,0x27,0x9c,0x159,0x258,0x38e,0x4f0,0x670,
												0x800,0x990,0xb10,0xc72,0xda8,0xea7,0xf64,0xfd9,
												0xFFF,0xfd9,0xf64,0xea7,0xda8,0xc72,0xb10,0x990,
												0x800,0x670,0x4f0,0x38e,0x258,0x159,0x9c,0x27};

/*
const lookup tables for all 4 symbols
00 => 100% Amplitude & Phaseshift	0�
01 => 100% Amplitude & Phaseshift  180�
10 =>  50% Amplitude & Phaseshift	0�
11 =>  50% Amplitude & Phaseshift  180�
MSB = Amplitude | LSB = Phaseshift
=> Generated by https://www.daycounter.com/Calculators/Sine-Generator-Calculator.phtml

calucalted offset offset
0x0,0x14,0x4e,0xad,0x12c,0x1c7,0x278,0x338,
0x400,0x4c8,0x588,0x639,0x6d4,0x753,0x7b2,0x7ec,
0x800,0x7ec,0x7b2,0x753,0x6d4,0x639,0x588,0x4c8,
0x400,0x338,0x278,0x1c7,0x12c,0xad,0x4e,0x14
=
0x400,0x414,0x44e,0x4ad,0x52c,0x5c1,0x678,0x738,
0x800,0x8c8,0x988,0xa39,0xad4,0xb53,0xbb2,0xbec,
0xc00,0xbec,0xbb2,0xb53,0xad4,0xa39,0x988,0x8c8,
0x800,0x738,0x678,0x5c1,0x52c,0x4ad,0x44e,0x414
*/

const int16_t Symbol_00_lookup[NR_OF_SAMPLES] = {	
											0x0,0x27,0x9c,0x159,0x258,0x38e,0x4f0,0x670,
											0x800,0x990,0xb10,0xc72,0xda8,0xea7,0xf64,0xfd9,
											0xFFF,0xfd9,0xf64,0xea7,0xda8,0xc72,0xb10,0x990,
											0x800,0x670,0x4f0,0x38e,0x258,0x159,0x9c,0x27,
											};
										
const int16_t Symbol_01_lookup[NR_OF_SAMPLES] = {	
											0xFFF,0xfd9,0xf64,0xea7,0xda8,0xc72,0xb10,0x990,
											0x800,0x670,0x4f0,0x38e,0x258,0x159,0x9c,0x27,
											0x0,0x27,0x9c,0x159,0x258,0x38e,0x4f0,0x670,
											0x800,0x990,0xb10,0xc72,0xda8,0xea7,0xf64,0xfd9,
											};

const int16_t Symbol_10_lookup[NR_OF_SAMPLES] = {	
											0x400,0x414,0x44e,0x4ad,0x52c,0x5c1,0x678,0x738,
											0x800,0x8c8,0x988,0xa39,0xad4,0xb53,0xbb2,0xbec,
											0xc00,0xbec,0xbb2,0xb53,0xad4,0xa39,0x988,0x8c8,
											0x800,0x738,0x678,0x5c1,0x52c,0x4ad,0x44e,0x414
											};	

const int16_t Symbol_11_lookup[NR_OF_SAMPLES] = {	
											0xc00,0xbec,0xbb2,0xb53,0xad4,0xa39,0x988,0x8c8,
											0x800,0x738,0x678,0x5c1,0x52c,0x4ad,0x44e,0x414,
											0x400,0x414,0x44e,0x4ad,0x52c,0x5c1,0x678,0x738,
											0x800,0x8c8,0x988,0xa39,0xad4,0xb53,0xbb2,0xbec
											};

// const Idle Bit's
const uint8_t Idle_bits_1 = 0b01;
const uint8_t Idle_bits_2 = 0b10;

// const Sync bits
const uint8_t Sync_Symbol = 0b11;

// test lenght bits
const uint8_t Length_bits[] = {01,00};
	
// flag to toggle idle bits	
volatile bool send_idle_bit_2 = false;
volatile bool New_datas_rdy = false;
volatile bool Datas_rdy	= false;
volatile uint8_t x = 0;

uint8_t DataToSend = 0;


// var to store protocoll symbols for queue
uint8_t protocoll_symbols[30] = {};
uint16_t SymbolCounter = 0;

volatile uint8_t	Rx_Symbol[30] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};	
uint8_t queue_sampels;



//------------ Queues -----------
QueueHandle_t xSymbolQueue;



void vQuamGen(void *pvParameters) {
	while(evDMAState == NULL) {
		vTaskDelay(3/portTICK_RATE_MS);
	}
	xEventGroupWaitBits(evDMAState, DMAGENREADY, false, true, portMAX_DELAY);
	
	// baudrat 9600
	USARTC0.BAUDCTRLA = 0xD0 & 0xFF;
	USARTC0.BAUDCTRLB |= ((0 & 0x0F) << 0x04);
	
	USARTC0.CTRLA = USART_RXCINTLVL_LO_gc;
	USARTC0.STATUS |= USART_RXCIF_bm;
	
	USARTC0.CTRLB = USART_TXEN_bm | USART_RXEN_bm;
	USARTC0.CTRLC = USART_CHSIZE_8BIT_gc;
	USARTC0.CTRLC &= ~(USART_PMODE0_bm | USART_PMODE1_bm | USART_SBMODE_bm);
	PORTC.DIR = 0x08;
	
	while(!(USARTC0.STATUS & USART_DREIF_bm));
	USARTC0.DATA = 'G';
	
	xSymbolQueue	= xQueueCreate(10, sizeof(uint8_t)*30); 
	
	for(;;){		
		vTaskDelay(100/portTICK_RATE_MS);
	}
	
}//

void fillBuffer(uint16_t buffer[NR_OF_SAMPLES]) {
		
	if(  New_datas_rdy ){
		// fetch them		
		if(xQueueReceiveFromISR(xSymbolQueue, (void*)&Rx_Symbol[x],NULL ) == pdTRUE){	
			// if succesfully reset flag
			New_datas_rdy = false;	
			Datas_rdy = true;
		}
	}
	
	// check previous Idle bit
	if(Datas_rdy & !send_idle_bit_2){
		switch(Rx_Symbol[x]){
			case 0b00000000:{
				for(int i = 0; i < NR_OF_SAMPLES;i++) {
						buffer[i] = Symbol_00_lookup[i];
					}
			break;
			}
			case 0b00000001:{
				for(int i = 0; i < NR_OF_SAMPLES;i++) {
					buffer[i] = Symbol_01_lookup[i];
				}
				break;
			}
			case 0b00000010:{
				for(int i = 0; i < NR_OF_SAMPLES;i++) {
					buffer[i] = Symbol_10_lookup[i];
				}
				break;
			}
			case 0b00000011:{
				for(int i = 0; i < NR_OF_SAMPLES;i++) {
					buffer[i] = Symbol_11_lookup[i];
				}
				break;
			}				
		}// end switch
		if( x < SymbolCounter){
			x++;
		}
		else{
			// if all datas send, reset flag
			Datas_rdy = false;
			x = 0;
		}		
	}
	// if no Datas ready, send idle bits
	else{
		// toggle Idle bits
		if( !send_idle_bit_2){
			for(int i = 0; i < NR_OF_SAMPLES;i++) {
				buffer[i] = Symbol_01_lookup[i];				
			}
			send_idle_bit_2 = true;
		}
		else{
			for(int i = 0; i < NR_OF_SAMPLES;i++) {
				buffer[i] = Symbol_10_lookup[i];
			}			
			send_idle_bit_2 = false;
		}
	}// end if no messages in queue
}// end void fillbuffer


void createProtocoll( uint8_t Data_Length, uint8_t Data ){
	uint8_t Protocoll_Index = 0;
	uint8_t Length_Symbol = 0;
	uint8_t Data_Symbol = 0;
	uint32_t CheckSum	= 0;
	uint8_t CheckSum_Symbol = 0;
	
	
	// first of all, add sync symbol
	protocoll_symbols[Protocoll_Index] = Sync_Symbol;
	Protocoll_Index++;
	
	// add lenght symbol
	for(uint8_t i = 0; i<=3; i++){
		Length_Symbol = Data_Length >> (6 - (2*i) ) &0b11;
		protocoll_symbols[Protocoll_Index] = Length_Symbol;
		CheckSum += Length_Symbol;
		Protocoll_Index++;
	}
	
	// add data symbol
	for(uint8_t i = 0; i<=3; i++){
		Data_Symbol = Data >> (6 - (2*i) ) &0b11;
		protocoll_symbols[Protocoll_Index] = Data_Symbol;
		CheckSum += Data_Symbol;
		Protocoll_Index++;
	}
	
	// add checksum
	// 16 symbols
	for(uint8_t i =0;i<=15;i++){
		CheckSum_Symbol = CheckSum >> (30 - (2*i)) &0b11;
		protocoll_symbols[Protocoll_Index] = CheckSum_Symbol;
		Protocoll_Index++;
	}
	
	xQueueSend(xSymbolQueue, (void*) &protocoll_symbols, (TickType_t) 10);
	SymbolCounter = Protocoll_Index;
	New_datas_rdy = true;	
}


void vButtonTask(void *pvParameters) {
	initButtonHandler();
	setupButton(BUTTON1, &PORTF, 4, 1);
	setupButton(BUTTON2, &PORTF, 5, 1);
	setupButton(BUTTON3, &PORTF, 6, 1);
	setupButton(BUTTON4, &PORTF, 7, 1);		
	vTaskDelay(3000);
	
	for(;;) {
		
		if(getButtonState(BUTTON1, true) == buttonState_Short){
			createProtocoll(4,132);			
		}
		vTaskDelay(10/portTICK_RATE_MS);
	}
}



ISR(DMA_CH0_vect)
{
	//static signed BaseType_t test;	
	DMA.CH0.CTRLB|=0x10;
	fillBuffer(&dacBuffer0[0]);
}

ISR(DMA_CH1_vect)
{
	DMA.CH1.CTRLB|=0x10;
	fillBuffer(&dacBuffer1[0]);
}



/* 
// Button 1 send value 35
// => sync (11), lenght (11) data (10 00 11)
if(getButtonState(BUTTON1, true) == buttonState_Short){
	// first add sync symbol to queue
	protocoll_symbols[0] = Sync_Symbol;
	
	// add lenght symbol
	protocoll_symbols[1] = 0b11;
	
	// data
	protocoll_symbols[2] = 0b10;
	protocoll_symbols[3] = 0b00;
	protocoll_symbols[4] = 0b11;
	SymbolCounter = 4;
	
	// send complete protocoll to queue
	xQueueSend(xSymbolQueue, (void*) &protocoll_symbols, (TickType_t) 10);
	New_datas_rdy = true;
	
}

if(getButtonState(BUTTON2, false) == buttonState_Short){
	// ft 132
	// 1000 0100 -> 10 00 01 00
	// sync - lenght	- data		- checksum
	//	11     01 00   10 00 01 00  tbd
	DataToSend = 132;
	
	
	// first add sync symbol to queue
	protocoll_symbols[0] = Sync_Symbol;
	// add lenght symbol, length = 8bit => 4 symbol
	// adjust lenght symbol, just for testing
	
	
	protocoll_symbols[1] = 0b00;
	protocoll_symbols[2] = 0b00;
	protocoll_symbols[3] = 0b01;
	protocoll_symbols[4] = 0b00;
	
	//data symbols
	calc_symbol = DataToSend>>6 & 0b11;
	protocoll_symbols[5] = calc_symbol;
	calc_symbol = DataToSend>>4 & 0b11;
	protocoll_symbols[6] = calc_symbol;
	calc_symbol = DataToSend>>2 & 0b11;
	protocoll_symbols[7] = calc_symbol;
	calc_symbol = DataToSend & 0b11;
	protocoll_symbols[8] = calc_symbol;
	
	// checksum
	// data checksum => 10 + 00 + 01 + 00 => 3
	// add length => 01 00 => 1 +3 = 4 => 01 00
	protocoll_symbols[9]	= 0b01;
	protocoll_symbols[10]	= 0b00;
	
	//symbol length
	SymbolCounter = 10;
	// send complete protocoll to queue
	xQueueSend(xSymbolQueue, (void*) &protocoll_symbols, (TickType_t) 10);
	New_datas_rdy=true;
}

*/