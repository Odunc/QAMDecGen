/*
 * QAMDecGen.c
 *
 * Created: 20.03.2018 18:32:07
 * Author : chaos
 */ 

//#include <avr/io.h>
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
#include "stack_macros.h"

#include "mem_check.h"

#include "init.h"
#include "utils.h"
#include "errorHandler.h"
#include "NHD0420Driver.h"

#include "qaminit.h"
#include "qamgen.h"
#include "qamdec.h"

#include "rtos_buttonhandler.h"



#define BUTTON1SHORTPRESSEDMASK     0x01
#define BUTTON2SHORTPRESSEDMASK     0x02



extern void vApplicationIdleHook( void );
void vControllTask(void *pvParameters);
void vButtonTask(void *pvParameters);


TaskHandle_t xControllTask;
TaskHandle_t xButtonTask;

typedef	enum{
	idle,
	data_1,
	data_2,	
	writedata
} eControllStates;


void vApplicationIdleHook( void )
{	
	
}

int main(void)
{
    resetReason_t reason = getResetReason();

	vInitClock();
	vInitDisplay();
	
	initDAC();
	initDACTimer();
	initGenDMA();
	initADC();
	initADCTimer();
	initDecDMA();
	
	xTaskCreate(vQuamGen, NULL, configMINIMAL_STACK_SIZE+800, NULL, 2, NULL);
	xTaskCreate(vQuamDec, NULL, configMINIMAL_STACK_SIZE+100, NULL, 1, NULL);
	xTaskCreate(vControllTask, NULL, configMINIMAL_STACK_SIZE+100, NULL, 1, &xControllTask);
	xTaskCreate(vButtonTask, (const char *) "ButtonTask", configMINIMAL_STACK_SIZE, NULL,1, NULL);
	

	vDisplayClear();
	vDisplayWriteStringAtPos(0,0,"FreeRTOS 10.0.1");
	vDisplayWriteStringAtPos(1,0,"EDUBoard 1.0");
	vDisplayWriteStringAtPos(2,0,"QAMDECGEN-Base");
	vDisplayWriteStringAtPos(3,0,"ResetReason: %d", reason);
	vTaskStartScheduler();
	return 0;
}


void vControllTask(void *pvParameters){
	(void) pvParameters;
	uint32_t Buttonvalue;
	uint8_t DataString[33];
	
	eControllStates Controll = idle;
	
	for(;;) {
		xTaskNotifyWait(0, 0xffffffff, &Buttonvalue, pdMS_TO_TICKS(200));
		
		switch( Controll){
			case idle:{
				if (Buttonvalue&BUTTON1SHORTPRESSEDMASK)
				{
					Controll=data_1;
				}
				
				if (Buttonvalue&BUTTON2SHORTPRESSEDMASK)
				{
					Controll=data_2;
				}
				break;
			}// end case idle
			
			case data_1: {
				//0b0010001
				DataString[0] = 0b01; // 35 dec
				DataString[1] = 0b00;
				DataString[2] = 0b10;
				DataString[3] = 0b00;
				Controll = writedata;
				break;
			}// end case data 1
			
			case data_2: {
				DataString[0] = 0b10011100; // 156 dec
				DataString[1] = 0b11000000; // 192 dec
				Controll = writedata;
			}// end case data_2
			
			case writedata: {
				vsendCommand(DataString);
				Controll = idle;
				break;
			}// end case write data
		}// end switch		
	}//end for
}// end void



void vButtonTask(void *pvParameters) {
	initButtonHandler();
	setupButton(BUTTON1, &PORTF, 4, 1);
	setupButton(BUTTON2, &PORTF, 5, 1);
	setupButton(BUTTON3, &PORTF, 6, 1);
	setupButton(BUTTON4, &PORTF, 7, 1);
	vTaskDelay(3000);
	
	for(;;) {
		if(getButtonState(BUTTON1, false) == buttonState_Short){
			xTaskNotify(xControllTask,BUTTON1SHORTPRESSEDMASK,eSetValueWithOverwrite);	
		}
		
		if(getButtonState(BUTTON2, false) == buttonState_Short){
			xTaskNotify(xControllTask,BUTTON2SHORTPRESSEDMASK,eSetValueWithOverwrite);
		}
		vTaskDelay(100/portTICK_RATE_MS);
	}
}
