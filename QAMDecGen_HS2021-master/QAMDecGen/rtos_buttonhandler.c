#include <stdio.h>
#include <string.h>
#include <avr/io.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "event_groups.h"

#include "rtos_buttonhandler.h"

typedef struct {
    PORT_t *buttonPort;
	int8_t buttonPin;
    bool idleLevel;
    uint32_t pressCounter;
    buttonState_t buttonState;
    uint32_t buttonStateTimeout;
} button_t;

button_t buttons[NR_OF_BUTTONS];
uint32_t buttonTimeoutTime = BUTTONTIME_LONG / BUTTONTIME_TASK;

SemaphoreHandle_t buttondataLock;

void setupButton(uint8_t buttonID, PORT_t *buttonPort, int8_t buttonPin, bool idleLevel) {
    if(buttondataLock == NULL) {
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
    if(buttondataLock == NULL) {
        printf("run initButtonHandler() first to initilaize Buttons!");
        return;
    }
    buttons[buttonID].buttonPin = buttonPin;
	buttons[buttonID].buttonPort = buttonPort;
    buttons[buttonID].idleLevel = idleLevel;
    buttons[buttonID].pressCounter = 0;
    buttons[buttonID].buttonState = buttonState_Idle;
	
	buttonPort->DIRCLR = 0x01 << buttonPin;
}

void testButton(int8_t buttonID) {
    button_t* b = &buttons[buttonID];
	int level = (b->buttonPort->IN & (0x01 << b->buttonPin)) >> b->buttonPin;
    if(--b->buttonStateTimeout == 0) {
        b->buttonState = buttonState_Idle;
    }
    
    if(b->idleLevel == (level != 0)) { //when button == idle
        if(b->pressCounter > (BUTTONTIME_SHORT / BUTTONTIME_TASK)) {
            if(b->pressCounter < (BUTTONTIME_LONG / BUTTONTIME_TASK)) {
                //ShortPress
                b->buttonState = buttonState_Short;
            } else {
                //LongPress
                b->buttonState = buttonState_Long;
            }
            b->buttonStateTimeout = buttonTimeoutTime;
        }
        b->pressCounter = 0;
    } else {
        b->pressCounter++;
    }
}

void vButtonHandlerTask(void* pvParameters) {
    buttondataLock = xSemaphoreCreateMutex();
    printf("Starting Buttonhandlertask...");
    while(1) {
        xSemaphoreTake(buttondataLock, portMAX_DELAY);
        for(int i = 0; i < NR_OF_BUTTONS;i++) {
            if(buttons[i].buttonPin != -1) {
                testButton(i);
            }
        }
        xSemaphoreGive(buttondataLock);
        vTaskDelay(BUTTONTIME_TASK/portTICK_PERIOD_MS);
    }
}

void initButtonHandler(void) {
    for(int i = 0; i < NR_OF_BUTTONS; i++) {
        buttons[i].buttonPin = -1;
    }
	buttondataLock = xSemaphoreCreateMutex();
    xTaskCreate(vButtonHandlerTask, "buttonHandlerTask", configMINIMAL_STACK_SIZE + 50, NULL, 2, NULL);
}

void setButtonTimeoutTime(uint32_t time_ms) {
    buttonTimeoutTime = time_ms / BUTTONTIME_TASK;
}

buttonState_t getButtonState(uint8_t buttonID, bool reset) {
    buttonState_t returnValue;
    if(buttondataLock == NULL) {
        return buttonState_Idle;
    }
    xSemaphoreTake(buttondataLock, portMAX_DELAY);
    returnValue = buttons[buttonID].buttonState;
    if(reset == true) {
        buttons[buttonID].buttonState = buttonState_Idle;
    }
    xSemaphoreGive(buttondataLock);
    return returnValue;
}