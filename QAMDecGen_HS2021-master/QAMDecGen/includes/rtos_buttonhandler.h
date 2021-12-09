#ifndef BUTTONHANDLER_H
#define BUTTONHANDLER_H


#define BUTTONTIME_SHORT        80
#define BUTTONTIME_LONG         500
#define BUTTONTIME_TASK         10

#define NR_OF_BUTTONS           4

#define BUTTON1                 0
#define BUTTON2                 1
#define BUTTON3                 2
#define BUTTON4					3

typedef enum {
    buttonState_Idle = 0,
    buttonState_Short = 1,
    buttonState_Long = 2
} buttonState_t;


void setupButton(uint8_t buttonID, PORT_t *buttonPort, int8_t buttonPin, bool idleLevel);
void initButtonHandler(void);
void setButtonTimeoutTime(uint32_t time_ms);
buttonState_t getButtonState(uint8_t buttonID, bool reset);

#endif