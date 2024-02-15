#ifndef CHARGEPROCESSCONTROLLER_H
#define CHARGEPROCESSCONTROLLER_H
#include <Arduino.h>
#include <iostream>
#include "ACSocketControllerDebug.h"

using namespace std;

#define PHASE_12V 12
#define PHASE_9V 9
#define PHASE_6V 6
#define PHASE_3V 3
#define PHASE_0V 0
#define PHASE_UE -1
#define PHASE_N12V -12

#define CONNECTED_OK 1
#define CONNECTED_CHARGING 2
#define CONNECTED_NOT 0
#define CONNECTED_ERROR -1

typedef struct
{
    uint16_t cpPin;
    uint16_t cpDetPin;
    uint16_t ppPin;
    uint16_t pwmChannel;
    uint16_t pwmDutyCycle;
    uint16_t connectorMaxCurrent;
    uint16_t contactorPin;
    uint16_t motorPin;
    uint16_t motorFB;
    uint16_t emsPin;
    bool evFull = false;
} EVSE_CONFIG_t;

class ChargeProcessController
{
public:
    EVSE_CONFIG_t evseConfig;
    const char *name;
    uint8_t id = 0;
    uint8_t ppLimit = 0;
    int8_t stateP = 0;
    int8_t stateN = 0;
    ChargeProcessController(EVSE_CONFIG_t _evseConfig, const char *_name, uint8_t _id) : evseConfig(_evseConfig), name(_name), id(_id){};
    ~ChargeProcessController(){};
    void INIT();
    uint8_t PP_READ();
    void setMaxCurrent(uint32_t current);
    int8_t isConnected();
    int8_t isCharging();
    void setStates(int8_t, int8_t);
    void CONTACTOR_CONTROL(bool state);
    void MOTOR_CONTROL(bool state);
    bool MOTOR_LOCK_FB();
    bool EM_STOP_FB();
    void CHARGE_PROCESS();
};
#endif
