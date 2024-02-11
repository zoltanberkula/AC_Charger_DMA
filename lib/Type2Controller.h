#pragma once

#include <Arduino.h>
#include "SPI.h"
#include <SimpleSerialShell.h>
#include "chargeController.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <rom/rtc.h>
#include <esp_adc_cal.h>
#include <esp32-hal-adc.h>
#include "hal_config.h"
#include "Type2ControllerDebug.h"

#define ACTIVE_0 0
#define ACTIVE_R SIDE_RIGHT
#define ACTIVE_L SIDE_LEFT
#define ACTIVE_DUAL 3

#define EVSE_MAX_CURRENT 16

#define SERIAL_MON_BAUD_RATE 115200 // Serial Monitor Baud Rate
#define CP_BUFF_SIZE 100

namespace Type2Controller
{
    extern uint8_t chargeMode;

    extern uint8_t lastMode;
    extern uint32_t maxCurrentR;
    extern uint32_t maxCurrentL;

    extern unsigned long now;

    extern volatile uint8_t buffLId; // id of circular buffer
    extern volatile uint8_t buffRId; // id of circular buffer

    extern bool startGR; // Use this for right side testing
    extern bool startGL; // Use this for left side testing
    extern bool stopGR;  // stop Gun1 command flag
    extern bool stopGL;  // stop Gun2 command flag
    extern uint8_t evseMaxCurrent;

    extern hw_timer_t *timer;     // timer variable of type hw_timer_t initialized with NULL value
    extern portMUX_TYPE timerMux; // variable of type portMUX_TYPE for synchronization between main loop and ISR

    extern ChargeController *evseR; // ChargeController 1 initialization
    extern ChargeController *evseL; // ChargeController 2 initialization

    void hmiBtn_INIT();
    void IRAM_ATTR cpSampler(); // ISR
    uint16_t getPP(uint8_t side);
    int startGunR(int argc, char **argv);
    int startGunL(int argc, char **argv);
    int stopGunR(int argc, char **argv);
    int stopGunL(int argc, char **argv);
    int reboot(int argc, char **argv);
    void EXECUTE();
    void setStates();
    void setup();
    uint8_t getMode();
    void dynamicLoadProcess();
    void loop();
    bool isConnected(uint8_t connectorId);

}