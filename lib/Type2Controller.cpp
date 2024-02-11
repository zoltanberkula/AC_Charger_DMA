#include "Type2Controller.h"

namespace Type2Controller
{

    volatile uint16_t cpSample_L; // raw value of the ADC CP_L
    volatile uint16_t cpSample_R; // raw value of the ADC CP_R
    volatile int8_t stateP_L;     // state according to the CP Positive Peak Voltage
    volatile int8_t stateN_L;     // state according to the CP Negative Peak Voltage
    volatile int8_t stateP_R;     // state according to the CP Positive Peak Voltage
    volatile int8_t stateN_R;     // state according to the CP Negative Peak Voltage
    volatile uint16_t ppSample_R;
    volatile uint16_t ppSample_L;
    volatile uint16_t buffL[CP_BUFF_SIZE]; // circular buffer
    volatile uint16_t buffR[CP_BUFF_SIZE]; // circular buffer
    EVSE_CONFIG_t evseRCFG;                // Evse 1 config initialization
    EVSE_CONFIG_t evseLCFG;                // Evse 2 config initialization

    uint8_t chargeMode = ACTIVE_0;
    uint8_t lastMode = ACTIVE_0;
    uint32_t maxCurrentR = 0;
    uint32_t maxCurrentL = 0;

    unsigned long now = 0;

    volatile uint8_t buffLId = 0; // id of circular buffer
    volatile uint8_t buffRId = 0; // id of circular buffer

    bool startGR = false; // Use this for right side testing
    bool startGL = false; // Use this for left side testing
    bool stopGR = false;  // stop Gun1 command flag
    bool stopGL = false;  // stop Gun2 command flag

    uint8_t evseMaxCurrent = EVSE_MAX_CURRENT;

    hw_timer_t *timer = NULL;                             // timer variable of type hw_timer_t initialized with NULL value
    portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; // variable of type portMUX_TYPE for synchronization between main loop and ISR

    ChargeController *evseR = nullptr; // ChargeController 1 initialization
    ChargeController *evseL = nullptr; // ChargeController 2 initialization

    void hmiBtn_INIT(){
            startBtn->onHigh = [](void *)
            { DEBUG_LN_TYPE2_CONTROLLER("%s", "Start Btn HIGH");
        DEBUG_LN_TYPE2_CONTROLLER("%s", "Start Btn HIGH"); };

            startBtn->onLow = [](void *)
            { DEBUG_LN_TYPE2_CONTROLLER("%s", "Start Btn LOW");
        DEBUG_LN_TYPE2_CONTROLLER("%s", "Start Btn LOW"); };

            stopBtn->onHigh = [](void *)
            { DEBUG_LN_TYPE2_CONTROLLER("%s", "Stop Btn HIGH");
        DEBUG_LN_TYPE2_CONTROLLER("%s", "Stop Btn LOW"); };

            stopBtn->onLow = [](void *)
            { DEBUG_LN_TYPE2_CONTROLLER("%s", "Stop Btn LOW");
        DEBUG_LN_TYPE2_CONTROLLER("%s", "Stop Btn LOW"); };
    };


    void IRAM_ATTR cpSampler() // ISR
    {
        portENTER_CRITICAL_ISR(&timerMux);
        cpSample_R = analogRead(CP_R_DET_PIN);
        cpSample_L = analogRead(CP_L_DET_PIN);
        buffR[buffRId++] = cpSample_R;
        buffL[buffLId++] = cpSample_L;

        ppSample_R = analogRead(PP_R_DET_PIN);
        ppSample_L = analogRead(PP_L_DET_PIN);
        portEXIT_CRITICAL_ISR(&timerMux);

        if (buffLId >= CP_BUFF_SIZE)
        {
            buffLId = 0;
        }
        if (buffRId >= CP_BUFF_SIZE)
        {
            buffRId = 0;
        }
    }

    uint16_t getPP(uint8_t side)
    {
        uint16_t ppVal = 0;
        portENTER_CRITICAL(&timerMux);
        if (side == SIDE_RIGHT)
        {
            ppVal = ppSample_R;
        }
        else if (side == SIDE_LEFT)
        {
            ppVal = ppSample_L;
        }
        portEXIT_CRITICAL(&timerMux);
        return ppVal;
    }

    int startGunR(int argc, char **argv)
    {
        DEBUG_LN_TYPE2_CONTROLLER("%s", "GUNR Started");
        startGR = true;
        stopGR = false;
        return 0;
    }

    int startGunL(int argc, char **argv)
    {
        DEBUG_LN_TYPE2_CONTROLLER("%s", "GUNL Started");
        startGL = true;
        stopGL = false;
        return 0;
    }

    int stopGunR(int argc, char **argv)
    {
        DEBUG_LN_TYPE2_CONTROLLER("%s", "GUNR Stopped");
        stopGR = true;
        startGR = false;
        return 0;
    }

    int stopGunL(int argc, char **argv)
    {
        DEBUG_LN_TYPE2_CONTROLLER("%s", "GUNL Stopped");
        stopGL = true;
        startGL = false;
        return 0;
    }

    int reboot(int argc, char **argv)
    {
        ESP.restart();
        return 0;
    }

    void EXECUTE()
    {
        shell.executeIfInput();
        if (startGR == true && stopGR == false)
        {
            DEBUG_LN_TYPE2_CONTROLLER("%s", evseR->name);
            evseR->setStates(stateP_R, stateN_R);
            evseR->CHARGE_PROCESS();
        }
        else if (stopGR == true)
        {
            // evseR->setMaxCurrent(0);
            DEBUG_LN_TYPE2_CONTROLLER("%s", "GUNR OFF");
            evseR->CONTACTOR_CONTROL(0);
            evseR->MOTOR_CONTROL(0);
            evseR->evseConfig.evFull = false;
        }

        if (startGL == true && stopGL == false)
        {
            DEBUG_LN_TYPE2_CONTROLLER("%s", evseL->name);
            evseL->setStates(stateP_L, stateN_L);
            evseL->CHARGE_PROCESS();
        }
        else if (stopGL == true)
        {
            // evseL->setMaxCurrent(0);
            DEBUG_LN_TYPE2_CONTROLLER("%s", "GUNL OFF");
            evseL->CONTACTOR_CONTROL(0);
            evseL->MOTOR_CONTROL(0);
            evseL->evseConfig.evFull = false;
        }
    }

    void setStates()
    {
        uint16_t tempLMax = 0; // left
        uint16_t tempLMin = 4096;
        uint16_t tempRMax = 0;
        uint16_t tempRMin = 4096;

        portENTER_CRITICAL(&timerMux); // aka Semaphore take

        for (int i = 0; i < CP_BUFF_SIZE; i++)
        {
            buffL[i] > tempLMax ? tempLMax = buffL[i] : 0;
            buffL[i] < tempLMin ? tempLMin = buffL[i] : 0;
        }

        for (int i = 0; i < CP_BUFF_SIZE; i++)
        {
            buffR[i] > tempRMax ? tempRMax = buffR[i] : 0;
            buffR[i] < tempRMin ? tempRMin = buffR[i] : 0;
        }

        portEXIT_CRITICAL(&timerMux); // aka Semaphore give
        DEBUG_LN_TYPE2_CONTROLLER("%u", tempLMax);
        DEBUG_LN_TYPE2_CONTROLLER("%u", tempRMax);

        if (tempLMax > 4090 && tempLMax <= 4095)
        {
            stateP_L = 12; // 12V
        }
        else if (tempLMax > 3800 && tempLMax < 4000)
        {
            stateP_L = 9; // 9V
        }
        else if (tempLMax > 2500 && tempLMax < 3600)
        {
            stateP_L = 6; // 6V
        }
        else if (tempLMax > 2400 && tempLMax < 2600)
        {
            stateP_L = 3; // 3V
        }
        else if (tempLMax > 2048 && tempLMax < 2350)
            stateP_L = 0;

        if (tempLMin > 2000 && tempLMin < 2200)
        {
            stateN_L = 0; // ERROR 0V
        }
        else if (tempLMin > 0 && tempLMin <= 100)
        {
            stateN_L = -12; //-12V
        }
        else
        {
            DEBUG_LN_TYPE2_CONTROLLER("%u", tempLMin);
            stateN_L = -1; // Unknown ERROR
        }

        if (tempRMax > 4090 && tempRMax <= 4095)
        {
            stateP_R = 12; // 12V
        }
        else if (tempRMax > 3800 && tempRMax < 4000)
        {
            stateP_R = 9; // 9V
        }
        else if (tempRMax > 2500 && tempRMax < 3600)
        {
            stateP_R = 6; // 6V
        }
        else if (tempRMax > 2400 && tempRMax < 2600)
        {
            stateP_R = 3; // 3V
        }
        else if (tempRMax > 2048 && tempRMax < 2350)
            stateP_R = 0;

        if (tempRMin > 2000 && tempRMin < 2200)
        {
            stateN_R = 0; // ERROR 0V
        }
        else if (tempRMin > 0 && tempRMin <= 100)
        {
            stateN_R = -12; //-12V
        }
        else
        {
            DEBUG_LN_TYPE2_CONTROLLER("%u", tempRMin);
            stateN_R = -1; // Unknown ERROR
        }
    }

    void setup()
    {
#ifdef BROWNOUT_OFF
        WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
#endif
        Serial.begin(SERIAL_MON_BAUD_RATE, SERIAL_8N1);
        shell.attach(Serial);
        shell.addCommand(F("reboot"), reboot);
        shell.addCommand(F("startGunR"), startGunR);
        shell.addCommand(F("startGunL"), startGunL);
        shell.addCommand(F("stopGunR"), stopGunR);
        shell.addCommand(F("stopGunL"), stopGunL);

        timer = timerBegin(0, 80, true);
        timerAttachInterrupt(timer, &cpSampler, true);
        timerAlarmWrite(timer, 1010, true);
        timerAlarmEnable(timer);

        evseRCFG.cpPin = CP_R_PWM_PIN;                      // EVSE 1 Params
        evseRCFG.cpDetPin = CP_R_DET_PIN;                   // EVSE 1 Params
        evseRCFG.ppPin = PP_R_DET_PIN;                      // EVSE 1 Params
        evseRCFG.pwmChannel = CP_R_PWM_CHANNEL;             // EVSE 1 Params
        evseRCFG.pwmDutyCycle = PWM_MAX;                    // EVSE 1 Params
        evseRCFG.contactorPin = R_3P_CONTACTOR_PIN;         // EVSE 1 Params
        evseRCFG.motorPin = R_MOTOR_PIN;                    // EVSE 1 Params
        evseRCFG.motorFB = R_MOTOR_LOCK_FB;                 // EVSE 1 Params
        evseRCFG.connectorMaxCurrent = EVSE_MAX_CURRENT_32; // EVSE 1 Params max current for one gun
        evseRCFG.emsPin = EM_STOP_PIN;                      // EVSE 1 Params

        evseLCFG.cpPin = CP_L_PWM_PIN;                      // EVSE 2 Params
        evseLCFG.cpDetPin = CP_L_DET_PIN;                   // EVSE 2 Params
        evseLCFG.ppPin = PP_L_DET_PIN;                      // EVSE 2 Params
        evseLCFG.pwmChannel = CP_L_PWM_CHANNEL;             // EVSE 2 Params
        evseLCFG.pwmDutyCycle = PWM_MAX;                    // EVSE 2 Params
        evseLCFG.contactorPin = L_3P_CONTACTOR_PIN;         // EVSE 2 Params
        evseLCFG.motorPin = L_MOTOR_PIN;                    // EVSE 2 Params
        evseLCFG.motorFB = L_MOTOR_LOCK_FB;                 // EVSE 2 Params
        evseLCFG.connectorMaxCurrent = EVSE_MAX_CURRENT_32; // EVSE 2 Params max current for one gun
        evseLCFG.emsPin = EM_STOP_PIN;                      // EVSE 2 Params

        evseR = new ChargeController(evseRCFG, "evseR", SIDE_RIGHT);
        evseL = new ChargeController(evseLCFG, "evseL", SIDE_LEFT);
        evseR->INIT(); // EVSE1 Initialization
        evseL->INIT(); // EVSE2 Initialization
    }

    uint8_t getMode()
    {
        if (evseR->isConnected() == CONNECTED_OK && evseL->isConnected() == CONNECTED_OK)
        {
            return ACTIVE_DUAL;
        }
        else if (evseR->isConnected() == CONNECTED_OK && evseL->isConnected() == CONNECTED_NOT)
        {
            return ACTIVE_R;
        }
        else if (evseL->isConnected() == CONNECTED_OK && evseR->isConnected() == CONNECTED_NOT)
        {
            return ACTIVE_L;
        }
        else
        {
            return ACTIVE_0;
        }
    }

    void dynamicLoadProcess()
    {
        if (lastMode != getMode())
        {
            lastMode = getMode();
            switch (lastMode)
            {
            case ACTIVE_0:
            {
                maxCurrentR = maxCurrentL = 0;
                break;
            }
            case ACTIVE_R:
            {
                maxCurrentR = evseMaxCurrent;
                maxCurrentL = 0;
                break;
            }
            case ACTIVE_L:
            {
                maxCurrentR = 0;
                maxCurrentL = evseMaxCurrent;
                break;
            }
            case ACTIVE_DUAL:
            {
                uint8_t evseMaxCurrHalf = evseMaxCurrent / 2;
                uint8_t ppLimitR = evseR->PP_READ();
                uint8_t ppLimitL = evseL->PP_READ();
                maxCurrentR = maxCurrentL = evseMaxCurrHalf;
                if (ppLimitL < evseMaxCurrHalf)
                {
                    maxCurrentR += evseMaxCurrHalf - ppLimitL;
                }
                else if (ppLimitR < evseMaxCurrHalf)
                {
                    maxCurrentL += evseMaxCurrHalf - ppLimitR;
                }
                break;
            }
            default:
            {
                DEBUG_LN_TYPE2_CONTROLLER("%s", "WARNING wrong dynamic load state");
                maxCurrentR = maxCurrentL = 0;
                break;
            }
            }
        }

        DEBUG_LN_TYPE2_CONTROLLER("MODE: %u", lastMode);

        evseR->setMaxCurrent(maxCurrentR);
        evseL->setMaxCurrent(maxCurrentL);
    }

    void loop()
    {
        now = millis();
        DEBUG_LN_TYPE2_CONTROLLER("%s", "START");
        setStates();
        EXECUTE();
        dynamicLoadProcess();
        DEBUG_LN_TYPE2_CONTROLLER("looptime: %lu", millis() - now);
    }

    bool isConnected(uint8_t connectorId)
    {
        switch (connectorId)
        {
        case 1:
            return evseL->isConnected();
        case 2:
            return evseL->isConnected();
        default:
            return false;
        };
    }

}