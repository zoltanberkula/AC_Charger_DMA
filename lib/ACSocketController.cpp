#include "ACSocketController.h"

namespace ACSocketController
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

    ChargeProcessController *evseR = nullptr; // ChargeProcessController 1 initialization
    ChargeProcessController *evseL = nullptr; // ChargeProcessController 2 initialization

    /**
     * The function hmiBtn_INIT sets up event handlers for the start and stop buttons, printing debug
     * messages when the buttons are pressed or released.
     */
    void hmiBtn_INIT()
    {
        startBtn->onHigh = [](void *)
        { DEBUG_LN_ACSOCKET_CONTROLLER("%s", "Start Btn HIGH");
        DEBUG_LN_ACSOCKET_CONTROLLER("%s", "Start Btn HIGH"); };

        startBtn->onLow = [](void *)
        { DEBUG_LN_ACSOCKET_CONTROLLER("%s", "Start Btn LOW");
        DEBUG_LN_ACSOCKET_CONTROLLER("%s", "Start Btn LOW"); };

        stopBtn->onHigh = [](void *)
        { DEBUG_LN_ACSOCKET_CONTROLLER("%s", "Stop Btn HIGH");
        DEBUG_LN_ACSOCKET_CONTROLLER("%s", "Stop Btn LOW"); };

        stopBtn->onLow = [](void *)
        { DEBUG_LN_ACSOCKET_CONTROLLER("%s", "Stop Btn LOW");
        DEBUG_LN_ACSOCKET_CONTROLLER("%s", "Stop Btn LOW"); };
    };

    /**
     * The function `cpSampler` is an interrupt service routine (ISR) that reads analog values from
     * pins, stores them in buffers, and resets the buffer indices when they reach the buffer size.
     */
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

    /**
     * The function "getPP" returns the value of ppSample_R if the input "side" is SIDE_RIGHT, and
     * returns the value of ppSample_L if the input "side" is SIDE_LEFT.
     *
     * @param side The parameter "side" is of type uint8_t, which means it is an 8-bit unsigned
     * integer. It is used to determine which side's ppSample value to return. The possible values for
     * "side" are defined as SIDE_RIGHT and SIDE_LEFT.
     *
     * @return a uint16_t value, which is the value of ppVal.
     */
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

    /**
     * The function "startGunR" sets a flag to indicate that the "GUNR" process has started.
     *
     * @param argc The parameter `argc` stands for "argument count" and represents the number of
     * command-line arguments passed to the program when it is executed.
     * @param argv argv is a pointer to an array of pointers to characters. It is used to pass command
     * line arguments to the main function. Each element in the array represents a command line
     * argument, where the first element (argv[0]) is the name of the program itself.
     *
     * @return an integer value of 0.
     */
    int startGunR(int argc, char **argv)
    {
        DEBUG_LN_ACSOCKET_CONTROLLER("%s", "GUNR Started");
        startGR = true;
        stopGR = false;
        return 0;
    }

    /**
     * The function "startGunL" sets a flag to indicate that the "GUNL" process has started.
     *
     * @param argc The parameter `argc` stands for "argument count" and represents the number of
     * command-line arguments passed to the program when it is executed.
     * @param argv argv is a pointer to an array of pointers to characters. It is used to pass command
     * line arguments to the main function. Each element in the array represents a command line
     * argument, where the first element (argv[0]) is the name of the program itself.
     *
     * @return an integer value of 0.
     */
    int startGunL(int argc, char **argv)
    {
        DEBUG_LN_ACSOCKET_CONTROLLER("%s", "GUNL Started");
        startGL = true;
        stopGL = false;
        return 0;
    }

    /**
     * The function "stopGunR" stops the "GUNR" process and updates the "stopGR" and "startGR"
     * variables accordingly.
     *
     * @param argc The parameter `argc` stands for "argument count" and represents the number of
     * command-line arguments passed to the program when it is executed.
     * @param argv argv is a pointer to an array of pointers to characters. It is used to pass command
     * line arguments to the main function. Each element in the array represents a command line
     * argument, with the first element (argv[0]) being the name of the program itself.
     *
     * @return an integer value of 0.
     */
    int stopGunR(int argc, char **argv)
    {
        DEBUG_LN_ACSOCKET_CONTROLLER("%s", "GUNR Stopped");
        stopGR = true;
        startGR = false;
        return 0;
    }

    /**
     * The function "stopGunL" stops the "GUNL" and returns 0.
     *
     * @param argc The parameter `argc` is an integer that represents the number of command-line
     * arguments passed to the program.
     * @param argv An array of strings representing the command-line arguments passed to the program.
     * The first element (argv[0]) is usually the name of the program itself. The remaining elements
     * (argv[1], argv[2], etc.) are the arguments passed to the program.
     *
     * @return an integer value of 0.
     */
    int stopGunL(int argc, char **argv)
    {
        DEBUG_LN_ACSOCKET_CONTROLLER("%s", "GUNL Stopped");
        stopGL = true;
        startGL = false;
        return 0;
    }

    /**
     * The function "reboot" restarts the program or device.
     *
     * @param argc The parameter `argc` stands for "argument count" and represents the number of
     * command-line arguments passed to the program. It is an integer value.
     * @param argv argv is a pointer to an array of pointers to null-terminated strings. Each string
     * represents one of the command-line arguments passed to the program. The last element of the
     * array is a null pointer.
     *
     * @return an integer value of 0.
     */
    int reboot(int argc, char **argv)
    {
        ESP.restart();
        return 0;
    }

    /**
     * The EXECUTE function checks the status of various variables and executes different actions based
     * on their values.
     */
    void EXECUTE()
    {
        shell.executeIfInput();
        if (startGR == true && stopGR == false)
        {
            DEBUG_LN_ACSOCKET_CONTROLLER("%s", evseR->name);
            evseR->setStates(stateP_R, stateN_R);
            evseR->CHARGE_PROCESS();
        }
        else if (stopGR == true)
        {
            // evseR->setMaxCurrent(0);
            DEBUG_LN_ACSOCKET_CONTROLLER("%s", "GUNR OFF");
            evseR->CONTACTOR_CONTROL(0);
            evseR->MOTOR_CONTROL(0);
            evseR->evseConfig.evFull = false;
        }

        if (startGL == true && stopGL == false)
        {
            DEBUG_LN_ACSOCKET_CONTROLLER("%s", evseL->name);
            evseL->setStates(stateP_L, stateN_L);
            evseL->CHARGE_PROCESS();
        }
        else if (stopGL == true)
        {
            // evseL->setMaxCurrent(0);
            DEBUG_LN_ACSOCKET_CONTROLLER("%s", "GUNL OFF");
            evseL->CONTACTOR_CONTROL(0);
            evseL->MOTOR_CONTROL(0);
            evseL->evseConfig.evFull = false;
        }
    }

    /**
     * The function "setStates" calculates the maximum and minimum values from two arrays, and based on
     * those values, assigns states to variables.
     */
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
        DEBUG_LN_ACSOCKET_CONTROLLER("%u", tempLMax);
        DEBUG_LN_ACSOCKET_CONTROLLER("%u", tempRMax);

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
            DEBUG_LN_ACSOCKET_CONTROLLER("%u", tempLMin);
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
            DEBUG_LN_ACSOCKET_CONTROLLER("%u", tempRMin);
            stateN_R = -1; // Unknown ERROR
        }
    }

    /**
     * The setup function initializes various parameters and objects for two EVSEs (Electric Vehicle
     * Supply Equipment) in a C++ program.
     */
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

        evseR = new ChargeProcessController(evseRCFG, "evseR", SIDE_RIGHT);
        evseL = new ChargeProcessController(evseLCFG, "evseL", SIDE_LEFT);
        evseR->INIT(); // EVSE1 Initialization
        evseL->INIT(); // EVSE2 Initialization
    }

    /**
     * The function `getMode()` returns the mode of operation based on the connection status of two
     * EVSE devices.
     *
     * @return The function `getMode()` returns a value of type `uint8_t`, which represents the mode of
     * operation. The possible return values are:
     */
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

    /**
     * The function "dynamicLoadDistribution" adjusts the maximum current for the right and left EVSEs
     * based on the current mode of operation.
     */
    void dynamicLoadDistribution()
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
                DEBUG_LN_ACSOCKET_CONTROLLER("%s", "WARNING wrong dynamic load state");
                maxCurrentR = maxCurrentL = 0;
                break;
            }
            }
        }

        DEBUG_LN_ACSOCKET_CONTROLLER("MODE: %u", lastMode);

        evseR->setMaxCurrent(maxCurrentR);
        evseL->setMaxCurrent(maxCurrentL);
    }

    /**
     * The function "loop" performs a series of tasks, including setting states, executing actions, and
     * dynamically distributing load, and then prints the time it took to complete these tasks.
     */
    void loop()
    {
        now = millis();
        DEBUG_LN_ACSOCKET_CONTROLLER("%s", "START");
        setStates();
        EXECUTE();
        dynamicLoadDistribution();
        DEBUG_LN_ACSOCKET_CONTROLLER("looptime: %lu", millis() - now);
    }

    /**
     * The function `isConnected` checks if a connector is connected based on its ID.
     *
     * @param connectorId The parameter `connectorId` is of type `uint8_t`, which is an unsigned 8-bit
     * integer. It represents the ID of a connector.
     *
     * @return a boolean value.
     */
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