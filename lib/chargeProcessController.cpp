#include <Arduino.h>
#include <math.h>

#include "chargeProcessController.h"
// #include "ACSocketController.h"
#include "hal_config.h"

// #define EVSE_DUAL 0
namespace ACSocketController
{
    extern uint16_t getPP(uint8_t);
}
using namespace std;

void ChargeProcessController::INIT()
{
    pinMode(evseConfig.cpPin, OUTPUT);
    pinMode(evseConfig.cpDetPin, INPUT);
    pinMode(evseConfig.ppPin, INPUT);
    pinMode(evseConfig.motorPin, OUTPUT);
    pinMode(evseConfig.contactorPin, OUTPUT);
    pinMode(evseConfig.motorFB, INPUT);
    ledcSetup(evseConfig.pwmChannel, CP_R_PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(evseConfig.cpPin, evseConfig.pwmChannel);
    ledcWrite(evseConfig.pwmChannel, PWM_MAX);
    pinMode(EN_24V_PIN, OUTPUT);
    digitalWrite(EN_24V_PIN, HIGH);
    digitalWrite(evseConfig.motorPin, LOW); // NEW MODIFICATION
    digitalWrite(evseConfig.contactorPin, LOW);
    pinMode(EM_STOP_PIN, INPUT);
};

///@brief function responsible for reading the resistance of the Proximity Pilot
///@return ppLimit that holds the maximum amount of current that the EV cable can handle
/**
 * The function `PP_READ()` reads the power factor value from an AC socket controller and determines
 * the appropriate power limit based on the value.
 *
 * @return the value of the variable ppLimit, which is of type uint8_t.
 */
uint8_t ChargeProcessController::PP_READ()
{
    uint16_t ppVal = ACSocketController::getPP(id);
    DEBUG_LN_ACSOCKET_CONTROLLER("getPP %u", ppVal);

    if (ppVal >= 2300 && ppVal <= 2600)
        ppLimit = PP_LIMIT_13;
    else if (ppVal >= 1500 && ppVal <= 1560)
        ppLimit = PP_LIMIT_20;
    else if (ppVal >= 580 && ppVal <= 640)
        ppLimit = PP_LIMIT_32;
    else if (ppVal >= 220 && ppVal <= 300)
        ppLimit = PP_LIMIT_63;
    else
        ppLimit = PP_LIMIT_0;

    return ppLimit;
};

///@brief function responsible for setting the maximum current that the EVSE can handle
///@return the maximum current
/**
 * The function sets the maximum current for charging and calculates the corresponding PWM duty value
 * for controlling the charging process.
 *
 * @param current The "current" parameter represents the current value that needs to be set. It is of
 * type uint32_t, which means it is an unsigned 32-bit integer.
 */
void ChargeProcessController::setMaxCurrent(uint32_t current)
{
    PP_READ();
    DEBUG_LN_ACSOCKET_CONTROLLER("ppLimit: %u", ppLimit);
    DEBUG_LN_ACSOCKET_CONTROLLER("current: %u", current);
    DEBUG_LN_ACSOCKET_CONTROLLER("evseConfig.connectorMaxCurrent: %u", evseConfig.connectorMaxCurrent);
    current = _min(evseConfig.connectorMaxCurrent, current);
    current = _min(ppLimit, current);

    // pwm calculate
    uint32_t pwm = 0;

    if (current <= 50)
        pwm = (current * 100) / 6;
    else
        pwm = (current * 100) / 25 + 64;

    DEBUG_LN_ACSOCKET_CONTROLLER("%u", current);
    DEBUG_LN_ACSOCKET_CONTROLLER("%u", pwm);

    uint32_t pwmDutyValue = (pwm * PWM_MAX) / 1000; // 8 vs 10 bits?
    if (current == 0)
        pwmDutyValue = PWM_MAX;

    DEBUG_LN_ACSOCKET_CONTROLLER("%s  pwmdutyvalue  %u ", name, pwmDutyValue);

    ledcWrite(evseConfig.pwmChannel, pwmDutyValue);
};

///@brief function responsible for controlling the 3P Contactors
/**
 * The function `CONTACTOR_CONTROL` controls the state of a contactor pin based on the input state.
 *
 * @param state The "state" parameter is a boolean value that determines whether the contactor should
 * be turned on or off. If "state" is true (1), the contactor will be turned on by setting the
 * corresponding pin to HIGH. If "state" is false (0), the contactor will be
 */
void ChargeProcessController::CONTACTOR_CONTROL(bool state)
{
    if (state == 1)
    {
        digitalWrite(evseConfig.contactorPin, HIGH);
        DEBUG_LN_ACSOCKET_CONTROLLER("%s", "Contactor ON");
    }
    else
    {
        digitalWrite(evseConfig.contactorPin, LOW);
        DEBUG_LN_ACSOCKET_CONTROLLER("%s", "Contactor OFF");
    }
};

///@brief function responsible for controlling the motor lock mechanism
/**
 * The function `MOTOR_CONTROL` controls the state of a motor based on the input parameter `state`.
 *
 * @param state The "state" parameter is a boolean value that determines whether the motor should be
 * turned on or off. If "state" is true (1), the motor will be turned on by setting the corresponding
 * pin to HIGH. If "state" is false (0), the motor will be turned off by
 */
void ChargeProcessController::MOTOR_CONTROL(bool state)
{
    if (state == 1)
    {
        digitalWrite(evseConfig.motorPin, HIGH);
        DEBUG_LN_ACSOCKET_CONTROLLER("%s", "Motor ON");
    }
    else
    {
        digitalWrite(evseConfig.motorPin, LOW);
        DEBUG_LN_ACSOCKET_CONTROLLER("%s", "Motor OFF");
    }
};

///@brief function responsible for monitoring the state of the motor feedback PIN
/**
 * The function `MOTOR_LOCK_FB()` reads the digital input pin `evseConfig.motorFB` and returns the
 * value.
 *
 * @return the value of the variable "lockFB", which is of type uint16_t.
 */
bool ChargeProcessController::MOTOR_LOCK_FB()
{
    uint16_t lockFB = digitalRead(evseConfig.motorFB);
    DEBUG_LN_ACSOCKET_CONTROLLER("lockFB %u", lockFB);
    return lockFB;
};

///@brief function responsible for monitoring the state of the motor feedback PIN
/**
 * The function returns the value of the digital pin connected to the emergency stop button.
 *
 * @return the value of the digitalRead(evseConfig.emsPin) function.
 */
bool ChargeProcessController::EM_STOP_FB()
{
    return digitalRead(evseConfig.emsPin);
};

/**
 * The function `isConnected()` checks if the charge process controller is connected and returns a
 * status code.
 *
 * @return either CONNECTED_OK or CONNECTED_NOT, depending on the conditions specified in the if-else
 * statement.
 */
int8_t ChargeProcessController::isConnected()
{
    if (stateP == PHASE_3V || stateP == PHASE_6V || stateP == PHASE_9V)
    {
        // if(stateN == PHASE_N12V)
        DEBUG_LN_ACSOCKET_CONTROLLER("%s", "CONNECTED_OK");
        return CONNECTED_OK;
        // else
        //     return CONNECTED_ERROR;
    }
    else
    {
        DEBUG_LN_ACSOCKET_CONTROLLER("%s", "CONNECTED_NOT");
        return CONNECTED_NOT;
    }
}

/**
 * The function isCharging checks if the charge process controller is in the charging state.
 *
 * @return the value of the macro CONNECTED_CHARGING if the condition (stateP == PHASE_3V || stateP ==
 * PHASE_6V) is true. Otherwise, it is returning the value of the macro CONNECTED_NOT.
 */
int8_t ChargeProcessController::isCharging()
{
    if (stateP == PHASE_3V || stateP == PHASE_6V)
    {
        // if(stateN == PHASE_N12V)
        return CONNECTED_CHARGING;
        // else
        //     return CONNECTED_ERROR;
    }
    else
        return CONNECTED_NOT;
}

/**
 * The function "setStates" sets the values of the variables "stateP" and "stateN" to the provided
 * values.
 *
 * @param _stateP The _stateP parameter represents the state of the positive charge process. It is of
 * type int8_t, which means it can hold integer values from -128 to 127.
 * @param _stateN The "_stateN" parameter is an integer variable that represents the state of the
 * negative terminal in a charge process.
 */
void ChargeProcessController::setStates(int8_t _stateP, int8_t _stateN)
{
    stateP = _stateP;
    stateN = _stateN;
}

///@brief function responsible for managing the peripherals according to the current state of the charging process
/**
 * The function `CHARGE_PROCESS()` controls the charging process for an electric vehicle based on the
 * current state of the charging system.
 *
 * @return The function does not return any value. It has a return type of void, which means it does
 * not return anything.
 */
void ChargeProcessController::CHARGE_PROCESS()
{
    DEBUG_LN_ACSOCKET_CONTROLLER("%s", name);
    DEBUG_LN_ACSOCKET_CONTROLLER("stateP %d", stateP);
    DEBUG_LN_ACSOCKET_CONTROLLER("stateN %d", stateN);

    switch (stateP)
    {
    case PHASE_12V:
        DEBUG_LN_ACSOCKET_CONTROLLER("%s", "EV NOT CONNECTED");
        DEBUG_LN_ACSOCKET_CONTROLLER("%s", "STATE A 12V");
        MOTOR_CONTROL(0);
        CONTACTOR_CONTROL(0);
        // setMaxCurrent(0);
        evseConfig.evFull = false;
        break;
    case PHASE_9V:
        if (evseConfig.evFull == true)
        {
            DEBUG_LN_ACSOCKET_CONTROLLER("%s", "EV FULL");
            // setMaxCurrent(0);
            CONTACTOR_CONTROL(0);
            MOTOR_CONTROL(0);
            // evseConfig.evFull = false;
            break;
        };
        DEBUG_LN_ACSOCKET_CONTROLLER("%s", "EV CONNECTED");
        DEBUG_LN_ACSOCKET_CONTROLLER("%s", "STATE B 9V");
        MOTOR_CONTROL(1);
        break;
    case PHASE_6V:
        DEBUG_LN_ACSOCKET_CONTROLLER("%s", "EV CHARGING");
        DEBUG_LN_ACSOCKET_CONTROLLER("%s", "STATE C 6V");
        MOTOR_CONTROL(1);
        if (MOTOR_LOCK_FB() == 0)
        {
            CONTACTOR_CONTROL(1);
        };
        evseConfig.evFull = true;
        break;
    case PHASE_3V:
        DEBUG_LN_ACSOCKET_CONTROLLER("%s", "EV CHARGING VENTILATION REQUIRED");
        DEBUG_LN_ACSOCKET_CONTROLLER("%s", "STATE D 3V");
        evseConfig.evFull = true;
        break;
    case PHASE_0V:
        DEBUG_LN_ACSOCKET_CONTROLLER("%s", "EV NOT AVAILABLE");
        DEBUG_LN_ACSOCKET_CONTROLLER("%s", "STATE E 0V");
        break;
    case PHASE_UE:
        DEBUG_LN_ACSOCKET_CONTROLLER("%s", "UNKNOWN ERROR");
        DEBUG_LN_ACSOCKET_CONTROLLER("%s", "STATE UE");
        CONTACTOR_CONTROL(0);
        MOTOR_CONTROL(0);
        break;
    case PHASE_N12V:
        DEBUG_LN_ACSOCKET_CONTROLLER("%s", "PHASE -12V");
        DEBUG_LN_ACSOCKET_CONTROLLER("%s", "STATE F N/A");
        break;
    default:
        DEBUG_LN_ACSOCKET_CONTROLLER("%s", "INVALID PHASE");
        CONTACTOR_CONTROL(0);
        MOTOR_CONTROL(0);
        break;
    }
    return;
};
