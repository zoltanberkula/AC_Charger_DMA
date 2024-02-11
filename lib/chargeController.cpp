#include <Arduino.h>
#include <math.h>

#include "chargeController.h"
//#include "Type2Controller.h"
#include "hal_config.h"

//#define EVSE_DUAL 0
namespace Type2Controller
{
    extern uint16_t getPP(uint8_t);
}
using namespace std;

void ChargeController::INIT()
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
uint8_t ChargeController::PP_READ()
{
    uint16_t ppVal = Type2Controller::getPP(id);
    DEBUG_LN_TYPE2_CONTROLLER("getPP %u", ppVal);

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
void ChargeController::setMaxCurrent(uint32_t current)
{
    PP_READ();
    DEBUG_LN_TYPE2_CONTROLLER("ppLimit: %u", ppLimit);
    DEBUG_LN_TYPE2_CONTROLLER("current: %u", current);
    DEBUG_LN_TYPE2_CONTROLLER("evseConfig.connectorMaxCurrent: %u", evseConfig.connectorMaxCurrent);
    current = _min(evseConfig.connectorMaxCurrent, current);
    current = _min(ppLimit, current);

    // pwm calculate
    uint32_t pwm = 0;

    if (current <= 50)
        pwm = (current * 100) / 6;
    else
        pwm = (current * 100) / 25 + 64;

    DEBUG_LN_TYPE2_CONTROLLER("%u", current);
    DEBUG_LN_TYPE2_CONTROLLER("%u", pwm);

    uint32_t pwmDutyValue = (pwm * PWM_MAX) / 1000; // 8 vs 10 bits?
    if (current == 0)
        pwmDutyValue = PWM_MAX;

    DEBUG_LN_TYPE2_CONTROLLER("%s  pwmdutyvalue  %u ", name, pwmDutyValue);

    ledcWrite(evseConfig.pwmChannel, pwmDutyValue);
};

///@brief function responsible for controlling the 3P Contactors
void ChargeController::CONTACTOR_CONTROL(bool state)
{
    if (state == 1)
    {
        digitalWrite(evseConfig.contactorPin, HIGH);
        DEBUG_LN_TYPE2_CONTROLLER("%s", "Contactor ON");
    }
    else
    {
        digitalWrite(evseConfig.contactorPin, LOW);
        DEBUG_LN_TYPE2_CONTROLLER("%s", "Contactor OFF");
    }
};

///@brief function responsible for controlling the motor lock mechanism
void ChargeController::MOTOR_CONTROL(bool state)
{
    if (state == 1)
    {
        digitalWrite(evseConfig.motorPin, HIGH);
        DEBUG_LN_TYPE2_CONTROLLER("%s", "Motor ON");
    }
    else
    {
        digitalWrite(evseConfig.motorPin, LOW);
        DEBUG_LN_TYPE2_CONTROLLER("%s", "Motor OFF");
    }
};

///@brief function responsible for monitoring the state of the motor feedback PIN
bool ChargeController::MOTOR_LOCK_FB()
{
    uint16_t lockFB = digitalRead(evseConfig.motorFB);
    DEBUG_LN_TYPE2_CONTROLLER("lockFB %u", lockFB);
    return lockFB;
};

///@brief function responsible for monitoring the state of the motor feedback PIN
bool ChargeController::EM_STOP_FB()
{
    return digitalRead(evseConfig.emsPin);
};

int8_t ChargeController::isConnected()
{
    if (stateP == PHASE_3V || stateP == PHASE_6V || stateP == PHASE_9V)
    {
        // if(stateN == PHASE_N12V)
        DEBUG_LN_TYPE2_CONTROLLER("%s", "CONNECTED_OK");
        return CONNECTED_OK;
        // else
        //     return CONNECTED_ERROR;
    }
    else
    {
        DEBUG_LN_TYPE2_CONTROLLER("%s", "CONNECTED_NOT");
        return CONNECTED_NOT;
    }
}

int8_t ChargeController::isCharging()
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

void ChargeController::setStates(int8_t _stateP, int8_t _stateN)
{
    stateP = _stateP;
    stateN = _stateN;
}

///@brief function responsible for managing the peripherals according to the current state of the charging process
void ChargeController::CHARGE_PROCESS()
{
    DEBUG_LN_TYPE2_CONTROLLER("%s", name);
    DEBUG_LN_TYPE2_CONTROLLER("stateP %d", stateP);
    DEBUG_LN_TYPE2_CONTROLLER("stateN %d", stateN);

    switch (stateP)
    {
    case PHASE_12V:
        DEBUG_LN_TYPE2_CONTROLLER("%s", "EV NOT CONNECTED");
        DEBUG_LN_TYPE2_CONTROLLER("%s", "STATE A 12V");
        MOTOR_CONTROL(0);
        CONTACTOR_CONTROL(0);
        // setMaxCurrent(0);
        evseConfig.evFull = false;
        break;
    case PHASE_9V:
        if (evseConfig.evFull == true)
        {
            DEBUG_LN_TYPE2_CONTROLLER("%s", "EV FULL");
            // setMaxCurrent(0);
            CONTACTOR_CONTROL(0);
            MOTOR_CONTROL(0);
            // evseConfig.evFull = false;
            break;
        };
        DEBUG_LN_TYPE2_CONTROLLER("%s", "EV CONNECTED");
        DEBUG_LN_TYPE2_CONTROLLER("%s", "STATE B 9V");
        MOTOR_CONTROL(1);
        break;
    case PHASE_6V:
        DEBUG_LN_TYPE2_CONTROLLER("%s", "EV CHARGING");
        DEBUG_LN_TYPE2_CONTROLLER("%s", "STATE C 6V");
        MOTOR_CONTROL(1);
        if (MOTOR_LOCK_FB() == 0)
        {
            CONTACTOR_CONTROL(1);
        };
        evseConfig.evFull = true;
        break;
    case PHASE_3V:
        DEBUG_LN_TYPE2_CONTROLLER("%s", "EV CHARGING VENTILATION REQUIRED");
        DEBUG_LN_TYPE2_CONTROLLER("%s", "STATE D 3V");
        evseConfig.evFull = true;
        break;
    case PHASE_0V:
        DEBUG_LN_TYPE2_CONTROLLER("%s", "EV NOT AVAILABLE");
        DEBUG_LN_TYPE2_CONTROLLER("%s", "STATE E 0V");
        break;
    case PHASE_UE:
        DEBUG_LN_TYPE2_CONTROLLER("%s", "UNKNOWN ERROR");
        DEBUG_LN_TYPE2_CONTROLLER("%s", "STATE UE");
        CONTACTOR_CONTROL(0);
        MOTOR_CONTROL(0);
        break;
    case PHASE_N12V:
        DEBUG_LN_TYPE2_CONTROLLER("%s", "PHASE -12V");
        DEBUG_LN_TYPE2_CONTROLLER("%s", "STATE F N/A");
        break;
    default:
        DEBUG_LN_TYPE2_CONTROLLER("%s", "INVALID PHASE");
        CONTACTOR_CONTROL(0);
        MOTOR_CONTROL(0);
        break;
    }
    return;
};
