#ifndef LINEAR_H
#define LINEAR_H

#include <Arduino.h>
#include <Preferences.h>
#include "DRV8825.h"

constexpr int   MOTOR_STEPS = 200;
constexpr int   MOTOR_RPM   = 30;
constexpr int   PIN_DIR     = 17;
constexpr int   PIN_STEP    = 18;
constexpr int   PIN_SLEEP   = 19;

constexpr int   PIN_ENDSTOP = 15;
constexpr int   STEPS_PER_REV = 200;
constexpr int   PULLEY_TEETH  = 16;
constexpr float BELT_PITCH    = 2.0f;
constexpr float MM_PER_STEP   = (PULLEY_TEETH * BELT_PITCH) / STEPS_PER_REV;

enum Estado  { PARADO, HOMING, INDO, VOLTANDO };
enum Comando { PLAY, STOP, BYPASS };

class LinearCar {
public:
    LinearCar();
    void  setup();
    void  step_loop();
    void  setTargetStep(long targetSteps);
    void  setHomingZeroStep(long value);
    void  setCommand(Comando value);
    void  startFollowingLine();
    void  setBypassControl(Estado value);
    void  requestHoming();
    void  setSpeed(float rpm);
    float getSpeed() const;
    void  setLimitePassos(long limite);
    long  getLimitePassos() const;
    void  setDelta(long d);
    long  getDelta() const;
    long  getSteps() const;
    long  getRelativeSteps() const;
    float getRelativePosition() const;
    float getAbsolutePosition() const;

private:
    DRV8825     stepper;
    Preferences preferences;
    mutable portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

    Comando comando  = STOP;
    Estado  estado   = PARADO;
    Estado  controle = PARADO;
    long    steps    = 0;
    long    homing_zero_step = 0;
    long    limitePassos     = 5000;
    long    delta            = 1;
    float   motorRpm         = static_cast<float>(MOTOR_RPM);
    Estado  lastDirection    = INDO;

    void handleHoming();
    void updateStateFromCommand();
    void runMovement();
    bool isEndstopTriggered() const;
};

extern LinearCar linearCar;

#endif
