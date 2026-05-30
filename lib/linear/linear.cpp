#include "linear.h"

LinearCar::LinearCar()
    : stepper(MOTOR_STEPS, PIN_DIR, PIN_STEP)
{}

void LinearCar::setup()
{
    pinMode(PIN_SLEEP, OUTPUT);
    digitalWrite(PIN_SLEEP, HIGH);
    pinMode(PIN_RESET, OUTPUT);
    digitalWrite(PIN_RESET, HIGH);
    pinMode(PIN_ENDSTOP, INPUT_PULLUP);

    preferences.begin("linear_car", false);
    limitePassos = preferences.getLong("limitePassos", 1750);
    delta        = preferences.getLong("delta", 1);
    motorRpm     = preferences.getFloat("rpm", MOTOR_RPM);

    stepper.begin(motorRpm);
    stepper.enable();
    stepper.setMicrostep(1);
}

void LinearCar::step_loop()
{
    Estado estadoLocal;
    taskENTER_CRITICAL(&stateMux);
    estadoLocal = estado;
    taskEXIT_CRITICAL(&stateMux);

    if (estadoLocal == HOMING) {
        handleHoming();
        return;
    }

    updateStateFromCommand();
    runMovement();
    stepper.nextAction();
}

void LinearCar::handleHoming()
{
    taskENTER_CRITICAL(&stateMux);
    bool stopRequested = (comando == STOP);
    taskEXIT_CRITICAL(&stateMux);

    if (stopRequested) {
        taskENTER_CRITICAL(&stateMux);
        estado = PARADO;
        taskEXIT_CRITICAL(&stateMux);
        stepper.startMove(0);
        return;
    }

    if (isEndstopTriggered()) {
        long zero;
        taskENTER_CRITICAL(&stateMux);
        estado = PARADO;
        zero   = homing_zero_step;
        steps  = zero;
        taskEXIT_CRITICAL(&stateMux);

        stepper.startMove(-1);
        Serial.print("Homing completo. Passos definidos para: ");
        Serial.println(zero);
        return;
    }

    stepper.startMove(1);
    stepper.nextAction();
}

void LinearCar::updateStateFromCommand()
{
    taskENTER_CRITICAL(&stateMux);
    if (comando == PLAY) {
        if (steps >= (homing_zero_step - 50))
            estado = INDO;
        else if (steps <= (homing_zero_step - limitePassos))
            estado = VOLTANDO;
        else
            estado = lastDirection;
        taskEXIT_CRITICAL(&stateMux);
        return;
    }

    if (comando == BYPASS) {
        estado = controle;
        taskEXIT_CRITICAL(&stateMux);
        return;
    }

    if (comando == STOP)
        estado = PARADO;
    taskEXIT_CRITICAL(&stateMux);
}

void LinearCar::runMovement()
{
    Estado estadoLocal;
    taskENTER_CRITICAL(&stateMux);
    estadoLocal = estado;
    taskEXIT_CRITICAL(&stateMux);

    if (estadoLocal == INDO) {
        stepper.startMove(-1);
        taskENTER_CRITICAL(&stateMux);
        steps--;
        lastDirection = INDO;
        taskEXIT_CRITICAL(&stateMux);
    } else if (estadoLocal == VOLTANDO) {
        stepper.startMove(1);
        taskENTER_CRITICAL(&stateMux);
        steps++;
        lastDirection = VOLTANDO;
        taskEXIT_CRITICAL(&stateMux);
    } else {
        stepper.startMove(0);
    }
}

bool LinearCar::isEndstopTriggered() const
{
    return !digitalRead(PIN_ENDSTOP);
}

void LinearCar::setLimitePassos(long limite)
{
    taskENTER_CRITICAL(&stateMux);
    limitePassos = limite;
    taskEXIT_CRITICAL(&stateMux);
    preferences.putLong("limitePassos", limite);
}

long LinearCar::getLimitePassos() const
{
    taskENTER_CRITICAL(&stateMux);
    long result = limitePassos;
    taskEXIT_CRITICAL(&stateMux);
    return result;
}

void LinearCar::setDelta(long d)
{
    taskENTER_CRITICAL(&stateMux);
    delta = d;
    taskEXIT_CRITICAL(&stateMux);
    preferences.putLong("delta", d);
}

long LinearCar::getDelta() const
{
    taskENTER_CRITICAL(&stateMux);
    long result = delta;
    taskEXIT_CRITICAL(&stateMux);
    return result;
}

void LinearCar::setTargetStep(long targetSteps)
{
    stepper.startMove(targetSteps);
}

void LinearCar::setHomingZeroStep(long value)
{
    taskENTER_CRITICAL(&stateMux);
    homing_zero_step = value;
    taskEXIT_CRITICAL(&stateMux);
}

void LinearCar::setCommand(Comando value)
{
    taskENTER_CRITICAL(&stateMux);
    comando = value;
    taskEXIT_CRITICAL(&stateMux);
}

void LinearCar::startFollowingLine()
{
    taskENTER_CRITICAL(&stateMux);
    comando = PLAY;
    if (estado == HOMING || estado == PARADO) {
        estado = INDO;
    }
    taskEXIT_CRITICAL(&stateMux);
}

void LinearCar::setBypassControl(Estado value)
{
    taskENTER_CRITICAL(&stateMux);
    controle = value;
    comando  = BYPASS;
    taskEXIT_CRITICAL(&stateMux);
}

void LinearCar::requestHoming()
{
    taskENTER_CRITICAL(&stateMux);
    estado   = HOMING;
    comando  = BYPASS;
    controle = PARADO;
    taskEXIT_CRITICAL(&stateMux);
}

void LinearCar::setSpeed(float rpm)
{
    if (rpm < 1.0f)
        rpm = 1.0f;
    motorRpm = rpm;
    stepper.setRPM(motorRpm);
    preferences.putFloat("rpm", motorRpm);
}

float LinearCar::getSpeed() const
{
    return motorRpm;
}

long LinearCar::getSteps() const
{
    taskENTER_CRITICAL(&stateMux);
    long current = steps;
    taskEXIT_CRITICAL(&stateMux);
    return current;
}

long LinearCar::getRelativeSteps() const
{
    taskENTER_CRITICAL(&stateMux);
    long relative = steps - homing_zero_step;
    taskEXIT_CRITICAL(&stateMux);
    return relative;
}

float LinearCar::getRelativePosition() const
{
    return static_cast<float>(getRelativeSteps()) * MM_PER_STEP;
}

float LinearCar::getAbsolutePosition() const
{
    return static_cast<float>(getSteps()) * MM_PER_STEP;
}
