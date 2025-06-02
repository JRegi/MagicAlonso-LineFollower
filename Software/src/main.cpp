#include <sensor_array.h>
#include <motor.h>

#define NUM_SENSORS 8
#define PIN_S0 4
#define PIN_S1 25
#define PIN_S2 33
#define PIN_S3 32
#define PIN_SIG 34
#define PIN_LED 23

#define SETPOINT 350
#define KP 0.1
#define KD 0.4

#define MAX_PID 4900

#define MAX_SPEED 100
#define MIN_SPEED 30
#define BASE_SPEED 100

#define PIN_MR1 16
#define PIN_MR2 17
#define PIN_ML1 27
#define PIN_ML2 26

#define CHANNEL_MR1 0
#define CHANNEL_MR2 1
#define CHANNEL_ML1 2
#define CHANNEL_ML2 3

#define PWM_FREQUENCY 1000
#define PWM_RESOLUTION 8

SensorArray QTRArray(NUM_SENSORS, PIN_S0, PIN_S1, PIN_S2, PIN_S3, PIN_SIG, PIN_LED, SensorArray::BLACK);

MotorPair motors(PIN_MR1, PIN_MR2, CHANNEL_MR1, CHANNEL_MR2, PIN_ML1, PIN_ML2,
                 CHANNEL_ML1, CHANNEL_ML2, PWM_FREQUENCY, PWM_RESOLUTION);

static uint16_t lastError;

void PIDController()
{
    uint16_t position = QTRArray.ReadPosition();

    int16_t proportional = position - SETPOINT;
    int16_t derivative = proportional - lastError;
    int16_t pid = (proportional * KP) + (derivative * KD);

    lastError = proportional;

    pid = constrain(pid, -MAX_SPEED, MAX_PID);

    uint8_t speedRight = map(-pid, 0, -MAX_PID, 0, MAX_SPEED);
    uint8_t speedLeft = map(pid, 0, MAX_PID, 0, MAX_SPEED);

    speedRight = constrain(speedRight, 0, MAX_SPEED);
    speedLeft = constrain(speedLeft, 0, MAX_SPEED);

    if (position == 0)
    {
        motors.Brake();
    }
    else
    {
        motors.MoveForward(speedRight, speedLeft);
    }
}

void setup()
{
}

void loop()
{
    QTRArray.CalibrateSensors();
    PIDController();
}