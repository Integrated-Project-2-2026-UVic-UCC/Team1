#ifndef SERVO_CTRL_H
#define SERVO_CTRL_H
#include <Arduino.h>
#include <math.h> // Corregido: es math.h, no mat.h

class ServoCtrl
{
public:
    ServoCtrl(int pin, int channel, int resolution);
    void write(float rads);
    void writeDegrees(float degrees);

private:
    int _pin;
    int _channel;
    int _resolution;
};

ServoCtrl::ServoCtrl(int pin, int channel, int resolution)
{
    _pin = pin;
    _channel = channel;
    _resolution = resolution;
    ledcSetup(_channel, 50, _resolution);
    ledcAttachPin(_pin, _channel);
}

void ServoCtrl::write(float rads)
{
    const float min_us = 500.0;
    const float max_us = 2500.0;
    const float period_us = 20000.0; // 50 Hz

    const float max_servo_rads = (3.0 * M_PI) / 2.0 + 0.09; // aprox 4.71 rads mes offset de tolerancies

    // capar rangos
    if (rads < 0)
        rads = 0;
    if (rads > max_servo_rads)
        rads = max_servo_rads;

    // TODO: añadir offset por rangos

    // FÓRMULA CORREGIDA:
    // Calculamos el pulso basándonos en que el máximo recorrido es 270 grados
    float pulse = min_us + (rads / max_servo_rads) * (max_us - min_us);

    float duty = pulse / period_us;

    // Cálculo a 16 bits seguro
    uint16_t max_duty_val = (1 << _resolution) - 1;
    uint16_t duty_bits = (uint32_t)(duty * max_duty_val);

    ledcWrite(_channel, duty_bits);
}

void ServoCtrl::writeDegrees(float degrees)
{
    write(degrees * (M_PI / 180.0));
}
#endif