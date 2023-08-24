
#pragma once
#include "Arduino.h"


class ANADRUM_Channel
{
public:
    ANADRUM_Channel();
    void begin(int ENABLE, int DIR, int PULSE, int FIRE, int STOP);
    void loop();
    void fire();
    int steps_to_move;
    int shots_fired;
    long start_move_millis;
    boolean ready = false;

private:
    int pin_en;
    int pin_dir;
    int pin_pulse;
    int pin_fire;
    int pin_stop;
    boolean in_step;
};

ANADRUM_Channel::ANADRUM_Channel()
{
}

//----------------------------------------------------------------------------------------
void ANADRUM_Channel::begin(int ENABLE, int DIR, int PULSE, int FIRE, int STOP)
{
    pin_en = ENABLE;
    pin_dir = DIR;
    pin_pulse = PULSE;
    pin_fire = FIRE;
    pin_stop = STOP;

    pinMode(pin_en, OUTPUT);
    pinMode(pin_dir, OUTPUT);
    pinMode(pin_pulse, OUTPUT);
    pinMode(pin_fire, OUTPUT);
    pinMode(pin_stop, INPUT);

    digitalWrite(pin_fire, LOW);
    digitalWrite(pin_en, LOW);
    digitalWrite(pin_pulse, LOW);
    digitalWrite(pin_dir, LOW);
}

//----------------------------------------------------------------------------------------
void ANADRUM_Channel::fire()
{
    shots_fired++;
    start_move_millis = millis() + WAIT_TIME_AFTER_FIRE;
    steps_to_move = MIN_STEPS_TO_MOVE;
    ready = false;
    in_step = true;
}

//----------------------------------------------------------------------------------------
void ANADRUM_Channel::loop()
{
    if (steps_to_move)
    {
        digitalWrite(pin_en, LOW);

        // delay motor movement after shot
        if (millis() > start_move_millis)
        {
            if (in_step)
            {
                digitalWrite(pin_pulse, HIGH);
                in_step = false;
            }
            else
            {
                digitalWrite(pin_pulse, LOW);
                in_step = true;
                steps_to_move--;
            }
        }
    }

    // only check for next endstop when we're sure we have moved out of current one
    if (steps_to_move < MIN_STEPS_TO_MOVE / 2)
    {
        ready = digitalRead(pin_stop);
        if (ready)
        {
            steps_to_move = 0;
        }
        else
        {
            steps_to_move++;
        }

        if (!steps_to_move)
        {
            digitalWrite(pin_en, HIGH);
        }
    }
}