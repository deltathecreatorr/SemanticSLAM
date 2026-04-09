#include "EncoderHandler.hpp"
#include <pico/platform.h> 

const int FL_A = 20; const int FL_B = 21;
const int RL_A = 15; const int RL_B = 14; 
const int FR_A = 27; const int FR_B = 28;
const int RR_A = 11; const int RR_B = 12;

volatile long fl_total = 0;
volatile long rl_total = 0;
volatile long fr_total = 0;
volatile long rr_total = 0;

void __not_in_flash_func(fl_isr)() {
    if (digitalRead(FL_A) == digitalRead(FL_B)) fl_total++;
    else fl_total--;
}

void __not_in_flash_func(rl_isr)() {
    if (digitalRead(RL_A) == digitalRead(RL_B)) rl_total++;
    else rl_total--;
}

void __not_in_flash_func(fr_isr)() {
    if (digitalRead(FR_A) == digitalRead(FR_B)) fr_total--;
    else fr_total++;
}

void __not_in_flash_func(rr_isr)() {
    if (digitalRead(RR_A) == digitalRead(RR_B)) rr_total--;
    else rr_total++;
}

void initEncoders() {
    pinMode(FL_A, INPUT_PULLUP); pinMode(FL_B, INPUT_PULLUP);
    pinMode(RL_A, INPUT_PULLUP); pinMode(RL_B, INPUT_PULLUP);
    pinMode(FR_A, INPUT_PULLUP); pinMode(FR_B, INPUT_PULLUP);
    pinMode(RR_A, INPUT_PULLUP); pinMode(RR_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(FL_A), fl_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RL_A), rl_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(FR_A), fr_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RR_A), rr_isr, CHANGE);
}

WheelTicks getEncoderTicks() {
    return {fl_total, rl_total, fr_total, rr_total};
}

void resetEncoders() {
    fl_total = 0; rl_total = 0; fr_total = 0; rr_total = 0;
}