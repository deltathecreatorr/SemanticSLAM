#include "EncoderHandler.hpp"

int RL_A = 21; int RL_B = 20;
int FL_A = 14; int FL_B = 15; 
int RR_A = 27; int RR_B = 28;
int FR_A = 11; int FR_B = 12;

volatile long EncoderHandler::fl_ticks = 0;
volatile long EncoderHandler::rl_ticks = 0;
volatile long EncoderHandler::fr_ticks = 0;
volatile long EncoderHandler::rr_ticks = 0;

void __not_in_flash_func(EncoderHandler::fl_isr)() {
    if (digitalRead(FL_A) == digitalRead(FL_B)) fl_ticks++;
    else fl_ticks--;
}

void __not_in_flash_func(EncoderHandler::rl_isr)() {
    if (digitalRead(RL_A) == digitalRead(RL_B)) rl_ticks++;
    else rl_ticks--;
}

void __not_in_flash_func(EncoderHandler::fr_isr)() {
    if (digitalRead(FR_A) == digitalRead(FR_B)) fr_ticks--;
    else fr_ticks++;
}

void __not_in_flash_func(EncoderHandler::rr_isr)() {
    if (digitalRead(RR_A) == digitalRead(RR_B)) rr_ticks--;
    else rr_ticks++;
}

void EncoderHandler::initEncoders() {
    pinMode(FL_A, INPUT); pinMode(FL_B, INPUT);
    pinMode(RL_A, INPUT); pinMode(RL_B, INPUT);
    pinMode(FR_A, INPUT); pinMode(FR_B, INPUT);
    pinMode(RR_A, INPUT); pinMode(RR_B, INPUT);

    attachInterrupt(digitalPinToInterrupt(FL_A), fl_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RL_A), rl_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(FR_A), fr_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RR_A), rr_isr, CHANGE);
}

void EncoderHandler::resetEncoders() {
    noInterrupts();
    fl_ticks = 0; rl_ticks = 0; fr_ticks = 0; rr_ticks = 0;
    interrupts();
}

WheelTicks EncoderHandler::getEncoderTicks() {
    noInterrupts();
    WheelTicks current = {fl_ticks, rl_ticks, fr_ticks, rr_ticks};
    interrupts();
    return current;
}
