#ifndef ENCODER_HANDLER_HPP
#define ENCODER_HANDLER_HPP

#include <Arduino.h>

struct WheelTicks {
    long fl_ticks;
    long rl_ticks;
    long fr_ticks;
    long rr_ticks;
};

void initEncoders();
WheelTicks getEncoderTicks();
void resetEncoders();

#endif