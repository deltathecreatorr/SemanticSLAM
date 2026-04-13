#ifndef ENCODERHANDLER_HPP
#define ENCODERHANDLER_HPP

#include <Arduino.h>

struct WheelTicks {
    long fl;
    long rl;
    long fr;
    long rr;
};

class EncoderHandler {
    public:
        static void initEncoders();
        static void resetEncoders();
        static WheelTicks getEncoderTicks();

    private:
        static const int RL_A = 21; static const int RL_B = 20;
        static const int FL_A = 14; static const int FL_B = 15; 
        static const int RR_A = 27; static const int RR_B = 28;
        static const int FR_A = 11; static const int FR_B = 12;

        static volatile long fl_ticks;
        static volatile long rl_ticks;
        static volatile long fr_ticks;
        static volatile long rr_ticks;

        static void __not_in_flash_func(fl_isr)();
        static void __not_in_flash_func(rl_isr)();
        static void __not_in_flash_func(fr_isr)();
        static void __not_in_flash_func(rr_isr)();
};

#endif