#pragma once

#include <Arduino.h>
#include<MA.h>

class BALL{
    public:
        MA ball_x;
        MA ball_y;
        MA far__;
        MA ball_get_val_1;
        MA ball_get_val_2;
        float ang;
        float far;
        float far_old = 0;
        float x_pos;
        float y_pos;
        int flag = 1;
        float dx;
        float far_;
        int get_val;
        int ball_get;
        int get_th = 80;
        BALL();
        int getBallposition();
        void print();
        void begin();
        void get_resister_1(int);
        void get_resister_2(int);

    private:
        int get_1;
        int get_2;
};