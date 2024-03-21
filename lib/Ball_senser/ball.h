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
        MA dx_;
        MA dy_;
        MA d_theta_;
        uint8_t data_byte[6];
        float ang;
        float far;
        float far_old = 0;
        float x_pos;
        float y_pos;
        float x_pos_old;
        float y_pos_old;
        float dx_dy;
        float d_theta;
        float dx;
        float dy;
        int flag = 1;
        float dp;
        float far_;
        int get_val;
        int ball_get;
        int get_th = 70;
        BALL();
        int getBallposition();
        void print();
        void begin();
        void get_resister_1(int);
        void get_resister_2(int);
        int get_1;
        int get_2;

    private:
};