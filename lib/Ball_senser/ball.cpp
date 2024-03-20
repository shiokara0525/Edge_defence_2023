#include<ball.h>

BALL::BALL(){
    ball_x.setLenth(10);
    ball_y.setLenth(10);
    dx_.setLenth(300);
    dy_.setLenth(300);
    d_theta_.setLenth(30);
    ball_get_val_1.setLenth(50);
    ball_get_val_2.setLenth(50);
    far__.setLenth(500);
}


void BALL::begin(){
    Serial8.begin(115200);
}


int BALL::getBallposition(){
    unsigned int contain[4];

    contain[0] = (uint16_t(data_byte[0]) << 8);
    contain[1] = (uint16_t(data_byte[1]));
    contain[2] = (uint16_t(data_byte[2]) << 8);
    contain[3] = (uint16_t(data_byte[3]));
    get_resister_1(data_byte[4]);
    get_resister_2(data_byte[5]);
    float x = ball_x.demandAve(int16_t(contain[0] | contain[1]));
    float y = ball_y.demandAve(int16_t(contain[2] | contain[3]));

    if(x == 0 && y == 0){
        flag = 0;
        return flag;
    }
    else{
        flag = 1;
    }
    x *= 0.05;
    y *= 0.05;
    if(0 < x){
        x_pos = 130 - x;
    }
    else{
        x_pos = -130 - x;
    }
    if(0 < y){
        y_pos = 130 - y;
    }
    else{
        y_pos = -130 - y;
    }
    if(150 < abs(x)){
        x = (x < 0 ? -150 : 150);
    }
    if(150 < abs(y)){
        y = (y < 0 ? -150 : 150);
    }

    if(30 < get_1 && 30 < get_2 && (40 < get_1 || 40 < get_2) && get_th < get_val){
        ball_get = 1;
    }
    else{
        ball_get = 0;
    }
    ang = degrees(atan2(y,x));
    far = sqrt(x_pos*x_pos + y_pos*y_pos);
    dp = (far - far_old);
    far_ = far__.demandAve(far);
    x_pos = x;
    y_pos = y;
    dx = dx_.demandAve((x_pos - x_pos_old) * 100);
    dy = dy_.demandAve((y_pos - y_pos_old) * 100);
    if(1.0 < abs(dx) || 1.0 < abs(dy)){
        dx_dy = dx / dy;
        d_theta = d_theta_.demandAve(degrees(atan2(dy,dx)));
    }
    else{
        dx_dy = 0;
        d_theta = 0;
    }
    if(dx_dy < -30){
        dx_dy = -30;
    }
    else if(30 < dx_dy){
        dx_dy = 30;
    }
    far_old = far;
    x_pos_old = x_pos;
    y_pos_old = y_pos;
    return flag;
}


void BALL::get_resister_1(int n){
    get_1 = ball_get_val_1.demandAve(n);
}


void BALL::get_resister_2(int n){
    get_2 = ball_get_val_2.demandAve(n);
}


void BALL::print(){
    Serial.println();
    Serial.print(">ang:");
    Serial.println(ang);
    Serial.print(" far : ");
    Serial.println(far_);
    // Serial.print(" x : ");
    // Serial.print(x_pos);
    // Serial.print(" y : ");
    // Serial.print(y_pos);
    Serial.print(">dx:");
    Serial.println(dx);
    Serial.print(">dy:");
    Serial.println(dy);
    Serial.print(">dΘ:");
    Serial.println(d_theta);
    Serial.print(" get_val : ");
    Serial.print(get_val);
    // Serial.print(" get : ");
    // Serial.print(ball_get);
    Serial.print(" get_1 : ");
    Serial.print(get_1);
    Serial.print(" get_2 : ");
    Serial.print(get_2);
    // Serial.print(" flag : ");
    // Serial.print(flag);
}