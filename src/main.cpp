#include <Arduino.h>
#include<Adafruit_NeoPixel.h>
#include<ac.h>
#include<ball.h>
#include<line.h>
#include<motor_a.h>
#include<MA.h>
#include<timer.h>
#include<Cam.h>
#include<kicker.h>

BALL ball;
LINE line;
AC ac;
motor_attack MOTOR;
Kicker kicker;
timer Timer;
timer Main;
timer start_t;
int M_time;

int Lside_A = 0;
int Lside_B = 999;
timer L_;
timer sentor_t;
int sentor_A = 0;
int sentor_B = 999;
int go_flag = 0;

int Bget_B = 999;

int A = 0;
int B = 999;
int c = 0;
int stop_range = 10;
int P_range = 30;
const int far_th = 130;
int go_val = 180;
int print_flag = 1;// 1だったらシリアルプリントする
int line_F = 0;
int back_Flag = 0;
int goang_old = 0;

//======================================================カメラ======================================================//

int goal_color = 0;  //青が0 黄色が1
Cam cam_front(4);
Cam cam_back(3);
int CB_old = 999;
timer CB_t;

//======================================================スタートスイッチ======================================================//

int LED = 13;
int toogle_f;
int toogle_P = 27;
void Switch();
int Target_dir;

//======================================================ライン======================================================//

int Line_flag = 0;
int Line_target_dir;
int L_time;

//======================================================関数たち======================================================/

void setup() {
  Serial.begin(9600);
  ball.begin();
  line.begin();
  ac.setup();
  cam_front.begin();
  cam_back.begin();
  kicker.setup();
  pinMode(LED,OUTPUT);
  if(goal_color == 0){
    cam_front.color = 0;  //青が0 黄色が1
    cam_back.color = 1;  //青が0 黄色が1
  }
  else if(goal_color == 1){
    cam_front.color = 1;  //青が0 黄色が1
    cam_back.color = 0;  //青が0 黄色が1
  }
  // MOTOR.motor_ac(100);
  // delay(200);
  // MOTOR.motor_0();
  Switch();
}

void loop() {
  Main.reset();
  ball.getBallposition();
  cam_front.getCamdata();
  cam_back.getCamdata();
  int line_flag = line.getLINE_Vec();  //ラインセンサの入力
  angle go_ang(ball.ang,true);         //進む角度のオブジェクト

  float AC_val = 100;                  //姿勢制御の出力
  int max_val = go_val;                //進む出力
  float target = Target_dir;           //目標角度

  int AC_flag = 0;                     //0だったら絶対的な角度とる 1だったらゴール向く
  int kick_ = 0;                       //0だったらキックしない 1だったらキック
  int M_flag = 1;                      //1だったら動き続ける 0だったら止まる

  /*---------------------------------------------------------状況判断ステート--------------------------------------------------------*/

  c = 0;

  if((45 < abs(line.ang) && abs(line.ang) < 135) && cam_back.on == 1 && cam_back.Size < 60){  //横向きにラインを踏んでるフラグ
    Lside_A = 1;
  }
  else{
    Lside_A = 0;
  }

  if(Lside_A == 1){          //(横向きにラインを踏み続けているか<=>コートの横向きのライン上にいるか)判定
    if(Lside_A != Lside_B){
      Lside_B = Lside_A;
      L_.reset();
    }
    if(2500 < L_.read_ms()){
      A = 15;
      line_F = 1;
    }
  }
  else{
    if(Lside_A != Lside_B){
      Lside_B = Lside_A;
    }
  }

  if(back_Flag == 1 && line_flag == 0){
    A = 15;
    line_F = 1;
  }

  if(cam_back.on == 1){
    CB_old = 1;
  }
  else{
    if(cam_back.on != CB_old){
      CB_old = cam_back.on;
      CB_t.reset();
    }
    if(1000 < CB_t.read_ms()){
      A = 16;
    }
  }

  if(sentor_A == 1){          //前にボールがあり続けるか判定
    if(sentor_A != sentor_B){
      sentor_B = sentor_A;
      sentor_t.reset();
    }
    if(3000 < sentor_t.read_ms()){
      A = 11;
      sentor_t.reset();
    }
  }
  else{
    if(sentor_A != sentor_B){
      sentor_B = sentor_A;
    }
  }

  if(A == 11){     //前進し続けるか判定
    if(1000 < sentor_t.read_ms() && kicker.kick_flag == 0){
      A = 15;
      c = 1;
      sentor_t.reset();
      if(line_flag == 1){
        line_F = 1;
      }
      else{
        line_F = 2;
      }
    }
    else{
      c = 1;
    }

    if((500 < sentor_t.read_ms() && line_flag == 1) || 45 < abs(ball.ang)){
      A = 15;
      c = 1;
      line_F = 1;
    }
  }

  if(A == 15){      //戻るとき後ろに下がり続けるか判定
    if(line_F != 4){
      c = 1;
    }
    else{
      line_F = 0;
    }
  }

  if(c == 0){  //平常時どうするか判定
    if(line_flag == 1){
      if(abs(ball.ang) < 150){
        if(cam_back.on == 1){
          A = 10;
        }
        else{
          A = 16;
        }
      }
      else{
        A = 5;
      }
    }
    else{
      A = 20;
    }

    if(cam_back.on == 0){
      A = 16;
    }
  }


  /*---------------------------------------------------------動作ステート--------------------------------------------------------*/


  if(A == 5){  //ボールがない時止まる
    if(A != B){
      B = A;
    }
    M_flag = 3;
  }



  if(A == 10){  //ライントレース(アルゴリズムブログで書きたいな)
    if(A != B){
      B = A;
    }
    int go_flag = 0;
    double go_border[2];  //ボールの角度によって進む方向を変えるためのボーダーの変数(ラインに対して垂直な直線で進む角度の区分を分けるイメージ)
    angle balldir(ball.ang,true);  //ボールの角度を入れるオブジェクト
    if(2 < line.num){
      line.ang = 90;
    }

    if(line.ang < 0){
      go_border[0] = line.ang;
      go_border[1] = line.ang + 180;
    }
    else{
      go_border[0] = line.ang - 180;
      go_border[1] = line.ang;
    }

    balldir.to_range(go_border[0],false);  //ボールの角度をボーダーの範囲に収める(go_border[0] ~ go_border[1]+180)

    if(go_border[0] < balldir.degree && balldir.degree < go_border[1]){  //ボールの角度を区分分けする
      go_flag = 0;
    }
    else{
      go_flag = 1;
    }

    go_ang = go_border[go_flag] + 90;  //進む角度決定
    go_ang.to_range(180,true);  //進む角度を-180 ~ 180の範囲に収める

    int back_F = 0;

    if(140 < abs(go_ang.degree)){       //進む角度が真後ろにあるとき
      go_ang += 180;
      back_F = 1;
    }
    else if(120 < abs(go_ang.degree)){
      M_flag = 0;
    }
    else if(abs(go_ang.degree) < 60){  //前めに進むとき
      MOTOR.line_val = 2;
    }
    else{                              //横に進むとき
      MOTOR.line_val = 1.0;
    }

    if(cam_back.on == 0){
      A = 16;
    }

    sentor_A = 0;
    for(int i = 0; i < 2; i++){
      int dif_val = abs(ball.ang - go_border[i]);
      if(dif_val < stop_range && back_F == 0){  //正面方向にボールがあったら停止するよ
        M_flag = 0;
        if(abs(ball.ang) < 45){
          sentor_A = 1;
        }
      }
    }
    go_ang.to_range(180,true);  //進む角度を-180 ~ 180の範囲に収める
  }



  if(A == 11){  //ボールが前にあるから前進
    if(A != B){
      B = A;
      goang_old = ball.ang;
    }
    go_ang = ball.ang;
    if(ball.ball_get){
      if(ball.ball_get != Bget_B){
        Bget_B = ball.ball_get;
        Timer.reset();
      }
      if(300 < Timer.read_ms()){
        kick_ = 1;
        AC_flag = 1;
      }
    }
    else{
      Bget_B = 0;
    }

    if(30 < abs(ball.ang - goang_old)){
      A = 15;
    }
    M_flag = 2;
  }



  if(A == 15){  //ゴール際に戻る
    if(A != B){
      B = A;
      Timer.reset();
      line_F = 1;
    }
    go_ang = -cam_back.ang + 180;
    M_flag = 2;
    max_val -= 30;
    if(line_F == 1){
      if(line_flag == 0){
        line_F++;
      }
      digitalWrite(LED,LOW);
    }
    if(line_F == 2){
      if(line_flag == 1 && 200 < Timer.read_ms()){
        line_F++;
      }
      digitalWrite(LED,HIGH);
    }
    if(line_F == 3){
      if(300 < Timer.read_ms()){
        line_F++;
      }
      go_ang = 0;
      digitalWrite(LED,LOW);
    }
  }


  if(A == 16){
    if(A != B){
      B = A;
    }
    go_ang = 0;
    M_flag = 2;
  }



  if(A == 20){  //ロボットがちょっと押し出されちゃったから戻る
    if(A != B){
      B = A;
      Timer.reset();
    }
    if(300 < Timer.read_ms() && cam_back.Size < 60){
      A = 11;
    }
    go_ang = line.ang_old;
    M_flag = 2;
  }



  /*---------------------------------------------------------出力ステート--------------------------------------------------------*/


  back_Flag = 0;
  ac.dir_target = target;

  if(30 < abs(ac.dir)){
    AC_val = ac.getAC_val();
    M_flag = 0;
    if(abs(line.ang_old) < 90){
      back_Flag = 0;
    }
    else{
      back_Flag = 1;
    }
  }
  else if(AC_flag == 0){
    AC_val = ac.getAC_val();
  }
  else if(AC_flag == 1){
    AC_val = ac.getCam_val(cam_front.ang) * 2;
  }


  kicker.run(kick_);

  if(M_flag == 1){
    MOTOR.moveMotor_L(go_ang,max_val,AC_val,line);
  }
  else if(M_flag == 0){
    MOTOR.motor_ac(AC_val);
  }
  else if(M_flag == 2){
    MOTOR.moveMotor_0(go_ang,max_val,AC_val,0);
  }
  else if(M_flag == 3){
    MOTOR.motor_0();
  }


  // digitalWrite(LED,cam_back.on);


  if(print_flag == 1){
    // Serial.print(" | L_ : ");
    // Serial.print(L_.read_ms());
    // Serial.print(" | Lside_A : ");
    // Serial.print(Lside_A);
    // Serial.print(" | A : ");
    // Serial.print(A);
    Serial.print(" | ");
    Serial.print(go_ang.degree);
    // Serial.print(" | sentor_t : ");
    // Serial.print(sentor_t.read_ms());
    // Serial.print(" | line_F : ");
    // Serial.print(line_F);
    // Serial.print(" | line_val : ");
    // Serial.print(MOTOR.line_val);
    Serial.print(" | ");
    ball.print();
    // Serial.int(" | ");
    // line.print();
    // Serial.print(" | ");
    // line.print_2();
    // Serial.print(" | ");
    // ac.print();
    Serial.print(" | ");
    cam_back.print();
    // Serial.print(" | ");
    // cam_back.print();
    // Serial.print(" | ");
    // Serial.print(L_time);
    // Serial.print(" | ");
    // Serial.print(M_time);
  }

  if(toogle_f != digitalRead(toogle_P)){
    MOTOR.motor_0();
    Switch();
    A = 0;
    sentor_t.reset();
    L_.reset();
    start_t.reset();
  }
  Serial.println();
  M_time = Main.read_us();
}



void Switch(){
  digitalWrite(LED,HIGH);
  toogle_f = digitalRead(toogle_P);
  delay(100);
  while(digitalRead(toogle_P) == toogle_f);
  digitalWrite(LED,LOW);
  ac.setup_2();
  Target_dir = ac.dir_n;
  toogle_f = digitalRead(toogle_P);
  delay(100);
  while(digitalRead(toogle_P) == toogle_f);
  toogle_f = digitalRead(toogle_P);  //トグルがもげちゃったからいったんLチカでスタート
}



void serialEvent3(){
  // Serial.print("sawa1");
  uint8_t reBuf[6];
  if(Serial3.available() < 6){
    return;
  }

  for(int i = 0; i < 6; i++){
    reBuf[i] = Serial3.read();
  }
  while(Serial3.available()){
    Serial3.read();
  }

  if(reBuf[0] == 38 && reBuf[5] == 37){
    for(int i = 0; i < 4; i++){
      cam_back.data_byte[i] = reBuf[i+1];
    }
  }

  // for(int i = 0; i < 6; i++){
  //   Serial.print(" ");
  //   Serial.print(reBuf[i]);
  // }
  // Serial.println();
}



void serialEvent4(){
  // Serial.print("sawa2");
  uint8_t reBuf[6];
  if(Serial4.available() < 6){
    return;
  }

  for(int i = 0; i < 6; i++){
    reBuf[i] = Serial4.read();
  }
  while(Serial4.available()){
    Serial4.read();
  }

  if(reBuf[0] == 38 && reBuf[5] == 37){
    for(int i = 0; i < 4; i++){
      cam_front.data_byte[i] = reBuf[i+1];
    }
  }

  for(int i = 0; i < 6; i++){
    // Serial.print(" ");
    // Serial.print(reBuf[i]);
  }
}



void serialEvent6(){
  // Serial.println(" sawa3 ");
  uint8_t read[6];
  int n = 1;
  if(Serial6.available() < 6){
    return;
  }
  read[0] = Serial6.read();
  if(read[0] != 38){
    return;
  } 
  while(0 < Serial6.available()){
    if(n < 6){
      read[n] = Serial6.read();
    }
    else{
      Serial6.read();
    }
    n++;
  }

  if(read[0] == 38 && read[5] == 37){
    line.data_byte[0] = read[1];
    line.data_byte[1] = read[2];
    line.data_byte[2] = read[3];
    line.data_byte[3] = read[4];
  }
  else{
    // Serial.print(" Error!! ");
  }
  // for(int i = 0; i < 6; i++){
  //   Serial.print(read[i]);
  //   Serial.print(" ");
  // }
  // Serial.println();
}



void serialEvent8(){
  uint8_t read[8];
  if(Serial8.available() < 8){
    return;
  }
  read[0] = Serial8.read();
  if(read[0] != 0xFF){
    return;
  }
  for(int i = 1; i < 8; i++){
    read[i] = Serial8.read();
  }
  while(Serial8.available()){
    Serial8.read();
  }

  if(read[0] == 0xFF && read[7] == 0xAA){
    ball.data_byte[0] = read[1];
    ball.data_byte[1] = read[2];
    ball.data_byte[2] = read[3];
    ball.data_byte[3] = read[4];
    ball.data_byte[4] = read[5];
    ball.data_byte[5] = read[6];
  }

  // for(int i = 0; i < 8; i++){
  //   Serial.print(" ");
  //   Serial.print(read[i]);
  // }
  // Serial.println();
}