#include <Arduino.h>
#include<Adafruit_NeoPixel.h>
#include<ac.h>
#include<ball.h>
#include<line.h>
#include<motor_a.h>
#include<MA.h>
#include<timer.h>
#include<Cam.h>

BALL ball;
LINE line;
AC ac;
motor_attack MOTOR;
timer Timer;
timer Main;
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
int go_val = 160;
int print_flag = 1;// 1だったらシリアルプリントする
int line_F = 0;
//======================================================きっく======================================================//
void kick();
timer kick_time;
int Kick_F = 0;
const int C = 32;
const int K = 31;
int kick_flag = 0;
//======================================================neopiku======================================================//
#define DELAYVAL 500
#define PIN        30 
#define NUMPIXELS 16

int Neo[16] = {12,11,10,9,8,7,6,5,4,3,2,1,0,15,14,13};
int Neo_p = 999;

Adafruit_NeoPixel pixels(DELAYVAL, PIN, NEO_GRB + NEO_KHZ800);
//======================================================カメラ======================================================//
int goal_color = 0;  //青が0 黄色が1
Cam cam_front(4);
Cam cam_back(3);
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
  pixels.begin();
  pixels.clear();
  pinMode(LED,OUTPUT);
  pinMode(K,OUTPUT);
  pinMode(C,OUTPUT);
  digitalWrite(C,HIGH);
  digitalWrite(K,LOW);
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
  // Switch();
  ac.setup_2();
  Target_dir = ac.dir_n;
}

void loop() {
  Main.reset();
  ball.getBallposition();
  cam_front.getCamdata();
  cam_back.getCamdata();


  float AC_val = 100;
  angle go_ang(ball.ang,true);
  int max_val = go_val;
  int line_flag = line.getLINE_Vec();
  int AC_flag = 0; //0だったら絶対的な角度とる 1だったらゴール向く
  int kick_ = 0; //0だったらキックしない 1だったらキック
  int M_flag = 1; //1だったら動き続ける 0だったら止まる
  float target = Target_dir;

  /*---------------------------------------------------------状況判断ステート--------------------------------------------------------*/

  c = 0;

  if((45 < abs(line.ang) && abs(line.ang) < 135)){
    Lside_A = 1;
  }
  else{
    Lside_A = 0;
  }

  if(Lside_A == 1){
    if(Lside_A != Lside_B){
      Lside_B = Lside_A;
      L_.reset();
    }
    if(5000 < L_.read_ms()){
      A = 15;
      line_F = 1;
    }
  }
  else{
    if(Lside_A != Lside_B){
      Lside_B = Lside_A;
    }
  }

  if(sentor_A == 1){
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

  if(A == 11){
    if(1000 < sentor_t.read_ms() && Kick_F == 0){
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

    if(500 < sentor_t.read_ms() && line_flag == 1){
      A = 15;
      c = 1;
      line_F = 1;
    }
  }

  if(A == 15){
    if(line_F != 3){
      c = 1;
    }
    else{
      line_F = 0;
    }
  }

  if(c == 0){
    if(line_flag == 1){
      A = 10;
    }
    else{
      if(ball.flag == 0){
        A = 5;
      }
      else{
        A = 20;
      }
    }
  }


  /*---------------------------------------------------------動作ステート--------------------------------------------------------*/


  if(A == 5){
    if(A != B){
      B = A;
    }
    M_flag = 3;
  }


  if(A == 10){
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

    if(135 < abs(go_ang.degree)){       //進む角度が真後ろにあるとき
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
    sentor_A = 0;
    for(int i = 0; i < 2; i++){
      int dif_val = abs(ball.ang - go_border[i]);
      if(dif_val < stop_range && back_F == 0){  //正面方向にボールがあったら停止するよ
        M_flag = 0;
        if(abs(ball.ang) < 30){
          sentor_A = 1;
        }
      }
    }
    go_ang.to_range(180,true);  //進む角度を-180 ~ 180の範囲に収める
  }


  if(A == 11){
    if(A != B){
      B = A;
    }
    go_ang = ball.ang * 2;
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
    M_flag = 2;
  }


  if(A == 15){
    if(A != B){
      B = A;
    }
    if(line_F == 1){
      if(line_flag == 0){
        line_F++;
      }
    }
    if(line_F == 2){
      if(line_flag == 1){
        line_F++;
      }
    }
    go_ang = -cam_back.ang + 180;
    M_flag = 2;
    max_val -= 40;
  }

  if(A == 20){
    if(A != B){
      B = A;
      Timer.reset();
    }
    go_ang = line.ang_old;
    M_flag = 2;
  }



  ac.dir_target = target;
  if(AC_flag == 0){
    AC_val = ac.getAC_val();
  }
  else if(AC_flag == 1){
    AC_val = ac.getCam_val(cam_front.ang) * 2;
  }

  if(kick_ == 1){
    if(Kick_F == 0){
      Kick_F = 1;
      kick_time.reset();
    }
  }

  if(Kick_F == 1){
    if(kick_time.read_ms() < 10){
      digitalWrite(C,LOW);
    }
    else if(kick_time.read_ms() < 60){
      digitalWrite(K,HIGH);
      digitalWrite(LED,HIGH);
    }
    else if(kick_time.read_ms() < 70){
      digitalWrite(K,LOW);
      digitalWrite(LED,LOW);
    }
    else{
      digitalWrite(C,HIGH);
      Kick_F = 0;
    }
  }

  digitalWrite(LED,cam_front.on);

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


  if(print_flag == 1){
    // Serial.print(" | L_ : ");
    // Serial.print(L_.read_ms());
    // Serial.print(" | Lside_A : ");
    // Serial.print(Lside_A);
    Serial.print(" | A : ");
    Serial.print(A);
    Serial.print(" | ");
    Serial.print(go_ang.degree);
    // Serial.print(" | sentor_t : ");
    // Serial.print(sentor_t.read_ms());
    // Serial.print(" | line_F : ");
    // Serial.print(line_F);
    // Serial.print(" | line_val : ");
    // Serial.print(MOTOR.line_val);
    // Serial.print(" | line_x : ");
    // Serial.print(line.dis_X * MOTOR.line_val);
    Serial.print(" | ");
    ball.print();
    // Serial.print(" | ");
    // line.print();
    // Serial.print(" | ");
    // line.print_2();
    Serial.print(" | ");
    ac.print();
    // Serial.print(" | ");
    // cam_front.print();
    // Serial.print(" | ");
    // cam_back.print();
    // Serial.print(" | ");
    // Serial.print(L_time);
    // Serial.print(" | ");
    // Serial.print(M_time);
  }

  // if(toogle_f != digitalRead(toogle_P)){
  //   MOTOR.motor_0();
  //   Switch();
  // }
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



void kick(){
  // esc.writeMicroseconds(1000);
  digitalWrite(C,LOW);
  delay(10);
  digitalWrite(K,HIGH);
  digitalWrite(LED,HIGH);
  delay(10);
  digitalWrite(K,LOW);
  digitalWrite(LED,LOW);
  delay(10);
  digitalWrite(C,HIGH);
  // MOTOR.Moutput(4,-200);
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

  for(int i = 0; i < 6; i++){
    Serial.print(" ");
    Serial.print(reBuf[i]);
  }
  Serial.println();
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
  // Serial.println("sawa4");
  int n;
  int x,y;
  word revBuf_word[7];
  byte revBuf_byte[7];
  //受信データ数が、一定時間同じであれば、受信完了としてデータ読み出しを開始処理を開始する。
  //受信データあり ※6バイト以上になるまでまつ
  if(Serial8.available() < 7){
    // Serial.print(Serial8.available());
    return;
  }

  revBuf_byte[0] = Serial8.read();
  if(revBuf_byte[0] != 0xFF){
    // Serial.print("!!!!!");
    return;
  }
    //---------------------------
    //受信データをバッファに格納
    //---------------------------
    n = 1;
  while(Serial8.available()>0 ){ //受信データがなくなるまで読み続ける
    //6バイト目まではデータを格納、それ以上は不要なデータであるため捨てる。
    if(n < 7){
      revBuf_byte[n] = Serial8.read();   //revBuf_byte[n] = Serial2.read();
    }
    else{
      Serial8.read(); //Serial2.read();  //読みだすのみで格納しない。
    }
    n++;
  }
  // for(int i = 0; i < 7; i++){
  //   Serial.print(revBuf_byte[i]);
  //   Serial.print(" ");
  // }
    //---------------------------
    //データの中身を確認
    //---------------------------
    //データの先頭、終了コードあることを確認
  if((revBuf_byte[0] == 0xFF ) && ( revBuf_byte[6] == 0xAA )){
  //いったんWORD型（16bitデータ）としてから、int16_tとする。
    revBuf_word[0] = (uint16_t(revBuf_byte[1])<< 8);//上位8ビットをbyteから、Wordに型変換して格納　上位桁にするため8ビットシフト
    revBuf_word[1] = uint16_t(revBuf_byte[2]);//下位8ビットをbyteから、Wordに型変換して格納      
    x = int16_t(revBuf_word[0]|revBuf_word[1]);//上位8ビット、下位ビットを合成（ビットのORを取ることで格納する。）
    // ※int ではなく　int16_t　にすることが必要。intだけだと、32ビットのintと解釈されマイナス値がマイナスとみなされなくなる、int16_tは、16ビット指定の整数型変換
    revBuf_word[2] = (uint16_t(revBuf_byte[3])<< 8);//上位8ビットをbyteから、Wordに型変換して格納　上位桁にするため8ビットシフト
    revBuf_word[3] = uint16_t(revBuf_byte[4]);//下位8ビットをbyteから、Wordに型変換して格納      
    y = int16_t(revBuf_word[2]|revBuf_word[3]);//上位8ビット、下位ビットを合成（ビットのORを取ることで格納する。）
    // ※int ではなく　int16_t　にすることが必要。intだけだと、32ビットのintと解釈されマイナス値がマイナスとみなされなくなる、int16_tは、16ビット指定の整数型変換
    ball.get_resister(revBuf_byte[5]);

    x = ball.ball_x.demandAve(x);
    y = ball.ball_y.demandAve(y);
    // Serial.print("!!!!!!!!!!!!!");
  }
  else{
    // printf("ERR_REV");
  }
  
}