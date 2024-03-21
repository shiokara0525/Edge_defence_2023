#include<OLED_a.h>

void oled_attack::setup(){
  EEPROM.get(address,line.LINE_Level);//EEPROMから読み出し
  address += sizeof(line.LINE_Level);  //アドレスを次の変数のアドレスにする
  EEPROM.get(address,RA_size);//EEPROMから読み出し(前回取り出した変数からアドレスを取得し、次のアドレスをここで入力する)
  address += sizeof(RA_size);  //アドレスを次の変数のアドレスにする
  EEPROM.get(address,val_max);//EEPROMから読み出し(前回取り出した変数からアドレスを取得し、次のアドレスをここで入力する)
  address += sizeof(val_max);  //アドレスを次の変数のアドレスにする
  EEPROM.get(address,Button_selectCF);//EEPROMから読み出し(前回取り出した変数からアドレスを取得し、次のアドレスをここで入力する)
  address += sizeof(Button_selectCF);  //アドレスを次の変数のアドレスにする
  for(int i = 0; i < 6; i++){
    EEPROM.get(address,check_val[i]);//EEPROMから読み出し(前回取り出した変数からアドレスを取得し、次のアドレスをここで入力する)
    address += sizeof(check_val[i]);  //アドレスを次の変数のアドレスにする
  }
  EEPROM.get(address,ball.get_th);
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    // Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
}




void oled_attack::OLED() {
  //OLEDの初期化
  display.display();
  display.clearDisplay();

  timer_OLED.reset(); //タイマーのリセット(OLED用)
  toogle = digitalRead(Toggle_Switch);

  int A_OLED = 0;
  int B_OLED = 999;  //ステート初期化のための変数
  int aa = 0;  //タクトスイッチのルーレット状態防止用変数

  int flash_OLED = 0;  //ディスプレイの中で白黒点滅させたいときにつかう
  int OLED_select = 1;  //スイッチが押されたときにどこを選択しているかを示す変数(この数字によって選択画面の表示が変化する)
  int Button_select = 0;  //スイッチが押されたときにどこを選択しているかを示す変数(この数字によってexitかnextかが決まる)

  int OLED_ball_x = 0;
  int OLED_ball_y = 0;

  int OLED_cam_x = 0;
  int OLED_cam_y = 0;

  float line_x = 0;
  float line_y = 0;

  float Ax = 0;
  float Ay = 0;
  float Bx = 0;
  float By = 0;

  int OLED_line_ax = 0;
  int OLED_line_ay = 0;
  int OLED_line_bx = 0;
  int OLED_line_by = 0;

  MOTOR.NoneM_flag = 0;  //モーター動作ありのフラグ
  
  while(1){
    // // Serial.print("start!!!!");
    if(timer_OLED.read_ms() > 500) //0.5秒ごとに実行(OLEDにかかれてある文字を点滅させるときにこの周期で点滅させる)
    {
      if(flash_OLED == 0){
        flash_OLED = 1;
      }
      else{
        flash_OLED = 0;
      }
      timer_OLED.reset(); //タイマーのリセット(OLED用)
    }


    if(A_OLED == 0)  //メインメニュー
    {
      // Serial.print(" A_OLED=0 start ");
      if(A_OLED != B_OLED)  //ステートが変わったときのみ実行(初期化)
      {
        OLED_select = 1;  //選択画面をデフォルトにする
        B_OLED = A_OLED;
      }
      // Serial.print(" Cat ");

      //OLEDの初期化
      display.display();
      // Serial.print(" Cat0.5 ");
      display.clearDisplay();
      // Serial.print(" Cat1 ");

      //選択画面だということをしらせる言葉を表示
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0,0);
      display.println("Hi! bro!");
      display.setCursor(0,10);
      display.println("What's up?");

      //文字と選択画面の境目の横線を表示
      display.drawLine(0, 21, 128, 21, WHITE);
      // Serial.print(" select start ");

      //選択画面の表示
      if(OLED_select == 1)  //STARTを選択しているとき
      {
        // Serial.print(" select=0 start ");
        //START値を調整
        display.setTextSize(2);
        if(flash_OLED == 0){  //白黒反転　何秒かの周期で白黒が変化するようにタイマーを使っている（flash_OLEDについて調べたらわかる）
          display.setTextColor(BLACK, WHITE);
        }
        else{
          display.setTextColor(WHITE);
        }
        display.setCursor(0,35);
        display.println("START");

        //選択画面で矢印マークを中央に表示
        display.fillTriangle(70, 43, 64, 37, 64, 49, WHITE);  //▶の描画

        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(90,35);
        display.println("Set");
        display.setCursor(88,44);
        display.println("Motar");

        //タクトスイッチが押されたら(手を離されるまで次のステートに行かせたくないため、変数aaを使っている)
        if(aa == 0){
          if(digitalRead(Tact_Switch[1]) == LOW){  //タクトスイッチが押されたら
            aa = 1;
          }
        }else{
          if(digitalRead(Tact_Switch[1]) == HIGH){  //タクトスイッチが手から離れたら
            A_OLED = 10;  //その選択されているステートにレッツゴー
            aa = 0;
          }
        }
        // Serial.print(" select=1 end ");
      }
      else if(OLED_select == 2)  //Set Motarを選択しているとき
      {
        //Motar値を調整
        display.setTextSize(2);
        if(flash_OLED == 0){  //白黒反転　何秒かの周期で白黒が変化するようにタイマーを使っている（flash_OLEDについて調べたらわかる）
          display.setTextColor(BLACK, WHITE);
        }
        else{
          display.setTextColor(WHITE);
        }
        display.setCursor(12,27);
        display.println("Set");
        display.setCursor(0,44);
        display.println("Motar");

        //選択画面で矢印マークを中央に表示
        display.fillTriangle(70, 43, 64, 37, 64, 49, WHITE);  //▶の描画

        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(90,35);
        display.println("Set");
        display.setCursor(88,45);
        display.println("Line");

        //タクトスイッチが押されたら(手を離されるまで次のステートに行かせたくないため、変数aaを使っている)
        if(aa == 0){
          if(digitalRead(Tact_Switch[1]) == LOW){  //タクトスイッチが押されたら
            aa = 1;
          }
        }else{
          if(digitalRead(Tact_Switch[1]) == HIGH){  //タクトスイッチが手から離れたら
            A_OLED = 60;  //その選択されているステートにレッツゴー
            aa = 0;
          }
        }
      }
      else if(OLED_select == 3)  //Set Lineを選択しているとき
      {
        //Line値を調整
        display.setTextSize(2);
        if(flash_OLED == 0){  //白黒反転　何秒かの周期で白黒が変化するようにタイマーを使っている（flash_OLEDについて調べたらわかる）
          display.setTextColor(BLACK, WHITE);
        }
        else{
          display.setTextColor(WHITE);
        }
        display.setCursor(12,27);
        display.println("Set");
        display.setCursor(6,44);
        display.println("Line");

        //選択画面で矢印マークを中央に表示
        display.fillTriangle(70, 43, 64, 37, 64, 49, WHITE);  //▶の描画

        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(85,35);
        display.println("Check");
        display.setCursor(88,45);
        display.println("Line");

        //タクトスイッチが押されたら(手を離されるまで次のステートに行かせたくないため、変数aaを使っている)
        if(aa == 0){
          if(digitalRead(Tact_Switch[1]) == LOW){  //タクトスイッチが押されたら
            aa = 1;
          }
        }else{
          if(digitalRead(Tact_Switch[1]) == HIGH){  //タクトスイッチが手から離れたら
            A_OLED = 20;  //その選択されているステートにレッツゴー
            aa = 0;
          }
        }
      }
      else if(OLED_select == 4)  //Check Lineを選択しているとき
      {
        //Check Lineの文字設定
        display.setTextSize(2);
        if(flash_OLED == 0){  //白黒反転　何秒かの周期で白黒が変化するようにタイマーを使っている（flash_OLEDについて調べたらわかる）
          display.setTextColor(BLACK, WHITE);
        }
        else{
          display.setTextColor(WHITE);
        }
        display.setCursor(0,27);
        display.println("Check");
        display.setCursor(6,44);
        display.println("Line");

        //選択画面で矢印マークを中央に表示
        display.fillTriangle(70, 43, 64, 37, 64, 49, WHITE);  //▶の描画

        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(90,35);
        display.println("Set");
        display.setCursor(94,45);
        display.println("Ball");

        //タクトスイッチが押されたら(手を離されるまで次のステートに行かせたくないため、変数aaを使っている)
        if(aa == 0){
          if(digitalRead(Tact_Switch[1]) == LOW){  //タクトスイッチが押されたら
            aa = 1;
          }
        }else{
          if(digitalRead(Tact_Switch[1]) == HIGH){  //タクトスイッチが手から離れたら
            A_OLED = 30;  //その選択されているステートにレッツゴー
            aa = 0;
          }
        }
      }
      else if(OLED_select == 5)  //Set RA（回り込みの大きさ）を選択しているとき
      {
        //回り込みの大きさを調整
        display.setTextSize(2);
        if(flash_OLED == 0){  //白黒反転　何秒かの周期で白黒が変化するようにタイマーを使っている（flash_OLEDについて調べたらわかる）
          display.setTextColor(BLACK, WHITE);
        }
        else{
          display.setTextColor(WHITE);
        }
        display.setCursor(12,27);
        display.println("Set");
        display.setCursor(18,44);
        display.println("Ball");

        //選択画面で矢印マークを中央に表示
        display.fillTriangle(70, 43, 64, 37, 64, 49, WHITE);  //▶の描画

        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(85,35);
        display.println("Check");
        display.setCursor(88,45);
        display.println("Ball");

        //タクトスイッチが押されたら(手を離されるまで次のステートに行かせたくないため、変数aaを使っている)
        if(aa == 0){
          if(digitalRead(Tact_Switch[1]) == LOW){  //タクトスイッチが押されたら
            aa = 1;
          }
        }else{
          if(digitalRead(Tact_Switch[1]) == HIGH){  //タクトスイッチが手から離れたら
            A_OLED = 40;  //その選択されているステートにレッツゴー
            aa = 0;
          }
        }
      }
      else if(OLED_select == 6)  //Check Ballを選択しているとき
      {
        //Check Ballの文字設定
        display.setTextSize(2);
        if(flash_OLED == 0){  //白黒反転　何秒かの周期で白黒が変化するようにタイマーを使っている（flash_OLEDについて調べたらわかる）
          display.setTextColor(BLACK, WHITE);
        }
        else{
          display.setTextColor(WHITE);
        }
        display.setCursor(0,27);
        display.println("Check");
        display.setCursor(6,44);
        display.println("Ball");

        //選択画面で矢印マークを中央に表示
        display.fillTriangle(70, 43, 64, 37, 64, 49, WHITE);  //▶の描画

        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(85,35);
        display.println("Check");
        display.setCursor(88,45);
        display.println("B_get");

        //タクトスイッチが押されたら(手を離されるまで次のステートに行かせたくないため、変数aaを使っている)
        if(aa == 0){
          if(digitalRead(Tact_Switch[1]) == LOW){  //タクトスイッチが押されたら
            aa = 1;
          }
        }else{
          if(digitalRead(Tact_Switch[1]) == HIGH){  //タクトスイッチが手から離れたら
            A_OLED = 50;  //その選択されているステートにレッツゴー
            aa = 0;
          }
        }
      }
      else if(OLED_select == 7){
        display.setTextSize(2);
        if(flash_OLED == 0){  //白黒反転　何秒かの周期で白黒が変化するようにタイマーを使っている（flash_OLEDについて調べたらわかる）
          display.setTextColor(BLACK, WHITE);
        }
        else{
          display.setTextColor(WHITE);
        }
        display.setCursor(0,27);
        display.println("check");
        display.setCursor(10,44);
        display.println("B_get");

        //選択画面で矢印マークを中央に表示
        display.fillTriangle(70, 43, 64, 37, 64, 49, WHITE);  //▶の描画

        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(85,40);
        display.println("setAny");

        //タクトスイッチが押されたら(手を離されるまで次のステートに行かせたくないため、変数aaを使っている)
        if(aa == 0){
          if(digitalRead(Tact_Switch[1]) == LOW){  //タクトスイッチが押されたら
            aa = 1;
          }
        }else{
          if(digitalRead(Tact_Switch[1]) == HIGH){  //タクトスイッチが手から離れたら
            A_OLED = 90;  //その選択されているステートにレッツゴー
            aa = 0;
          }
        }
      }
      else if(OLED_select == 8)  //勝手にデバックしてもろて
      {
        //Check anyの文字設定
        display.setTextSize(2);
        if(flash_OLED == 0){  //白黒反転　何秒かの周期で白黒が変化するようにタイマーを使っている（flash_OLEDについて調べたらわかる）
          display.setTextColor(BLACK, WHITE);
        }
        else{
          display.setTextColor(WHITE);
        }
        display.setCursor(0,27);
        display.println("set");
        display.setCursor(10,44);
        display.println("Any");

        //選択画面で矢印マークを中央に表示
        display.fillTriangle(70, 43, 64, 37, 64, 49, WHITE);  //▶の描画

        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(85,40);
        display.println("checkCam");

        //タクトスイッチが押されたら(手を離されるまで次のステートに行かせたくないため、変数aaを使っている)
        if(aa == 0){
          if(digitalRead(Tact_Switch[1]) == LOW){  //タクトスイッチが押されたら
            aa = 1;
          }
        }else{
          if(digitalRead(Tact_Switch[1]) == HIGH){  //タクトスイッチが手から離れたら
            A_OLED = 70;  //その選択されているステートにレッツゴー
            aa = 0;
          }
        }
      }
      else if(OLED_select == 9){
        //Check Ballの文字設定
        display.setTextSize(2);
        if(flash_OLED == 0){  //白黒反転　何秒かの周期で白黒が変化するようにタイマーを使っている（flash_OLEDについて調べたらわかる）
          display.setTextColor(BLACK, WHITE);
        }
        else{
          display.setTextColor(WHITE);
        }
        display.setCursor(0,27);
        display.println("Check");
        display.setCursor(6,44);
        display.println("Cam");

        //選択画面で矢印マークを中央に表示
        display.fillTriangle(70, 43, 64, 37, 64, 49, WHITE);  //▶の描画

        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(85,35);
        display.println("Kick");
        display.setCursor(88,45);
        display.println("");

        //タクトスイッチが押されたら(手を離されるまで次のステートに行かせたくないため、変数aaを使っている)
        if(aa == 0){
          if(digitalRead(Tact_Switch[1]) == LOW){  //タクトスイッチが押されたら
            aa = 1;
          }
        }else{
          if(digitalRead(Tact_Switch[1]) == HIGH){  //タクトスイッチが手から離れたら
            A_OLED = 80;  //その選択されているステートにレッツゴー
            aa = 0;
          }
        }
      }
      else if(OLED_select == 10){
        display.setTextSize(2);
        if(flash_OLED == 0){  //白黒反転　何秒かの周期で白黒が変化するようにタイマーを使っている（flash_OLEDについて調べたらわかる）
          display.setTextColor(BLACK, WHITE);
        }
        else{
          display.setTextColor(WHITE);
        }
        display.setCursor(0,35);
        display.println("Kick");

        //選択画面で矢印マークを中央に表示
        display.fillTriangle(70, 43, 64, 37, 64, 49, WHITE);  //▶の描画

        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(85,35);
        display.println("Start");
        display.setCursor(88,45);
        display.println("");

        //タクトスイッチが押されたら(手を離されるまで次のステートに行かせたくないため、変数aaを使っている)
        if(aa == 0){
          if(digitalRead(Tact_Switch[1]) == LOW){  //タクトスイッチが押されたら
            aa = 1;
          }
        }else{
          if(digitalRead(Tact_Switch[1]) == HIGH){  //タクトスイッチが手から離れたら
            A_OLED = 100;  //その選択されているステートにレッツゴー
            aa = 0;
          }
        }
      }
      // Serial.print("A_OLED=0 end");
    }
    else if(A_OLED == 10)  //START
    { //機体の中心となるコート上での0°の位置を決めるところ
      if(A_OLED != B_OLED){  //ステートが変わったときのみ実行(初期化)
        Button_select = 0;  //ボタンの選択(next)をデフォルトにする
        B_OLED = A_OLED;
      };

      //OLEDの初期化
      display.display();
      display.clearDisplay();

      display.setTextSize(2);
      display.setTextColor(WHITE);
      display.setCursor(30,0);
      display.println("Please");
      display.setCursor(0,20);
      display.println("CAL");
      display.setCursor(40,20);
      display.println("&");
      display.setCursor(56,20);
      display.println("SetDir");

      display.setTextSize(1);
      display.setCursor(38,40);
      display.println("of BNO055");

      display.setTextColor(WHITE);
      if(Button_select == 1)  //exitが選択されていたら
      {
        if(flash_OLED == 0){  //白黒反転　何秒かの周期で白黒が変化するようにタイマーを使っている（flash_OLEDについて調べたらわかる）
          display.setTextColor(BLACK, WHITE);
        }
        else{
          display.setTextColor(WHITE);
        }
      }
      display.setCursor(0,56);
      display.println("Exit");

      display.setTextColor(WHITE);
      if(Button_select == 0)  //nextが選択されていたら（デフォルトはこれ）
      {
        if(flash_OLED == 0){  //白黒反転　何秒かの周期で白黒が変化するようにタイマーを使っている（flash_OLEDについて調べたらわかる）
          display.setTextColor(BLACK, WHITE);
        }
        else{
          display.setTextColor(WHITE);
        }
      }
      display.setCursor(104,56);
      display.println("Next");

      //タクトスイッチが押されたら(手を離されるまで次のステートに行かせたくないため、変数aaを使っている)
      if(aa == 0){
        if(digitalRead(Tact_Switch[1]) == LOW){  //タクトスイッチが押されたら
          aa = 1;
        }
      }else{
        if(digitalRead(Tact_Switch[1]) == HIGH){  //タクトスイッチが手から離れたら
          if(Button_select == 0)  //nextが選択されていたら
          {
            ac.setup_2();  //姿勢制御の値リセットするぜい
            A_OLED = 12;  //コート方向判定
          }
          else if(Button_select == 1)  //exitが選択されていたら
          {
            A_OLED = 0;  //メニュー画面に戻る
          }
          aa = 0;
        }
      }
    }
    else if(A_OLED == 12)
    {
      if(A_OLED != B_OLED){  //ステートが変わったときのみ実行(初期化)
        Button_select = 0;  //ボタンの選択(setDir)をデフォルトにする
        B_OLED = A_OLED;
      };

      display.display();
      display.clearDisplay();

      display.setTextSize(1);

      display.setTextColor(WHITE);
      if(Button_selectCF == 0)  //exitが選択されていたら
      {
        if(flash_OLED == 0){  //白黒反転　何秒かの周期で白黒が変化するようにタイマーを使っている（flash_OLEDについて調べたらわかる）
          display.setTextColor(BLACK, WHITE);
        }
        else{
          display.setTextColor(WHITE);
        }
      }
      display.setCursor(0,30);
      display.println("Yellow");

      display.setTextColor(WHITE);
      if(Button_selectCF == 1)  //exitが選択されていたら
      {
        if(flash_OLED == 0){  //白黒反転　何秒かの周期で白黒が変化するようにタイマーを使っている（flash_OLEDについて調べたらわかる）
          display.setTextColor(BLACK, WHITE);
        }
        else{
          display.setTextColor(WHITE);
        }
      }
      display.setCursor(50,55);
      display.println("Exit");

      display.setTextColor(WHITE);
      if(Button_selectCF == 2)  //exitが選択されていたら
      {
        if(flash_OLED == 0){  //白黒反転　何秒かの周期で白黒が変化するようにタイマーを使っている（flash_OLEDについて調べたらわかる）
          display.setTextColor(BLACK, WHITE);
        }
        else{
          display.setTextColor(WHITE);
        }
      }
      display.setCursor(90,30);
      display.println("Blue");

      //タクトスイッチが押されたら(手を離されるまで次のステートに行かせたくないため、変数aaを使っている)
      if(aa == 0){
        if(digitalRead(Tact_Switch[1]) == LOW){  //タクトスイッチが押されたら
          aa = 1;
        }
      }else{
        if(digitalRead(Tact_Switch[1]) == HIGH){  //タクトスイッチが手から離れたら
          if(Button_selectCF == 0)  //yellowが選択されていたら
          {
            cam_front.color = 1;
            cam_back.color = 2;
            A_OLED = 15;  //スタート画面に行く
          }
          else if(Button_selectCF == 2)  //blueが選択されていたら
          {
            cam_front.color = 2;
            cam_back.color = 1;
            A_OLED = 15;  //スタート画面に行く
          }
          else if(Button_selectCF == 1)  //exitが選択されていたら
          {
            A_OLED = 0;  //メニュー画面に戻る
          }
          address = 0x00;  //EEPROMのアドレスを0x00にする（リセット）
          address = sizeof(line.LINE_Level) + sizeof(RA_size) + sizeof(val_max);  //アドレスを次の変数のアドレスにする
          EEPROM.put(address, Button_selectCF);  //EEPROMにボールの閾値を保存
        }
      }
    }
    else if(A_OLED == 15)  //トグルを傾けたらロボット動作開始
    {
      if(A_OLED != B_OLED){  //ステートが変わったときのみ実行(初期化)
        Button_select = 1;  //ボタンの選択(setDir)をデフォルトにする
        B_OLED = A_OLED;
      };

      //OLEDの初期化
      display.display();
      display.clearDisplay();

      display.setTextSize(3);
      display.setTextColor(WHITE);
      display.setCursor(22,0);
      display.println("START");

      display.setTextSize(1);
      display.setCursor(38,35);
      display.println("Dir :");
      display.setTextSize(2);
      display.setCursor(80,30);
      display.println(int(ac.getnowdir()));

      //角度を再設定させるか、もとの選択画面に戻るかを決めるスイッチについての設定
      display.setTextSize(1);
      display.setTextColor(WHITE);
      if(Button_select == 0)  //exitが選択されていたら
      {
        if(flash_OLED == 0){  //白黒反転　何秒かの周期で白黒が変化するようにタイマーを使っている（flash_OLEDについて調べたらわかる）
          display.setTextColor(BLACK, WHITE);
        }
        else{
          display.setTextColor(WHITE);
        }
      }
      display.setCursor(0,55);
      display.println("Exit");

      display.setTextColor(WHITE);
      if(Button_select == 1)  //setDirが選択されていたら（デフォルトはこれ）
      {
        if(flash_OLED == 0){  //白黒反転　何秒かの周期で白黒が変化するようにタイマーを使っている（flash_OLEDについて調べたらわかる）
          display.setTextColor(BLACK, WHITE);
        }
        else{
          display.setTextColor(WHITE);
        }
      }
      display.setCursor(50,55);
      display.println("SetDir");

      display.setTextColor(WHITE);
      if(Button_select == 2)  //noneMが選択されていたら（デフォルトはこれ）
      {
        if(flash_OLED == 0){  //白黒反転　何秒かの周期で白黒が変化するようにタイマーを使っている（flash_OLEDについて調べたらわかる）
          display.setTextColor(BLACK, WHITE);
        }
        else{
          display.setTextColor(WHITE);
        }
      }
      display.setCursor(92,55);
      display.println("NoneM");

      //タクトスイッチが押されたら(手を離されるまで次のステートに行かせたくないため、変数aaを使っている)
      if(aa == 0){
        if(digitalRead(Tact_Switch[1]) == LOW){  //タクトスイッチが押されたら
          aa = 1;
        }
      }else{
        if(digitalRead(Tact_Switch[1]) == HIGH){  //タクトスイッチが手から離れたら
          
          if(Button_select == 0)  //exitが選択されていたら
          {
            A_OLED = 0;  //メニュー画面に戻る
          }
          else if(Button_select == 1)  //SetDir Againが選択されていたら
          {
            ac.setup_2();  //姿勢制御の値リセットするぜい
          }
          else if(Button_select == 2)  //NoneMが選択されていたら
          {
            MOTOR.NoneM_flag = 1;  //モーター動作なしバージョンのフラグを立てる
          }
          aa = 0;
        }
      }
      if(digitalRead(Toggle_Switch) != toogle)  //
      {
        toogle = digitalRead(Toggle_Switch);
        display.clearDisplay(); //初期化してI2Cバスを解放する
        break;
      }
    }
    else if(A_OLED == 20)  //Set Line
    {
      if(A_OLED != B_OLED){  //ステートが変わったときのみ実行(初期化)
        Button_select = 0;  //ボタンの選択(next)をデフォルトにする
        B_OLED = A_OLED;
      };

      display.display();
      display.clearDisplay();

      display.setTextSize(2);
      display.setTextColor(WHITE);
      display.setCursor(16,0);
      display.println("Set Line");

      display.fillTriangle(110, 33, 104, 27, 104, 39, WHITE);  //▶の描画
      display.fillTriangle(18, 33, 24, 27, 24, 39, WHITE);  //◀の描画

      //数字を中央揃えにするためのコード
      display.setTextSize(3);
      display.setTextColor(WHITE);
      if(line.LINE_Level >= 1000){      //4桁の場合
        display.setCursor(28,22);
      }else if(line.LINE_Level >= 100){ //3桁の場合
        display.setCursor(40,22);
      }else if(line.LINE_Level >= 10){  //2桁の場合
        display.setCursor(48,22);
      }else{                       //1桁の場合
        display.setCursor(56,22);
      }
      display.println(line.LINE_Level);  //ラインの閾値を表示

      display.setTextSize(1);
      if(flash_OLED == 0){  //白黒反転　何秒かの周期で白黒が変化するようにタイマーを使っている（flash_OLEDについて調べたらわかる）
        display.setTextColor(BLACK, WHITE);
      }
      else{
        display.setTextColor(WHITE);
      }
      display.setCursor(44,56);
      display.println("Confirm");

      //タクトスイッチが押されたら(手を離されるまで次のステートに行かせたくないため、変数aaを使っている)
      //タクトスイッチが押されたら、メニューに戻る
      if(aa == 0){
        if(digitalRead(Tact_Switch[1]) == LOW){  //タクトスイッチが押されたら
          aa = 1;
          Serial6.write(38);
          Serial6.write(0);
          Serial6.write(line.LINE_Level);
          Serial6.write(37);
          Serial.print("!!!!!!");
        }
      }else{
        if(digitalRead(Tact_Switch[1]) == HIGH){  //タクトスイッチが手から離れたら
          address = 0x00;  //EEPROMのアドレスを0x00にする
          // line.LINE_Level = 700;  //初めにデータをセットしておかなければならない
          EEPROM.put(address, line.LINE_Level);  //EEPROMにラインの閾値を保存

          A_OLED = 0;  //メニュー画面へ戻る
          aa = 0;
        }
      }
    }
    else if(A_OLED == 30)  //Check Line
    {
      if(A_OLED != B_OLED){  //ステートが変わったときのみ実行(初期化)
        Button_select = 0;  //ボタンの選択(next)をデフォルトにする
        B_OLED = A_OLED;
      };
      
      display.display();
      display.clearDisplay();
      line.getLINE_Vec();
      //ラインの位置状況マップを表示する
      display.drawCircle(32, 32, 20, WHITE);  //○ 20
      display.drawCircle(32, 32, 30, WHITE);  //○ 20

      //ラインの直線と円の交点の座標を求める
      line_y = line.dis * cos(radians(line.ang));  //ラインのx座標
      line_x = line.dis * sin(radians(line.ang));  //ラインのy座標

      Ax = line_x - line_y * sqrt(9 - pow(line.dis, 2)) / line.dis;
      Ay = line_y + line_x * sqrt(9 - pow(line.dis, 2)) / line.dis;
      Bx = line_x + line_y * sqrt(9 - pow(line.dis, 2)) / line.dis;
      By = line_y - line_x * sqrt(9 - pow(line.dis, 2)) / line.dis;


      //ラインの線の座標をOLEDでの座標に変換(-1~1の値を0~60の値に変換)
      OLED_line_ax = map(Ax, 3, -3, 60, 0);  //ラインの線のA点のx座標
      OLED_line_ay = map(Ay, 3, -3, 0, 60);  //ラインの線のA点のy座標
      OLED_line_bx = map(Bx, 3, -3, 60, 0);  //ラインの線のB点のx座標
      OLED_line_by = map(By, 3, -3, 0, 60);  //ラインの線のB点のy座標

      if(line.LINE_on == 1){  //ラインがロボットの下にある
        //ラインの線を表示
        display.drawLine((OLED_line_ax + 2), (62 - OLED_line_ay), (OLED_line_bx + 2), (62 - OLED_line_by), WHITE);
      }

      //"Line"と表示する
      display.setTextSize(2);
      display.setTextColor(WHITE);
      display.setCursor(68,0);
      display.println("Line");

      //ここから下のコードのテキストをsize1にする
      display.setTextSize(1);
      display.setTextColor(WHITE);

      //ラインの角度を表示する
      display.setCursor(68,25);
      display.println("Dir:");
      if(line.LINE_on == 1){  //ラインがロボットの下にある
        display.setCursor(96,25);
        display.println(int(line.ang));
      }
      else{  //ラインがロボットの下にない
        display.fillRect(96, 25, 34, 10, WHITE);
      }

      //ラインの距離を表示する
      display.setCursor(68,39);
      display.println("far:");
      if(line.LINE_on == 1){  //ラインがロボットの下にある
        display.setCursor(96,39);
        display.println(line.dis);
      }
      else{  //ラインがロボットの下にない
        display.fillRect(96, 39, 34, 10, WHITE);
      }

      //タクトスイッチが押されたら(手を離されるまで次のステートに行かせたくないため、変数aaを使っている)
      //タクトスイッチが押されたら、メニューに戻る
      if(aa == 0){
        if(digitalRead(Tact_Switch[1]) == LOW){  //タクトスイッチが押されたら
          aa = 1;
        }
      }else{
        if(digitalRead(Tact_Switch[1]) == HIGH){  //タクトスイッチが手から離れたら
          A_OLED = 0;  //メニュー画面へ戻る
          aa = 0;
        }
      }

      // //白線の平均値を表示する
      // display.setCursor(68,44);
      // display.println("Whi:");
      // display.setCursor(96,44);
      // display.println(Lwhite);

      // //緑コートの平均値を表示する
      // display.setCursor(68,56);
      // display.println("Gre:");
      // display.setCursor(96,56);
      // display.println(Lgreen);
    }
    else if(A_OLED == 40){
      if(A_OLED != B_OLED){  //ステートが変わったときのみ実行(初期化)
        Button_select = 0;  //ボタンの選択(next)をデフォルトにする
        B_OLED = A_OLED;
      };

      display.display();
      display.clearDisplay();

      display.setTextSize(2);
      display.setTextColor(WHITE);
      display.setCursor(16,0);
      display.println("Set Ball");

      display.fillTriangle(110, 33, 104, 27, 104, 39, WHITE);  //▶の描画
      display.fillTriangle(18, 33, 24, 27, 24, 39, WHITE);  //◀の描画

      //数字を中央揃えにするためのコード
      display.setTextSize(3);
      display.setTextColor(WHITE);
      if(ball.get_th >= 1000){      //4桁の場合
        display.setCursor(28,22);
      }else if(ball.get_th >= 100){ //3桁の場合
        display.setCursor(40,22);
      }else if(ball.get_th >= 10){  //2桁の場合
        display.setCursor(48,22);
      }else{                       //1桁の場合
        display.setCursor(56,22);
      }
      display.println(ball.get_th);  //ラインの閾値を表示

      display.setTextSize(1);
      if(flash_OLED == 0){  //白黒反転　何秒かの周期で白黒が変化するようにタイマーを使っている（flash_OLEDについて調べたらわかる）
        display.setTextColor(BLACK, WHITE);
      }
      else{
        display.setTextColor(WHITE);
      }
      display.setCursor(44,56);
      display.println("Confirm");

      //タクトスイッチが押されたら(手を離されるまで次のステートに行かせたくないため、変数aaを使っている)
      //タクトスイッチが押されたら、メニューに戻る
      if(aa == 0){
        if(digitalRead(Tact_Switch[1]) == LOW){  //タクトスイッチが押されたら
          aa = 1;
        }
      }else{
        if(digitalRead(Tact_Switch[1]) == HIGH){  //タクトスイッチが手から離れたら
          address = 0x00;  //EEPROMのアドレスを0x00にする
          address += sizeof(line.LINE_Level) + sizeof(RA_size) + sizeof(val_max) + sizeof(Button_selectCF);  //アドレスを次の変数のアドレスにする
          for(int i = 0; i < 6; i++){
            address += sizeof(check_val[i]);
          }  
          EEPROM.put(address, ball.get_th);  //EEPROMにラインの閾値を保存
          A_OLED = 0;  //メニュー画面へ戻る
          aa = 0;
        }
      }
    }
    else if(A_OLED == 50)  //Check Ball
    {
      if(A_OLED != B_OLED){  //ステートが変わったときのみ実行(初期化)
        Button_select = 0;  //ボタンの選択(next)をデフォルトにする
        B_OLED = A_OLED;
      };
      ball.getBallposition();

      display.display();
      display.clearDisplay();

      //ボールの座標をOLED用にする（無理やりint型にしてOLEDのドットに合わせる）
      OLED_ball_x = map(ball.far * sin(radians(ball.ang)), -150, 150, 0, 60);  //
      OLED_ball_y = map(ball.far * cos(radians(ball.ang)), -150, 150, 0, 60);  //

      //ボールの位置状況マップを表示する
      display.drawCircle(32, 32, 30, WHITE);  //○ 30
      display.drawCircle(32, 32, 20, WHITE);  //○ 20
      display.drawCircle(32, 32, 10, WHITE);  //○ 10
      display.drawLine(2, 32, 62, 32, WHITE); //-
      display.drawLine(32, 2, 32, 62, WHITE); //|

      //ボールの位置を表示する
      if(1)  //ボールがあれば
      {
        display.fillCircle((OLED_ball_x + 2), (62 - OLED_ball_y), 5, WHITE);
      }

      //"Ball"と表示する
      display.setTextSize(2);
      display.setTextColor(WHITE);
      display.setCursor(68,0);
      display.println("Ball");

      //ここから下のコードのテキストをsize1にする
      display.setTextSize(1);
      display.setTextColor(WHITE);

      //ボールの角度を表示する
      display.setCursor(68,24);
      display.println("Dir:");
      if(ball.flag){  //ボールがあれば値を表示
        display.setCursor(96,24);
        display.println(int(ball.ang));
      }
      else{  //ボールがなければ白い四角形を表示
        display.fillRect(96, 24, 34, 10, WHITE);
      }

      //ボールの距離を表示する
      display.setCursor(68,38);
      display.println("far:");
      if(ball.flag){  //ボールがあれば値を表示
        display.setCursor(96,38);
        display.println(int(ball.far));
      }
      else{  //ボールがなければ白い四角形を表示
        display.fillRect(96, 38, 34, 10, WHITE);
      }

      //タクトスイッチが押されたら(手を離されるまで次のステートに行かせたくないため、変数aaを使っている)
      //タクトスイッチが押されたら、メニューに戻る
      if(aa == 0){
        if(digitalRead(Tact_Switch[1]) == LOW){  //タクトスイッチが押されたら
          aa = 1;
        }
      }else{
        if(digitalRead(Tact_Switch[1]) == HIGH){  //タクトスイッチが手から離れたら
          A_OLED = 0;  //メニュー画面へ戻る
          aa = 0;
        }
      }
    }
    else if(A_OLED == 60)  //Set Motar
    {
      if(A_OLED != B_OLED){  //ステートが変わったときのみ実行(初期化)
        Button_select = 0;  //ボタンの選択(next)をデフォルトにする
        B_OLED = A_OLED;
      };

      display.display();
      display.clearDisplay();

      display.setTextSize(2);
      display.setTextColor(WHITE);
      display.setCursor(14,0);
      display.println("Set Motar");

      display.fillTriangle(110, 33, 104, 27, 104, 39, WHITE);  //▶の描画
      display.fillTriangle(18, 33, 24, 27, 24, 39, WHITE);  //◀の描画

      //数字を中央揃えにするためのコード
      display.setTextSize(3);
      display.setTextColor(WHITE);
      if(val_max >= 1000){      //4桁の場合
        display.setCursor(28,22);
      }else if(val_max >= 100){ //3桁の場合
        display.setCursor(40,22);
      }else if(val_max >= 10){  //2桁の場合
        display.setCursor(48,22);
      }else{                       //1桁の場合
        display.setCursor(56,22);
      }
      display.println(val_max);  //ラインの閾値を表示

      display.setTextSize(1);
      if(flash_OLED == 0){  //白黒反転　何秒かの周期で白黒が変化するようにタイマーを使っている（flash_OLEDについて調べたらわかる）
        display.setTextColor(BLACK, WHITE);
      }
      else{
        display.setTextColor(WHITE);
      }
      display.setCursor(44,56);
      display.println("Confirm");

      //タクトスイッチが押されたら(手を離されるまで次のステートに行かせたくないため、変数aaを使っている)
      //タクトスイッチが押されたら、メニューに戻る
      if(aa == 0){
        if(digitalRead(Tact_Switch[1]) == LOW){  //タクトスイッチが押されたら
          aa = 1;
        }
      }else{
        if(digitalRead(Tact_Switch[1]) == HIGH){  //タクトスイッチが手から離れたら
          address = 0x00;  //EEPROMのアドレスを0x00にする（リセット）
          address = sizeof(line.LINE_Level) + sizeof(RA_size);  //アドレスを次の変数のアドレスにする
          // val_max = 100;  //初めにデータをセットしておかなければならない
          EEPROM.put(address, val_max);  //EEPROMにボールの閾値を保存
          A_OLED = 0;  //メニュー画面へ戻る
          aa = 0;
        }
      }
    }
    else if(A_OLED == 70)  //デバックとりやすくするゼット (計6個表示させられる) --------------------------------------------------------------------ここにみた変数名を入力してね---------------------
    {
      //OLEDの初期化
      display.display();
      display.clearDisplay();

      //テキストサイズと色の設定
      display.setTextSize(1);
      display.setTextColor(WHITE);
      if(check_select == 0){
        if(flash_OLED == 0){  //白黒反転　何秒かの周期で白黒が変化するようにタイマーを使っている（flash_OLEDについて調べたらわかる）
          display.setTextColor(BLACK, WHITE);
        }
        else{
          display.setTextColor(WHITE);
        }
      }
      display.setCursor(0,0);  //1列目
      display.println("goAng_45");  //この中に変数名を入力
      display.setCursor(60,0);
      display.setTextColor(WHITE);
      display.println(":");    //この中に知りたい変数を入力
      display.setCursor(66,0);
      display.println(check_val[0]);    //この中に知りたい変数を入力

      if(check_select == 1){
        if(flash_OLED == 0){  //白黒反転　何秒かの周期で白黒が変化するようにタイマーを使っている（flash_OLEDについて調べたらわかる）
          display.setTextColor(BLACK, WHITE);
        }
        else{
          display.setTextColor(WHITE);
        }
      }

      display.setCursor(0,10);  //2列目
      display.println("goAng_90");  //この中に変数名を入力
      display.setCursor(60,10);
      display.setTextColor(WHITE);
      display.println(":");    //この中に知りたい変数を入力
      display.setCursor(66,10);
      display.println(check_val[1]);    //この中に知りたい変数を入力

      if(check_select == 2){
        if(flash_OLED == 0){  //白黒反転　何秒かの周期で白黒が変化するようにタイマーを使っている（flash_OLEDについて調べたらわかる）
          display.setTextColor(BLACK, WHITE);
        }
        else{
          display.setTextColor(WHITE);
        }
      }

      display.setCursor(0,20); //3列目
      display.println("goAng_180");  //この中に変数名を入力
      display.setCursor(60,20);
      display.setTextColor(WHITE);
      display.println(":");    //この中に知りたい変数を入力
      display.setCursor(66,20);
      display.println(check_val[2]);    //この中に知りたい変数を入力

      if(check_select == 3){
        if(flash_OLED == 0){  //白黒反転　何秒かの周期で白黒が変化するようにタイマーを使っている（flash_OLEDについて調べたらわかる）
          display.setTextColor(BLACK, WHITE);
        }
        else{
          display.setTextColor(WHITE);
        }
      }

      display.setCursor(0,30); //4列目
      display.println("A");  //この中に変数名を入力
      display.setCursor(60,30);
      display.setTextColor(WHITE);
      display.println(":");    //この中に知りたい変数を入力
      display.setCursor(66,30);
      display.println(check_val[3]);    //この中に知りたい変数を入力

      if(check_select == 4){
        if(flash_OLED == 0){  //白黒反転　何秒かの周期で白黒が変化するようにタイマーを使っている（flash_OLEDについて調べたらわかる）
          display.setTextColor(BLACK, WHITE);
        }
        else{
          display.setTextColor(WHITE);
        }
      }

      display.setCursor(0,40); //5列目
      display.println("B");  //この中に変数名を入力
      display.setCursor(60,40);
      display.setTextColor(WHITE);
      display.println(":");    //この中に知りたい変数を入力
      display.setCursor(66,40);
      display.println(check_val[4]);    //この中に知りたい変数を入力

      if(check_select == 5){
        if(flash_OLED == 0){  //白黒反転　何秒かの周期で白黒が変化するようにタイマーを使っている（flash_OLEDについて調べたらわかる）
          display.setTextColor(BLACK, WHITE);
        }
        else{
          display.setTextColor(WHITE);
        }
      }

      display.setCursor(0,50); //6列目
      display.println("C");  //この中に変数名を入力
      display.setCursor(60,50);
      display.setTextColor(WHITE);
      display.println(":");    //この中に知りたい変数を入力
      display.setCursor(66,50);
      display.println(check_val[5]);    //この中に知りたい変数を入力

      //タクトスイッチが押されたら(手を離されるまで次のステートに行かせたくないため、変数aaを使っている)
      //タクトスイッチが押されたら、メニューに戻る
      if(aa == 0){
        if(digitalRead(Tact_Switch[1]) == LOW){  //タクトスイッチが押されたら
          aa = 1;
        }
      }else{
        if(digitalRead(Tact_Switch[1]) == HIGH){  //タクトスイッチが手から離れたら
          if(check_flag == 0){
            if(check_select == -1 || check_select == 6){
              A_OLED = 0;  //メニュー画面へ戻る
            }
            else{
              check_flag = 1;
            }
          }
          else{
            check_flag = 0;
            address = 0;
            address += sizeof(line.LINE_Level) + sizeof(RA_size) + sizeof(val_max) + sizeof(Button_selectCF);  //アドレスを次の変数のアドレスにする
            for(int i = 0; i < check_select; i++){
              address += sizeof(check_val[i]);
            }
            EEPROM.put(address, check_val[check_select]);  //EEPROMにボールの閾値を保存
          }

          aa = 0;
        }
      }
    }
    else if(A_OLED == 80){
      if(A_OLED != B_OLED){  //ステートが変わったときのみ実行(初期化)
        Button_select = 0;  //ボタンの選択(next)をデフォルトにする
        B_OLED = A_OLED;
      };
      cam_front.getCamdata();

      display.display();
      display.clearDisplay();

      //ボールの座標をOLED用にする（無理やりint型にしてOLEDのドットに合わせる）
      OLED_cam_x = map((80 - cam_front.Size) * sin(radians(cam_front.ang)), -80, 80, 0, 60);  //
      OLED_cam_y = map((80 - cam_front.Size) * cos(radians(cam_front.ang)), -80, 80, 0, 60);  //

      //ボールの位置状況マップを表示する
      display.drawCircle(32, 32, 30, WHITE);  //○ 30
      display.drawCircle(32, 32, 20, WHITE);  //○ 20
      display.drawCircle(32, 32, 10, WHITE);  //○ 10
      display.drawLine(2, 32, 62, 32, WHITE); //-
      display.drawLine(32, 2, 32, 62, WHITE); //|

      //ボールの位置を表示する
      if(1)  //ボールがあれば
      {
        display.fillCircle((OLED_cam_x + 2), (62 - OLED_cam_y), 5, WHITE);
      }

      //"Ball"と表示する
      display.setTextSize(2);
      display.setTextColor(WHITE);
      display.setCursor(68,0);
      display.println("Cam");

      //ここから下のコードのテキストをsize1にする
      display.setTextSize(1);
      display.setTextColor(WHITE);

      //ボールの角度を表示する
      display.setCursor(68,24);
      display.println("Dir:");
      if(cam_front.on){  //ボールがあれば値を表示
        display.setCursor(96,24);
        display.println(int(cam_front.ang));
      }
      else{  //ボールがなければ白い四角形を表示
        display.fillRect(96, 24, 34, 10, WHITE);
      }

      //ボールの距離を表示する
      display.setCursor(68,38);
      display.println("Size:");
      if(cam_front.on){  //ボールがあれば値を表示
        display.setCursor(96,38);
        display.println(int(cam_front.Size));
      }
      else{  //ボールがなければ白い四角形を表示
        display.fillRect(96, 38, 34, 10, WHITE);
      }

      //タクトスイッチが押されたら(手を離されるまで次のステートに行かせたくないため、変数aaを使っている)
      //タクトスイッチが押されたら、メニューに戻る
      if(aa == 0){
        if(digitalRead(Tact_Switch[1]) == LOW){  //タクトスイッチが押されたら
          aa = 1;
        }
      }else{
        if(digitalRead(Tact_Switch[1]) == HIGH){  //タクトスイッチが手から離れたら
          A_OLED = 0;  //メニュー画面へ戻る
          aa = 0;
        }
      }
    }
    else if(A_OLED == 90){
      if(A_OLED != B_OLED){  //ステートが変わったときのみ実行(初期化)
        Button_select = 0;  //ボタンの選択(next)をデフォルトにする
        B_OLED = A_OLED;
      };
      ball.getBallposition();
      display.display();
      display.clearDisplay();

      //テキストサイズと色の設定
      display.setTextSize(1);
      display.setTextColor(WHITE);

      display.setCursor(0,0);  //1列目
      display.println("Left");  //この中に変数名を入力
      display.setCursor(60,0);
      display.setTextColor(WHITE);
      display.println(":");    //この中に知りたい変数を入力
      display.setCursor(66,0);
      display.println(ball.get_1);    //この中に知りたい変数を入力


      display.setCursor(0,10);  //2列目
      display.println("Right");  //この中に変数名を入力
      display.setCursor(60,10);
      display.setTextColor(WHITE);
      display.println(":");    //この中に知りたい変数を入力
      display.setCursor(66,10);
      display.println(ball.get_2);    //この中に知りたい変数を入力


      display.setCursor(0,20); //3列目
      display.println("sum");  //この中に変数名を入力
      display.setCursor(60,20);
      display.setTextColor(WHITE);
      display.println(":");    //この中に知りたい変数を入力
      display.setCursor(66,20);
      display.println(ball.get_val);    //この中に知りたい変数を入力


      display.setCursor(0,30); //4列目
      display.println("A");  //この中に変数名を入力
      display.setCursor(60,30);
      display.setTextColor(WHITE);
      display.println(":");    //この中に知りたい変数を入力
      display.setCursor(66,30);
      display.println();    //この中に知りたい変数を入力


      display.setCursor(0,40); //5列目
      display.println("B");  //この中に変数名を入力
      display.setCursor(60,40);
      display.setTextColor(WHITE);
      display.println(":");    //この中に知りたい変数を入力
      display.setCursor(66,40);
      display.println();    //この中に知りたい変数を入力


      display.setCursor(0,50); //6列目
      display.println("C");  //この中に変数名を入力
      display.setCursor(60,50);
      display.setTextColor(WHITE);
      display.println(":");    //この中に知りたい変数を入力
      display.setCursor(66,50);
      display.println();    //この中に知りたい変数を入力

      //タクトスイッチが押されたら(手を離されるまで次のステートに行かせたくないため、変数aaを使っている)
      //タクトスイッチが押されたら、メニューに戻る
      if(aa == 0){
        if(digitalRead(Tact_Switch[1]) == LOW){  //タクトスイッチが押されたら
          aa = 1;
        }
      }else{
        if(digitalRead(Tact_Switch[1]) == HIGH){  //タクトスイッチが手から離れたら
          A_OLED = 0;  //メニュー画面へ戻る
          aa = 0;
        }
      }
    }
    else if(A_OLED == 100){
      if(A_OLED != B_OLED){  //ステートが変わったときのみ実行(初期化)
        Button_select = 0;  //ボタンの選択(next)をデフォルトにする
        B_OLED = A_OLED;
      };
      int kick_ = 0;
      display.display();
      display.clearDisplay();

      display.setTextSize(3);
      display.setTextColor(WHITE);
      display.setCursor(22,0);
      display.println("Kick");
      if(digitalRead(Toggle_Switch) != toogle)  //
      {
        toogle = digitalRead(Toggle_Switch);
        kick_ = 1;
      }
      kicker.run(kick_);
      if(aa == 0){
        if(digitalRead(Tact_Switch[1]) == LOW){  //タクトスイッチが押されたら
          aa = 1;
        }
      }else{
        if(digitalRead(Tact_Switch[1]) == HIGH){  //タクトスイッチが手から離れたら
          A_OLED = 0;  //メニュー画面へ戻る
          kicker.stop();
          aa = 0;
        }
      }
    }



    int Left = 0;
    int Right = 0;
    if(Left_A == 0){
      if(digitalRead(Tact_Switch[0]) == LOW){
        Left_A = 1;
        Left_t.reset();
      }
    }
    else if(Left_A == 1){
      if(100 < Left_t.read_ms()){
        if(digitalRead(Tact_Switch[0]) == HIGH){
          Left_A = 2;
        }
      }
      if(800 < Left_t.read_ms()){
        if(digitalRead(Tact_Switch[0]) == LOW){
          Left_A = 5;
        }
      }
    }
    else if(Left_A == 2){
      Left = 1;
      Left_A = 0;
    }
    else if(Left_A == 5){
      if(50 < Left_t.read_ms()){
        if(digitalRead(Tact_Switch[0]) == LOW){
          Left = 1;
          Left_t.reset();
        }
        else{
          Left_A = 0;
        }
      }
    }

    if(Right_A == 0){
      if(digitalRead(Tact_Switch[2]) == LOW){
        Right_A = 1;
        Right_t.reset();
      }
    }
    else if(Right_A == 1){
      if(100 < Right_t.read_ms()){
        if(digitalRead(Tact_Switch[2]) == HIGH){
          Right_A = 2;
        }
      }
      if(800 < Right_t.read_ms()){
        if(digitalRead(Tact_Switch[2]) == LOW){
          Right_A = 5;
        }
      }
    }
    else if(Right_A == 2){
      Right = 1;
      Right_A = 0;
    }
    else if(Right_A == 5){
      if(50 < Right_t.read_ms()){
        if(digitalRead(Tact_Switch[2]) == LOW){
          Right = 1;
          Right_t.reset();
        }
        else{
          Right_A = 0;
        }
      }
    }

    // Serial.print(" Left = ");
    // Serial.print(Left_A);
    // Serial.print(" Right = ");
    // Serial.println(Right_A);
    //ロータリーエンコーダーの値を取得し制御する

    if(A_OLED == 0)  //選択画面にいるときはOLED_selectを変更する
    {
      if(Right == 1)  //回転方向を判定
      {
        OLED_select++;  //次の画面へ
        if(OLED_select > 10)  //選択画面の数以上になったら1に戻す
        {
          OLED_select = 1;
        }
      }
    }
    else if(A_OLED == 10)  //スタート画面にいるときはButton_selectを変更する
    {
      if(Right == 1)  //回転方向を判定
      {
        Button_select = 0;  //next
      }
      else if(Left== 1)
      {
        Button_select = 1;  //exit
      }
    }
    else if(A_OLED == 12)
    {
      if(Right == 1)  //回転方向を判定
      {
        if(Button_selectCF < 2){
          Button_selectCF++;  //next
        }
      }
      else if(Left== 1)
      {
        if(Button_selectCF  > 0){
          Button_selectCF--;  //next
        }
      }
    }
    else if(A_OLED == 15)
    {
      if(Right == 1)  //回転方向を判定
      {
        if(Button_select < 2){
          Button_select++;  //next
        }
      }
      else if(Left== 1)
      {
        if(Button_select  > 0){
          Button_select--;  //next
        }
      }
    }
    else if(A_OLED == 20)  //ラインの閾値を変更する
    {
      if(Right == 1)  //回転方向を判定
      {
        if(line.LINE_Level < 1023)
        {
          line.LINE_Level++;
        }
      }
      else if(Left== 1)
      {
        if(line.LINE_Level > 0)
        {
          line.LINE_Level--;
        }
      }
    }
    else if(A_OLED == 40)  //ボールの閾値を変更する
    {
      if(Right == 1)  //回転方向を判定
      {
        if(ball.get_th < 1023)
        {
          ball.get_th++;
        }
      }
      else if(Left== 1)
      {
        if(ball.get_th > 0)
        {
          ball.get_th--;
        }
      }
    }
    else if(A_OLED == 60)  //モーターの出力を変更する
    {
      if(Right == 1)  //回転方向を判定
      {
        if(val_max < 1023)
        {
          val_max++;
        }
      }
      else if(Left== 1)
      {
        if(val_max > 0)
        {
          val_max--;
        }
      }
    }
    else if(A_OLED == 65){
      if(Right == 1){
        ball.get_th++;
        if(200 < ball.get_th){
          ball.get_th = 0;
        }
      }
      else if(Left == 1){
        ball.get_th--;
        if(ball.get_th < 0){
          ball.get_th = 200;
        }
      }
    }
    else if(A_OLED == 70){
      if(check_flag == 0){
        if(Left == 1){
          check_select++;
          if(check_select > 6){
            check_select = 0;
          }
        }
        else if(Right == 1){
          check_select--;
          if(check_select < -1){
            check_select = 5;
          }
        }
      }
      else{
        if(Right == 1){
          check_val[check_select]++;
        }
        else if(Left == 1){
          check_val[check_select]--;
        }
      }
    }
    // Serial.print("end!!!!!!!!");
    // Serial.print(" check_select : ");
    // Serial.print(check_select);
    // Serial.print(" flag : ");
    // Serial.print(check_flag);
    // cam_front.print();
    // Serial.println();
  }
}