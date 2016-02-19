#include <SPI.h>
#include <MsTimer2.h>
#include <ros.h>
#include <geometry_msgs/Pose.h>

// ピン定義。
#define PIN_SPI_MOSI 11
#define PIN_SPI_MISO 12
#define PIN_SPI_SCK 13
#define PIN_SPI_SS 10

ros::NodeHandle nh;

geometry_msgs::Pose pose_msgs;
ros::Publisher pose("pose",&pose_msgs);

void setup()
{  
  nh.initNode();
  nh.advertise(pose);
  pose_msgs.position.x = 1.0;
  pose_msgs.position.y = 2.0;
  pose_msgs.position.z = 3.0;
  pose.publish(&pose_msgs);
  nh.spinOnce();
  delay(1000);
  pinMode(PIN_SPI_MOSI, OUTPUT);
  pinMode(PIN_SPI_MISO, INPUT);
  pinMode(PIN_SPI_SCK, OUTPUT);
  pinMode(PIN_SPI_SS, OUTPUT);
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  Serial.begin(9600);
  digitalWrite(PIN_SPI_SS, HIGH);
 
  L6470_resetdevice(); //L6470リセット
  L6470_setup();  //L6470を設定  L6470_resetdevice(); //L6470リセット

  MsTimer2::set(51, fulash);//シリアルモニター用のタイマー割り込み
  MsTimer2::start();
  
  delay(4000);
  
  L6470_move(1,1600);//正転方向に1600ステップする 
  L6470_busydelay(5000); //前の処理が終わってから5秒待つ
  L6470_run(0,10000);//逆転方向に150step/sの速さで回転
  delay(6000);//前の処理が始まってから5秒待つ
  L6470_softstop();//回転停止、保持トルクあり
  L6470_busydelay(5000);
  L6470_goto(0x6789);//座標0x6789に最短でいける回転方向で移動
  L6470_busydelay(5000);
  L6470_run(0,0x4567);//逆転方向に267step/sの速さで回転
  delay(6000);
  L6470_hardhiz();//回転急停止、保持トルクなし*/
  delay(6000);
}

void loop(){

}

void L6470_setup(){
L6470_setparam_acc(0x40); //[R, WS] 加速度default 0x08A (12bit) (14.55*val+14.55[step/s^2])
L6470_setparam_dec(0x40); //[R, WS] 減速度default 0x08A (12bit) (14.55*val+14.55[step/s^2])
L6470_setparam_maxspeed(0x40); //[R, WR]最大速度default 0x041 (10bit) (15.25*val+15.25[step/s])
L6470_setparam_minspeed(0x01); //[R, WS]最小速度default 0x000 (1+12bit) (0.238*val[step/s])
L6470_setparam_fsspd(0x3ff); //[R, WR]μステップからフルステップへの切替点速度default 0x027 (10bit) (15.25*val+7.63[step/s])
L6470_setparam_kvalhold(0x20); //[R, WR]停止時励磁電圧default 0x29 (8bit) (Vs[V]*val/256)
L6470_setparam_kvalrun(0x20); //[R, WR]定速回転時励磁電圧default 0x29 (8bit) (Vs[V]*val/256)
L6470_setparam_kvalacc(0x20); //[R, WR]加速時励磁電圧default 0x29 (8bit) (Vs[V]*val/256)
L6470_setparam_kvaldec(0x20); //[R, WR]減速時励磁電圧default 0x29 (8bit) (Vs[V]*val/256)

L6470_setparam_stepmood(0x03); //ステップモードdefault 0x07 (1+3+1+3bit)
}

void fulash(){
  Serial.print("0x");
  Serial.print( L6470_getparam_abspos(),HEX);
  Serial.print("  ");
  Serial.print("0x");
  Serial.println( L6470_getparam_speed(),HEX);
}

