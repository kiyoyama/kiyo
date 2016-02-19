#include <ros.h>
#include <geometry_msgs/Pose.h>
#include <SPI.h>
#include <MsTimer2.h>
// ピン定義。
#define PIN_SPI_MOSI 11
#define PIN_SPI_MISO 12
#define PIN_SPI_SCK 13
#define PIN_SPI_SS 10

ros::NodeHandle nh;

geometry_msgs::Pose pose_msgs;
ros::Publisher pose("pose",&pose_msgs);
void L6470_setparam_abspos(long val){L6470_transfer(0x01,3,val);}
void L6470_setparam_elpos(long val){L6470_transfer(0x02,2,val);}
void L6470_setparam_mark(long val){L6470_transfer(0x03,3,val);}
void L6470_setparam_acc(long val){L6470_transfer(0x05,2,val);}
void L6470_setparam_dec(long val){L6470_transfer(0x06,2,val);}
void L6470_setparam_maxspeed(long val){L6470_transfer(0x07,2,val);}
void L6470_setparam_minspeed(long val){L6470_transfer(0x08,2,val);}
void L6470_setparam_fsspd(long val){L6470_transfer(0x15,2,val);}
void L6470_setparam_kvalhold(long val){L6470_transfer(0x09,1,val);}
void L6470_setparam_kvalrun(long val){L6470_transfer(0x0a,1,val);}
void L6470_setparam_kvalacc(long val){L6470_transfer(0x0b,1,val);}
void L6470_setparam_kvaldec(long val){L6470_transfer(0x0c,1,val);}
void L6470_setparam_intspd(long val){L6470_transfer(0x0d,2,val);}
void L6470_setparam_stslp(long val){L6470_transfer(0x0e,1,val);}
void L6470_setparam_fnslpacc(long val){L6470_transfer(0x0f,1,val);}
void L6470_setparam_fnslpdec(long val){L6470_transfer(0x10,1,val);}
void L6470_setparam_ktherm(long val){L6470_transfer(0x11,1,val);}
void L6470_setparam_ocdth(long val){L6470_transfer(0x13,1,val);}
void L6470_setparam_stallth(long val){L6470_transfer(0x14,1,val);}
void L6470_setparam_stepmood(long val){L6470_transfer(0x16,1,val);}
void L6470_setparam_alareen(long val){L6470_transfer(0x17,1,val);}
void L6470_setparam_config(long val){L6470_transfer(0x18,2,val);}

long L6470_getparam_abspos(){return L6470_getparam(0x01,3);}
long L6470_getparam_elpos(){return L6470_getparam(0x02,2);}
long L6470_getparam_mark(){return L6470_getparam(0x03,3);}
long L6470_getparam_speed(){return L6470_getparam(0x04,3);}
long L6470_getparam_acc(){return L6470_getparam(0x05,2);}
long L6470_getparam_dec(){return L6470_getparam(0x06,2);}
long L6470_getparam_maxspeed(){return L6470_getparam(0x07,2);}
long L6470_getparam_minspeed(){return L6470_getparam(0x08,2);}
long L6470_getparam_fsspd(){return L6470_getparam(0x15,2);}
long L6470_getparam_kvalhold(){return L6470_getparam(0x09,1);}
long L6470_getparam_kvalrun(){return L6470_getparam(0x0a,1);}
long L6470_getparam_kvalacc(){return L6470_getparam(0x0b,1);}
long L6470_getparam_kvaldec(){return L6470_getparam(0x0c,1);}
long L6470_getparam_intspd(){return L6470_getparam(0x0d,2);}
long L6470_getparam_stslp(){return L6470_getparam(0x0e,1);}
long L6470_getparam_fnslpacc(){return L6470_getparam(0x0f,1);}
long L6470_getparam_fnslpdec(){return L6470_getparam(0x10,1);}
long L6470_getparam_ktherm(){return L6470_getparam(0x11,1);}
long L6470_getparam_adcout(){return L6470_getparam(0x12,1);}
long L6470_getparam_ocdth(){return L6470_getparam(0x13,1);}
long L6470_getparam_stallth(){return L6470_getparam(0x14,1);}
long L6470_getparam_stepmood(){return L6470_getparam(0x16,1);}
long L6470_getparam_alareen(){return L6470_getparam(0x17,1);}
long L6470_getparam_config(){return L6470_getparam(0x18,2);}
long L6470_getparam_status(){return L6470_getparam(0x19,2);}


void L6470_run(int dia,long spd){
  if(dia==1)
    L6470_transfer(0x51,3,spd);
  else
    L6470_transfer(0x50,3,spd);
}
void L6470_stepclock(int dia){
  if(dia==1)
    L6470_transfer(0x59,0,0);    
  else
    L6470_transfer(0x58,0,0);
}
void L6470_move(int dia,long n_step){
  if(dia==1)
    L6470_transfer(0x41,3,n_step);
  else
    L6470_transfer(0x40,3,n_step);
}
void L6470_goto(long pos){
  L6470_transfer(0x60,3,pos);
}
void L6470_gotodia(int dia,int pos){
  if(dia==1)    
    L6470_transfer(0x69,3,pos);
  else    
    L6470_transfer(0x68,3,pos);
}
void L6470_gountil(int act,int dia,long spd){
  if(act==1)
    if(dia==1)
      L6470_transfer(0x8b,3,spd);
    else
      L6470_transfer(0x8a,3,spd);
  else
    if(dia==1)
      L6470_transfer(0x83,3,spd);
    else
      L6470_transfer(0x82,3,spd);
}  
void L6470_relesesw(int act,int dia){
  if(act==1)
    if(dia==1)
      L6470_transfer(0x9b,0,0);
    else
      L6470_transfer(0x9a,0,0);
  else
    if(dia==1)
      L6470_transfer(0x93,0,0);
    else
      L6470_transfer(0x92,0,0);
}
void L6470_gohome(){
  L6470_transfer(0x70,0,0);
}
void L6470_gomark(){
  L6470_transfer(0x78,0,0);
}
void L6470_resetpos(){
  L6470_transfer(0xd8,0,0);
}
void L6470_resetdevice(){
  L6470_send(0x00);//nop命令
  L6470_send(0x00);
  L6470_send(0x00);
  L6470_send(0x00);
  L6470_send(0xc0);
}
void L6470_softstop(){
  L6470_transfer(0xb0,0,0);
}
void L6470_hardstop(){
  L6470_transfer(0xb8,0,0);
}
void L6470_softhiz(){
  L6470_transfer(0xa0,0,0);
}
void L6470_hardhiz(){
  L6470_transfer(0xa8,0,0);
}
long L6470_getstatus(){
  long val=0;
  L6470_send(0xd0);
  for(int i=0;i<=1;i++){
    val = val << 8;
    digitalWrite(PIN_SPI_SS, LOW); // ~SSイネーブル。
    val = val | SPI.transfer(0x00); // アドレスもしくはデータ送信。
    digitalWrite(PIN_SPI_SS, HIGH); // ~SSディスエーブル 
  }
  return val;
}
//----------------------------------------------------------
void L6470_transfer(int add,int bytes,long val){
  int data[3];
  while(!busy_flag()){
  }
  L6470_send(add);
  for(int i=0;i<=bytes-1;i++){
    data[i] = val & 0xff;  
    val = val >> 8;
  }
  if(bytes==3){
    L6470_send(data[2]);
  }
  if(bytes>=2){
    L6470_send(data[1]);
  }
  if(bytes>=1){
    L6470_send(data[0]);
  }
}
void L6470_send(unsigned char add_or_val){
  digitalWrite(PIN_SPI_SS, LOW); // ~SSイネーブル。
  SPI.transfer(add_or_val); // アドレスもしくはデータ送信。
  digitalWrite(PIN_SPI_SS, HIGH); // ~SSディスエーブル。
}
void L6470_busydelay(long time){//BESYが解除されるまで待機
  while(!busy_flag()){
  }
  delay(time);
}
long L6470_getparam(int add,int bytes){
  long val=0;
  int send_add = add | 0x20;
  L6470_send(send_add);
  for(int i=0;i<=bytes-1;i++){
    val = val << 8;
    digitalWrite(PIN_SPI_SS, LOW); // ~SSイネーブル。
    val = val | SPI.transfer(0x00); // アドレスもしくはデータ送信。
    digitalWrite(PIN_SPI_SS, HIGH); // ~SSディスエーブル 
  }
  return val;
}
//----------------------------------------------------
int busy_flag(){ 
int sta=L6470_getparam_status();
sta = sta >> 1;
sta = sta & 0x0001;
return sta;
}

/*
statusレジスタbusyフラグ：[0:ビジー][1:アイドル]。
FLAGピン（オープンドレイン）：[0:GNDレベル（トランジスタON）][1:プルアップ時HIGHレベル（トランジスタOFF）]
*/

void setup(){
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
  // put your setup code here, to run once:
  //Serial.begin(9600);
  
  nh.initNode();
  nh.advertise(pose);
}

void loop() {
  // put your main code here, to run repeatedly:
  pose_msgs.position.x = 1.0;
  pose_msgs.position.y = 2.0;
  pose_msgs.position.z = 3.0;
  pose.publish(&pose_msgs);
  L6470_move(1,1600);//正転方向に1600ステップする 
  nh.spinOnce();
  
  delay(1000);
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

