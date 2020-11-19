/*
      包含头文件
*/
#include <Arduino.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>
#include <OneWire.h>


/*
    GPIO宏定义
*/
#define TEMP 5     //ds18b20使用的引脚
#define TEMP2 10    
#define kRecvPin 2   //红外遥控的引脚
#define LED 4        //  LED  提示
#define a 16      //步进电机使用的引脚
#define b 14
#define c 12
#define d 13
#define Fan 15       //风扇控制的引脚    按键5 开风扇   按键 4 关风扇
#define adc A0      //红外测距读取数据的引脚


/*
    OK 开始  0xFF38C7  #  结束     0xFFB04F    
    ↑   设置温度+    0xFF18E7
    *   结束            0xFF6897
    ↓   设置温度-    0xFF4AB5
    1   获取当前温度    0xFFA25D
    2   获取设置的温度   0xFF629D
    3    获取当前adc电压   0xFFE21D
    4    打开风扇       0xFF22DD
    5     关闭风扇      0xFF02FD

    
*/
/*
    变量定义
*/
float myTemp = 40.0;   //自己定义的温度，默认40.0
float tempResult1;     //得到的数据值
float tempResult2;     //得到的数据值
char MODE = 2;    //模式切换
char motorDir = 0;    //    1 上升    0 下降
char motorSpeed = 3;    //电机速度控制
int i = 0;              //计数控制
char t = 0;
int adcVal;       //adc读取数值  0 - 1023
decode_results results;     //  红外遥控器的数据接收缓冲区


//  IIC 单线读温度传感器数据
OneWire ds(TEMP);
OneWire ds2(TEMP2);
/*
  函数声明
*/
IRrecv irrecv(kRecvPin);
void startSystem(void);
void endSystem(void);
float getTemperture(void);
void  motor(int speeds, char dir);
void modeSet(decode_results res);
void setTemperture(char contrl);
/*
     GPIO 初始化函数
*/
void GPIO_Init(void){
    i = 0;
    adcVal = 0;
    pinMode(a,OUTPUT);
    pinMode(b,OUTPUT);
    pinMode(c,OUTPUT);
    pinMode(d,OUTPUT);
    pinMode(LED,OUTPUT);
    pinMode(adc,INPUT);
    pinMode(Fan,OUTPUT);
    Serial.begin(115200);
    irrecv.enableIRIn();  // 开启中断使能
  //  while (!Serial)  // 等待串口连接
      delay(50);
    Serial.println();     //打印使用的GPIO
    Serial.print("IRrecv Pin ");
    Serial.println(kRecvPin);
    Serial.print("DS18B20 Pin");
    Serial.println(TEMP);
    Serial.println(TEMP2);
    Serial.print("LED Pin");
    Serial.println(LED);
    Serial.print("ADC Pin");
    Serial.println(adc);
    Serial.print("Motor a Pin");
    Serial.println(a);
    Serial.print("Motor b Pin");
    Serial.println(b);
    Serial.print("Motor c Pin");
    Serial.println(c);
    Serial.print("Motor d Pin");
    Serial.println(d);
}

/*
  程序开始的地方，在这里初始化变量和GPIO口
*/
void setup() {
   GPIO_Init();
   MotorInit();
   Serial.println("系统初始化完成 .....");
}
/*
    循环执行函数
*/
void loop() {
    RecvData();     //接收红外遥控发送的数据
    if(MODE == 1){    //按键OK按下，电机开始运行
        motorMain();  
    }
    else{
     t ++;
        digitalWrite(b,LOW);
        digitalWrite(b,LOW);
        digitalWrite(c,LOW);
        digitalWrite(d,LOW);
        if(t > 100){
          t = 0;
          digitalWrite(LED,HIGH);
          delay(300);
         // Serial.println("lll");
        }
    }
    i++;
    if (i > 100) {    //每循环
        i = 0;
        digitalWrite(LED,LOW);
        tempResult1 = getTemperture1();
        Serial.println(tempResult1);
        tempResult2 = getTemperture2();
        Serial.println(tempResult2);
    }
}

//获取第一个温度传感器的数据
float getTemperture1(void){
  
  byte data[12];
  byte addr[8];
  
  if ( !ds.search(addr)) {
      //no more sensors on chain, reset search
      ds.reset_search();
      return -125;
  }
  
  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -125;
  }
  
  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return -125;
  }
  
  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end
  
  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad
  
  
  for (int i = 0; i < 9; i++) { // we need 9 bytes
      data[i] = ds.read();
  }
  
  ds.reset_search();
  
  byte MSB = data[1];
  byte LSB = data[0];
  
  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;
  
  return TemperatureSum;
}

//获取第二个温度传感器数据
float getTemperture2(void){
  
  byte data[12];
  byte addr[8];
  
  if ( !ds2.search(addr)) {
      //no more sensors on chain, reset search
      ds2.reset_search();
      return -125;
  }
  
  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -125;
  }
  
  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return -125;
  }
  
  ds2.reset();
  ds2.select(addr);
  ds2.write(0x44,1); // start conversion, with parasite power on at the end
  
  byte present = ds2.reset();
  ds2.select(addr);
  ds2.write(0xBE); // Read Scratchpad
  
  
  for (int i = 0; i < 9; i++) { // we need 9 bytes
      data[i] = ds2.read();
  }
  
  ds2.reset_search();
  
  byte MSB = data[1];
  byte LSB = data[0];
  
  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;
  
  return TemperatureSum;
}

/*设置参考的温度 开始运行时默认40℃
  参数 contrl  1   加设置温度
               0   减设置温度
*/
void setTemperture(char contrl){
   if(contrl == 1){
        myTemp += 0.5;
   }
   else if (contrl == 2){
        myTemp -= 0.5;
   }
    else{
        return;
    }
}

//红外遥控接收数据函数
void RecvData(void){
      char rest = 0;
      if (irrecv.decode(&results)) {
      serialPrintUint64(results.value, HEX);
      Serial.println("");
       if(results.value==0xFFB04F){
            rest = 1;
       }
       else if(results.value==0xFF38C7){
            rest = 2;
       }
       else if(results.value==0xFF18E7){
            rest = 3;
            digitalWrite(LED,HIGH);
       }
       else if(results.value==0xFF4AB5){
            rest = 4;
            digitalWrite(LED,HIGH);
       }
       else if(results.value==0xFFA25D){
            rest = 5;
            digitalWrite(LED,HIGH);
       }
       else if(results.value==0xFF629D){
            rest = 6;
            digitalWrite(LED,HIGH);
       }
       else if(results.value==0xFFE21D){
            rest = 7;
            digitalWrite(LED,HIGH);
       }
       else if(results.value==0xFF22DD){
            rest = 8;
            digitalWrite(LED,HIGH);
       }
       else if(results.value==0xFF02FD){
            rest = 9;
            digitalWrite(LED,HIGH);
       }
       else{
            rest =  0;    //不属于上述情况，赋值 0
       }
       modeSet(rest);   //进行模式匹配，设置对应的函数
       rest = 0;
      irrecv.resume();  // Receive the next value
     /* Serial.println("");
      irrecv.resume();  // Receive the next value
      sensor.requestTemperatures();
      Serial.print("当前温度：");
      Serial.println(sensor.getTempCByIndex(0));
      digitalWrite(LED,HIGH);*/
    }
}

//红外接收数据 匹配成功后执行函数
void modeSet(char res){
      switch(res){
        case 1:
              endSystem();break;
         case 2:
              startSystem();break;
         case 3:
              setTemperture(1),Serial.print("设置温度："),Serial.println(myTemp);break;     //加设置温度
         case 4:
              setTemperture(2),Serial.print("设置温度："),Serial.println(myTemp);break;     //减设置温度
         case 5:
              Serial.print("当前温度："),Serial.println(getTemperture1()),Serial.println(getTemperture2());break;      //获取当前温度
         case 6:
              Serial.print("设置温度："),Serial.println(myTemp);break;     //获取当前设置温度
         case 7:
              Serial.print("当前adc："),Serial.println(analogRead(adc));break;
         case 8:
               digitalWrite(Fan,LOW),Serial.println("关闭风扇");break;
         case 9:
               digitalWrite(Fan,HIGH),Serial.println("打开风扇");break;
         default : break;
      }
      delay(100);
}

//电机初始化函数  初始化前 MODE = 2，  初始化后  MODE = 0
void MotorInit(void)
{
    Serial.println("电机初始化开始");
    digitalWrite(LED,HIGH);   //点亮LED
    motorDir = 1;   //电机上升
    while(1){    //
        motor(motorSpeed,motorDir);     //电机上升
        adcVal = analogRead(adc);     //每循环一次都要读取红外测距的距离值
        i++;
        if(i > 200){        //  每 200个循环打印一次数据到串口，便于调试
            i = 0;
            Serial.print(" adcx :");
            Serial.println(adcVal);
        }
        if(adcVal > 900){     //红外测距采集到最大的模拟数据，电机已经上升到最大高度
          break;    //电机已经上升到最大高度，停止上升，电机初始化完成
        }
    }
    MODE = 0;
    Serial.println("电机初始化结束");
}

/*
  如果电机还未初始化（MODE = 2），则初始化电机，并未开始系统
        在MODE = 2 的情况下，要在电机初始化完成后再按一次按键才能运行系统上
  如果电机已经初始化 （MODE = 0）， 开始系统
*/
void startSystem(void){     
    digitalWrite(LED,HIGH);   //点亮LED指示灯
    if(MODE  == 0){   //电机已经初始化   
        MODE = 1;
        Serial.println(" test 1");
    }
    else if(MODE == 2){   
      MotorInit();
    }
    else{
      MODE = 0;
    }
}

/*
    结束系统，电机恢复到最低位置
    MODE = 2  如果要继续开始运行系统，则需要对电机进行初始化
    在系统开始函数有判断电机是否初始化
*/
void endSystem(void){     
    Serial.println("系统结束");
    MODE = 2;     //MODE 2，恢复初始状态模式
    digitalWrite(LED,HIGH);   //点亮LED指示灯
    motorDir = 1;   //电机下降
    while(1){    //
      motor(motorSpeed,motorDir);     //电机下降
      adcVal = analogRead(adc);       //每循环一次都要读取红外测距的距离值
      i++;
      if(i > 200){      //  每 200个循环打印一次数据到串口，便于调试
          i = 0;
          Serial.print(" adcx :");
          Serial.println(adcVal);
      }
      if(adcVal < 300){     //电机已经下降到最低高度，停止下降，电机系统结束，红外遥控不受影响
          break;    
      }
    }
    Serial.println("电机已恢复到初始位置");
}
/*
    根据温度改变电机高度
*/
void motorMain(void){
  if(tempResult1 > (myTemp + 1)){
      motorDir = 0;   //电机反转
  }
  else if(tempResult1 < (myTemp - 1)){
      motorDir = 1;   //电机正转
  }
  else {    //温度在误差范围内，不转
      motorDir = 2;
  }
  motor(motorSpeed,motorDir);//执行电机函数
}
/*
函数名称： motor
函数参数  speeds  电机的速度控制
          dir     电机方向控制
*/
void  motor(int speeds, char dir)
{
    if(dir == 0){
        pin_a();      //电机正转
        delay(speeds);
        pin_b();
        delay(speeds);
        pin_c();
        delay(speeds);
        pin_d();
        delay(speeds);
    }else if( dir == 1){
        pin_d();      //电机反转
        delay(speeds);
        pin_c();
        delay(speeds);
        pin_b();
        delay(speeds);
        pin_a();      //电机正转
        delay(speeds);
    }
    else {
      return;
    }
}
void pin_a()
{
    digitalWrite(a,HIGH);
    digitalWrite(b,LOW);
    digitalWrite(c,LOW);
    digitalWrite(d,LOW);
}
void pin_b()
{
    digitalWrite(a,LOW);
    digitalWrite(b,HIGH);
    digitalWrite(c,LOW);
    digitalWrite(d,LOW);
}
void pin_c()
{
    digitalWrite(a,LOW);
    digitalWrite(b,LOW);
    digitalWrite(c,HIGH);
    digitalWrite(d,LOW);
}
void pin_d()
{
    digitalWrite(a,LOW);
    digitalWrite(b,LOW);
    digitalWrite(c,LOW);
    digitalWrite(d,HIGH);
}
