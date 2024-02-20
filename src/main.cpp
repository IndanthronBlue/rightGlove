#include <Arduino.h>
#include <Wire.h>
#include <JY901.h>
#include <U8g2lib.h>

/* 引脚参数 */
#define bendsensor1 32
#define bendsensor2 33
#define bendsensor3 34
#define bendsensor4 35
#define bendsensor5 27 
#define led_R 4
#define led_G 16
#define led_B 17
#define JY901_SDA 21
#define JY901_SCL 22
#define OLED_SDA 18
#define OLED_SCL 23

/* 双路I2C */
TwoWire I2C_JY901 = TwoWire(0);
TwoWire I2C_OLED = TwoWire(1);

/* Oled 实例化 */
U8G2_SH1106_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/OLED_SCL, /* data=*/OLED_SDA, /* reset=*/U8X8_PIN_NONE);  // SDA:21 scl:22
const char* state = "FxxK";

/* 运动参数初始值 */
float angle_x_init, angle_y_init, angle_z_init; //角度
float blend1_max, blend1_min, blend2_max, blend2_min, blend3_max, blend3_min, blend4_max, blend4_min, blend5_max, blend5_min; //弯曲角度映射范围

/* 运动参数读数 */
float acc_x, acc_y, acc_z; //加速度
float gyro_x, gyro_y, gyro_z; //角速度
float angle_x, angle_y, angle_z; //角度
float blend1, blend2, blend3, blend4, blend5; //弯曲角度

void setup() {
  
  //打开串口
  Serial.begin(115200);

  //设置双路I2C引脚参数
  I2C_JY901.setPins(JY901_SDA, JY901_SCL);
  I2C_OLED.setPins(OLED_SDA, OLED_SCL);

  //启动IIC总线
  I2C_JY901.begin();
  I2C_OLED.begin();

  delay(100);

  // 屏幕初始化
  u8g2.begin();
  u8g2.enableUTF8Print();

  //打开jy901的IIC通道
  JY901.StartIIC();

  //设置引脚模式
  pinMode(bendsensor1, INPUT);
  pinMode(bendsensor2, INPUT);
  pinMode(bendsensor3, INPUT);
  pinMode(bendsensor4, INPUT);
  pinMode(bendsensor5, INPUT);
  pinMode(led_R, OUTPUT);
  pinMode(led_G, OUTPUT);
  pinMode(led_B, OUTPUT);

  //获取姿态初始参数以及弯曲角度映射范围，用以消除偏移量
  getInitValue();


}

void loop() {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_wqy15_t_gb2312);
    u8g2.drawUTF8(60, 40, state);
  } while (u8g2.nextPage());
  delay(200);
  //Obtain the curvature of five fingers , Store in frame data
  float val1 = analogRead(bendsensor1);
  float val2 = analogRead(bendsensor2);
  float val3 = analogRead(bendsensor3);
  float val4 = analogRead(bendsensor4);
  float val5 = analogRead(bendsensor5);

  Serial.print("val1: " + String(val1));
  Serial.print(" ");
  Serial.print("val2: " + String(val2));
  Serial.print(" ");
  Serial.print("val3: " + String(val3));
  Serial.print(" ");
  Serial.print("val4: " + String(val4));
  Serial.print(" ");
  Serial.print("val5: " + String(val5));

  Serial.println("");

  JY901.GetAcc();
  Serial.print("Acc:");
  Serial.print((float)JY901.stcAcc.a[0]/32768*16);
  Serial.print(" ");
  Serial.print((float)JY901.stcAcc.a[1]/32768*16);
  Serial.print(" ");
  Serial.println((float)JY901.stcAcc.a[2]/32768*16);
  
  JY901.GetGyro();  
  Serial.print("Gyro:");
  Serial.print((float)JY901.stcGyro.w[0]/32768*2000);
  Serial.print(" ");
  Serial.print((float)JY901.stcGyro.w[1]/32768*2000);
  Serial.print(" ");
  Serial.println((float)JY901.stcGyro.w[2]/32768*2000);
  
  JY901.GetAngle();
  Serial.print("Angle:");
  Serial.print((float)JY901.stcAngle.Angle[0]/32768*180);
  Serial.print(" ");
  Serial.print((float)JY901.stcAngle.Angle[1]/32768*180);
  Serial.print(" ");
  Serial.println((float)JY901.stcAngle.Angle[2]/32768*180);
  
  Serial.println("");
  delay(200);

}

//控制RGB颜色
void setRGB(int r, int g, int b){
  analogWrite(led_R, r);
  analogWrite(led_G, g); 
  analogWrite(led_B, b);
}

//在OLED上打印字符串
void printOLED(const char* str, int x = 60, int y = 40){
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_wqy15_t_gb2312);
    u8g2.drawUTF8(x, y, str);
  } while (u8g2.nextPage());
  delay(500);
}

//获取姿态初始参数以及弯曲角度映射范围，用以消除偏移量
void getInitValue(){

  //获取方位角度初始偏移量
  state = "正在获取方位角偏移量";
  printOLED(state);
  
  JY901.GetAngle();
  angle_x_init = (float)JY901.stcAngle.Angle[0]/32768*180;
  angle_y_init = (float)JY901.stcAngle.Angle[1]/32768*180;
  angle_z_init = (float)JY901.stcAngle.Angle[2]/32768*180;
  
  //将结果合并成一行，打印在串口
  Serial.println("angle_x_init: " + String(angle_x_init) + " angle_y_init: " + String(angle_y_init) + " angle_z_init: " + String(angle_z_init));

  delay(1000);

  //获取弯曲角度映射范围
  state = "正在获取弯曲映射范围...";
  printOLED(state);
  state = "请伸直手指";
  printOLED(state);
  blend1_min = analogRead(bendsensor1);
  blend2_min = analogRead(bendsensor2);
  blend3_min = analogRead(bendsensor3);
  blend4_min = analogRead(bendsensor4);
  blend5_min = analogRead(bendsensor5);

  //将结果合并成一行，打印在串口
  Serial.println("blend1_min: " + String(blend1_min) + " blend2_min: " + String(blend2_min) + " blend3_min: " + String(blend3_min) + " blend4_min: " + String(blend4_min) + " blend5_min: " + String(blend5_min));
  
  delay(1000);

  state = "请弯曲手指";
  printOLED(state);
  blend1_max = analogRead(bendsensor1);
  blend2_max = analogRead(bendsensor2);
  blend3_max = analogRead(bendsensor3);
  blend4_max = analogRead(bendsensor4);
  blend5_max = analogRead(bendsensor5);
  delay(1000);

  //将结果合并成一行，打印在串口
  Serial.println("blend1_max: " + String(blend1_max) + " blend2_max: " + String(blend2_max) + " blend3_max: " + String(blend3_max) + " blend4_max: " + String(blend4_max) + " blend5_max: " + String(blend5_max));

  state = "完成校准";
  printOLED(state);
  delay(1000);
}

//获取运动姿态
void getMotionValue(){
  JY901.GetAcc();
  acc_x = (float)JY901.stcAcc.a[0]/32768*16;
  acc_y = (float)JY901.stcAcc.a[1]/32768*16;
  acc_z = (float)JY901.stcAcc.a[2]/32768*16;
  
  JY901.GetGyro();
  gyro_x = (float)JY901.stcGyro.w[0]/32768*2000;
  gyro_y = (float)JY901.stcGyro.w[1]/32768*2000;
  gyro_z = (float)JY901.stcGyro.w[2]/32768*2000;
  
  JY901.GetAngle();
  angle_x = (float)JY901.stcAngle.Angle[0]/32768*180 - angle_x_init;
  angle_y = (float)JY901.stcAngle.Angle[1]/32768*180 - angle_y_init;
  angle_z = (float)JY901.stcAngle.Angle[2]/32768*180 - angle_z_init;
  
  blend1 = map(analogRead(bendsensor1), blend1_min, blend1_max, 0, 90);
  blend2 = map(analogRead(bendsensor2), blend2_min, blend2_max, 0, 90);
  blend3 = map(analogRead(bendsensor3), blend3_min, blend3_max, 0, 90);
  blend4 = map(analogRead(bendsensor4), blend4_min, blend4_max, 0, 90);
  blend5 = map(analogRead(bendsensor5), blend5_min, blend5_max, 0, 90);
}