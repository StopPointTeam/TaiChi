#include <Arduino.h>
#include <Wire.h>

#ifdef SENSOR_DEBUG
#include <NeoHWSerial.h>
#endif

#include "sensorTaiChi.h"


Sensor::Sensor()
{
    pinMode(BUTTON_1_OUT, INPUT_PULLUP);
    pinMode(BUTTON_2_OUT, INPUT_PULLUP);

    pinMode(GRAY_1_VCC, OUTPUT);
    digitalWrite(GRAY_1_VCC, HIGH);

    pinMode(GRAY_2_VCC, OUTPUT);
    digitalWrite(GRAY_2_VCC, HIGH);

    pinMode(GRAY_3_VCC, OUTPUT);
    digitalWrite(GRAY_3_VCC, HIGH);

    pinMode(GRAY_4_VCC, OUTPUT);
    digitalWrite(GRAY_4_VCC, HIGH);

    pinMode(GRAY_5_VCC, OUTPUT);
    digitalWrite(GRAY_5_VCC, HIGH);

    pinMode(GRAY_6_VCC, OUTPUT);
    digitalWrite(GRAY_6_VCC, HIGH);

    pinMode(GRAY_7_VCC, OUTPUT);
    digitalWrite(GRAY_7_VCC, HIGH);

    pinMode(BUTTON_1_VCC, OUTPUT);
    digitalWrite(BUTTON_1_VCC, HIGH);

    pinMode(BUTTON_2_VCC, OUTPUT);
    digitalWrite(BUTTON_2_VCC, HIGH);

    //设置灰度值为默认值
    SetGrayGate(GRAY_1, DEFAULT_GRAY_1_GATE);
    SetGrayGate(GRAY_2, DEFAULT_GRAY_2_GATE);
    SetGrayGate(GRAY_3, DEFAULT_GRAY_3_GATE);
    SetGrayGate(GRAY_4, DEFAULT_GRAY_4_GATE);
    SetGrayGate(GRAY_5, DEFAULT_GRAY_5_GATE);
    SetGrayGate(GRAY_6, DEFAULT_GRAY_6_GATE);
    SetGrayGate(GRAY_7, DEFAULT_GRAY_7_GATE);
}


//设置灰度传感器临界值
void Sensor::SetGrayGate(uint8_t gray_sensor_num, int gate)
{
    switch (gray_sensor_num)
    {
    case GRAY_1: gray_1_gate = gate; break;
    case GRAY_2: gray_2_gate = gate; break;
    case GRAY_3: gray_3_gate = gate; break;
    case GRAY_4: gray_4_gate = gate; break;
    case GRAY_5: gray_5_gate = gate; break;
    case GRAY_6: gray_6_gate = gate; break;
    case GRAY_7: gray_7_gate = gate; 
    }
}


//使灰度传感器闪烁
void Sensor::FlashGraySensor(uint8_t gray_sensor_num)
{
    switch (gray_sensor_num)
    {
    case GRAY_1: digitalWrite(GRAY_1_VCC, LOW); break;
    case GRAY_2: digitalWrite(GRAY_2_VCC, LOW); break;
    case GRAY_3: digitalWrite(GRAY_3_VCC, LOW); break;
    case GRAY_4: digitalWrite(GRAY_4_VCC, LOW); break;
    case GRAY_5: digitalWrite(GRAY_5_VCC, LOW); break;
    case GRAY_6: digitalWrite(GRAY_6_VCC, LOW); break;
    case GRAY_7: digitalWrite(GRAY_7_VCC, LOW);
    }

    #ifdef SENSOR_DEBUG
    //调试输出闪烁信息
    switch (gray_sensor_num)
    {
    case GRAY_1: NeoSerial.println("#SENSOR: FLASH GRAY_1 NOW!"); break;
    case GRAY_2: NeoSerial.println("#SENSOR: FLASH GRAY_2 NOW!"); break;
    case GRAY_3: NeoSerial.println("#SENSOR: FLASH GRAY_3 NOW!"); break;
    case GRAY_4: NeoSerial.println("#SENSOR: FLASH GRAY_4 NOW!"); break;
    case GRAY_5: NeoSerial.println("#SENSOR: FLASH GRAY_5 NOW!"); break;
    case GRAY_6: NeoSerial.println("#SENSOR: FLASH GRAY_6 NOW!"); break;
    case GRAY_7: NeoSerial.println("#SENSOR: FLASH GRAY_7 NOW!");
    }
    #endif

    delay(GRAY_FLASH_TIME);
    Sensor();
    delay(GRAY_FLASH_TIME);
}


bool Sensor::IsWhite(uint8_t gray_sensor_num)
{
    uint8_t gray_out_pin;
    int gray_val;
    int gray_gate;
    
    switch (gray_sensor_num)
    {
    case GRAY_1: gray_out_pin = GRAY_1_OUT; gray_gate = gray_1_gate; break;
    case GRAY_2: gray_out_pin = GRAY_2_OUT; gray_gate = gray_2_gate; break;
    case GRAY_3: gray_out_pin = GRAY_3_OUT; gray_gate = gray_3_gate; break;
    case GRAY_4: gray_out_pin = GRAY_4_OUT; gray_gate = gray_4_gate; break;
    case GRAY_5: gray_out_pin = GRAY_5_OUT; gray_gate = gray_5_gate; break;
    case GRAY_6: gray_out_pin = GRAY_6_OUT; gray_gate = gray_6_gate; break;
    case GRAY_7: gray_out_pin = GRAY_7_OUT; gray_gate = gray_7_gate; 
    }

    gray_val = analogRead(gray_out_pin);

    #ifdef SENSOR_DEBUG
    //调试输出灰度值
    switch (gray_sensor_num)
    {
    case GRAY_1: NeoSerial.print("#SENSOR: GRAY_1 and gate_val: "); break;
    case GRAY_2: NeoSerial.print("#SENSOR: GRAY_2 and gate_val: "); break;
    case GRAY_3: NeoSerial.print("#SENSOR: GRAY_3 and gate_val: "); break;
    case GRAY_4: NeoSerial.print("#SENSOR: GRAY_4 and gate_val: "); break;
    case GRAY_5: NeoSerial.print("#SENSOR: GRAY_5 and gate_val: "); break;
    case GRAY_6: NeoSerial.print("#SENSOR: GRAY_6 and gate_val: "); break;
    case GRAY_7: NeoSerial.print("#SENSOR: GRAY_7 and gate_val: ");
    }

    NeoSerial.print(gray_val);
    NeoSerial.print(" ");
    NeoSerial.println(gray_gate);
    #endif

    if (gray_val > gray_gate)
        return true;
    else return false;
}


//灰度传感器灰度值偏离比例，即 (gray_gate - gray_val) / gray_gate
float Sensor::GrayDeviationRate(uint8_t gray_sensor_num)
{
    uint8_t gray_out_pin;
    int gray_val;
    int gray_gate;
    float deviarion_rate;
    
    switch (gray_sensor_num)
    {
    case GRAY_1: gray_out_pin = GRAY_1_OUT; gray_gate = gray_1_gate; break;
    case GRAY_2: gray_out_pin = GRAY_2_OUT; gray_gate = gray_2_gate; break;
    case GRAY_3: gray_out_pin = GRAY_3_OUT; gray_gate = gray_3_gate; break;
    case GRAY_4: gray_out_pin = GRAY_4_OUT; gray_gate = gray_4_gate; break;
    case GRAY_5: gray_out_pin = GRAY_5_OUT; gray_gate = gray_5_gate; break;
    case GRAY_6: gray_out_pin = GRAY_6_OUT; gray_gate = gray_6_gate; break;
    case GRAY_7: gray_out_pin = GRAY_7_OUT; gray_gate = gray_7_gate; 
    }

    gray_val = analogRead(gray_out_pin);

    #ifdef SENSOR_DEBUG
    //调试输出灰度值
    switch (gray_sensor_num)
    {
    case GRAY_1: NeoSerial.print("#SENSOR: GRAY_1 and gate_val: "); break;
    case GRAY_2: NeoSerial.print("#SENSOR: GRAY_2 and gate_val: "); break;
    case GRAY_3: NeoSerial.print("#SENSOR: GRAY_3 and gate_val: "); break;
    case GRAY_4: NeoSerial.print("#SENSOR: GRAY_4 and gate_val: "); break;
    case GRAY_5: NeoSerial.print("#SENSOR: GRAY_5 and gate_val: "); break;
    case GRAY_6: NeoSerial.print("#SENSOR: GRAY_6 and gate_val: "); break;
    case GRAY_7: NeoSerial.print("#SENSOR: GRAY_7 and gate_val: ");
    }

    NeoSerial.print(gray_val);
    NeoSerial.print(" ");
    NeoSerial.print(gray_gate);
    #endif

    deviarion_rate = (float)gray_val / gray_gate;

    #ifdef SENSOR_DEBUG
    NeoSerial.print(" deviarion_rate: ");
    NeoSerial.println(deviarion_rate);
    #endif

    return deviarion_rate;
}


//碰撞传感器（开关）判断是否闭合
bool Sensor::IsPushed(uint8_t button_num)
{
    uint8_t button_out_pin;
    int button_val;

    if (button_num == BUTTON_1)
        button_out_pin = BUTTON_1_OUT;
    else button_out_pin = BUTTON_2_OUT;

    button_val = digitalRead(button_out_pin);

    #ifdef SENSOR_DEBUG
    //调试输出按钮状态
    if (button_num == BUTTON_1)
        NeoSerial.print("#SENSOR: BUTTON_1: ");
    else NeoSerial.print("#SENSOR: BUTTON_2: ");

    if (button_val == LOW)
        NeoSerial.println("pushed");
    else NeoSerial.println("released");
    #endif    

    if (button_val == LOW)
        return true;
    else return false;
}


//开启 HMC5883 的 I2C 通讯
void Sensor::StartHMC5883(void)
{
    Wire.begin();
    Wire.beginTransmission(HMC5883_ADDRESS);
    Wire.write(0x02);
    Wire.write(0x00);
    Wire.endTransmission();

    #ifdef SENSOR_DEBUG
    //调试输出
    NeoSerial.println("#SENSOR: Start HMC5883");
    #endif
}


//返回朝向角
float Sensor::GetAngle(void)
{
    long x, y, z;

    Wire.beginTransmission(HMC5883_ADDRESS);
    Wire.write(0x03);
    Wire.endTransmission();

    Wire.requestFrom(HMC5883_ADDRESS, 6);
    if (6 <= Wire.available())
    {
        x = Wire.read() << 8;
        x |= Wire.read();
        z = Wire.read() << 8;
        z |= Wire.read();
        y = Wire.read() << 8;
        y |= Wire.read();
    }

    //计算朝向角
    float m = sqrt(x * x + y * y);
    float angle;

    if (x >= 0)
        angle = acos(y / m);
    else angle = 2.0 * PI - acos(y / m);
    
    #ifdef SENSOR_DEBUG
    //调试输出朝向角
    NeoSerial.print("#SENSOR: Angle Value: ");
    NeoSerial.println(angle);  
    #endif

    return angle;
}