#include <Arduino.h>

#include "sensorTaiChi.h"


Sensor::Sensor()
{
    pinMode(BUTTON_1_OUT, INPUT);
    pinMode(BUTTON_2_OUT, INPUT);

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

    pinMode(BUTTON_1_VCC, OUTPUT);
    digitalWrite(BUTTON_1_VCC, HIGH);

    pinMode(BUTTON_2_VCC, OUTPUT);
    digitalWrite(BUTTON_2_VCC, HIGH);
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
    case GRAY_6: digitalWrite(GRAY_6_VCC, LOW);
    }

    #ifdef SENSOR_DEBUG
    //调试输出闪烁信息
    switch (gray_sensor_num)
    {
    case GRAY_1: Serial.println("#SENSOR: FLASH GRAY_1 NOW!"); break;
    case GRAY_2: Serial.println("#SENSOR: FLASH GRAY_2 NOW!"); break;
    case GRAY_3: Serial.println("#SENSOR: FLASH GRAY_3 NOW!"); break;
    case GRAY_4: Serial.println("#SENSOR: FLASH GRAY_4 NOW!"); break;
    case GRAY_5: Serial.println("#SENSOR: FLASH GRAY_5 NOW!"); break;
    case GRAY_6: Serial.println("#SENSOR: FLASH GRAY_6 NOW!");
    }
    #endif

    delay(GRAY_FLASH_TIME);
    Sensor();
    delay(GRAY_FLASH_TIME);
}


//灰度传感器判断下方是否为白色
bool Sensor::IsWhite(uint8_t gray_sensor_num)
{
    uint8_t gray_out_pin;
    int gray_val;
    
    switch (gray_sensor_num)
    {
    case GRAY_1: gray_out_pin = GRAY_1_OUT; break;
    case GRAY_2: gray_out_pin = GRAY_2_OUT; break;
    case GRAY_3: gray_out_pin = GRAY_3_OUT; break;
    case GRAY_4: gray_out_pin = GRAY_4_OUT; break;
    case GRAY_5: gray_out_pin = GRAY_5_OUT; break;
    case GRAY_6: gray_out_pin = GRAY_6_OUT;
    }

    gray_val = analogRead(gray_out_pin);

    #ifdef SENSOR_DEBUG
    //调试输出灰度值
    switch (gray_sensor_num)
    {
    case GRAY_1: Serial.print("#SENSOR: GRAY_1: "); break;
    case GRAY_2: Serial.print("#SENSOR: GRAY_2: "); break;
    case GRAY_3: Serial.print("#SENSOR: GRAY_3: "); break;
    case GRAY_4: Serial.print("#SENSOR: GRAY_4: "); break;
    case GRAY_5: Serial.print("#SENSOR: GRAY_5: "); break;
    case GRAY_6: Serial.print("#SENSOR: GRAY_6: ");
    }

    Serial.println(gray_val);
    #endif

    if (gray_val > GRAY_GATE_VAL)
        return true;
    else return false;
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
        Serial.print("#SENSOR: BUTTON_1: ");
    else Serial.print("#SENSOR: BUTTON_2: ");

    if (button_val == LOW)
        Serial.println("pushed");
    else Serial.println("released");
    #endif    

    if (button_val == LOW)
        return true;
    else return false;
}