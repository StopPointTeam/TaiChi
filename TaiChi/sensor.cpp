#include <Arduino.h>

#include "sensor.h"


Sensor::Sensor()
{
    pinMode(BUTTON_1_OUT, INPUT);
    pinMode(BUTTON_2_OUT, INPUT);
}


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

    #ifdef DEBUG
    //调试输出灰度值
    switch (gray_sensor_num)
    {
    case GRAY_1: Serial.print("GRAY_1: "); break;
    case GRAY_2: Serial.print("GRAY_2: "); break;
    case GRAY_3: Serial.print("GRAY_3: "); break;
    case GRAY_4: Serial.print("GRAY_4: "); break;
    case GRAY_5: Serial.print("GRAY_5: "); break;
    case GRAY_6: Serial.print("GRAY_6: ");
    }

    Serial.println(gray_val);
    #endif

    if (gray_val > GRAY_GATE_VAL)
        return true;
    else return false;
}


bool Sensor::IsPushed(uint8_t button_num)
{
    uint8_t button_out_pin;
    int button_val;

    if (button_num == BUTTON_1)
        button_out_pin = BUTTON_1_OUT;
    else button_out_pin = BUTTON_2_OUT;

    button_val = digitalRead(button_out_pin);

    #ifdef DEBUG
    //调试输出按钮状态
    if (button_num == BUTTON_1)
        Serial.print("BUTTON_1: ");
    else Serial.print("BUTTON_2: ");

    if (button_val == LOW)
        Serial.print("pushed\n");
    else Serial.print("released\n");
    #endif    

    if (button_val == LOW)
        return true;
    else return false;
}