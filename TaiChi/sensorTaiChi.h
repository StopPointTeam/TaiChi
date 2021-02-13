#ifndef SENSORTAICHI_H
#define SENSORTAICHI_H

#define DEBUG

//灰度传感器接口定义
#define GRAY_1_OUT A0
#define GRAY_2_OUT A1
#define GRAY_3_OUT A2
#define GRAY_4_OUT A3
#define GRAY_5_OUT A4
#define GRAY_6_OUT A5

//灰度传感器临界值
#define GRAY_GATE_VAL 800

//灰度传感器标识定义
#define GRAY_1 0
#define GRAY_2 1
#define GRAY_3 2
#define GRAY_4 3
#define GRAY_5 4
#define GRAY_6 5

//碰撞传感器接口定义
#define BUTTON_1_OUT 2
#define BUTTON_2_OUT 3

//碰撞传感器标识定义
#define BUTTON_1 0
#define BUTTON_2 1


class Sensor
{
public:
    Sensor();

    bool IsWhite(uint8_t gray_sensor_num);

    bool IsPushed(uint8_t button_num);
};


#endif