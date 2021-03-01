#ifndef SENSORTAICHI_H
#define SENSORTAICHI_H


//注释以关闭调试功能
#define SENSOR_DEBUG


//灰度传感器 OUT 接口定义
#define GRAY_1_OUT A0
#define GRAY_2_OUT A1
#define GRAY_3_OUT A2
#define GRAY_4_OUT A3
#define GRAY_5_OUT A4
#define GRAY_6_OUT A5

//灰度传感器 VCC 接口定义
#define GRAY_1_VCC 48
#define GRAY_2_VCC 49
#define GRAY_3_VCC 50
#define GRAY_4_VCC 51
#define GRAY_5_VCC 52
#define GRAY_6_VCC 53

//灰度传感器临界值
#define GRAY_GATE_VAL 700

//灰度传感器闪烁时间
#define GRAY_FLASH_TIME 200

//灰度传感器标识定义
#define GRAY_1 0
#define GRAY_2 1
#define GRAY_3 2
#define GRAY_4 3
#define GRAY_5 4
#define GRAY_6 5

//碰撞传感器 OUT 接口定义
#define BUTTON_1_OUT 2
#define BUTTON_2_OUT 3

//碰撞传感器 VCC 接口定义
#define BUTTON_1_VCC 46
#define BUTTON_2_VCC 47

//碰撞传感器标识定义
#define BUTTON_1 0
#define BUTTON_2 1


class Sensor
{
public:
    Sensor();

    //使灰度传感器闪烁
    void FlashGraySensor(uint8_t gray_sensor_num);

    //灰度传感器判断下方是否为白色
    bool IsWhite(uint8_t gray_sensor_num);

    //碰撞传感器（开关）判断是否闭合
    bool IsPushed(uint8_t button_num);
};


#endif