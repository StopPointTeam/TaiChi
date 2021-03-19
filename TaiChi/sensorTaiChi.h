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
#define GRAY_7_OUT A6

//灰度传感器 VCC 接口定义
#define GRAY_1_VCC 48
#define GRAY_2_VCC 49
#define GRAY_3_VCC 50
#define GRAY_4_VCC 51
#define GRAY_5_VCC 52
#define GRAY_6_VCC 53
#define GRAY_7_VCC 47

//灰度传感器临界值
#define GRAY_1_GATE 900
#define GRAY_2_GATE 900
#define GRAY_3_GATE 850
#define GRAY_4_GATE 850
#define GRAY_5_GATE 900
#define GRAY_6_GATE 880
#define GRAY_7_GATE 690

//灰度传感器闪烁时间
#define GRAY_FLASH_TIME 200

//灰度传感器标识定义
#define GRAY_1 0
#define GRAY_2 1
#define GRAY_3 2
#define GRAY_4 3
#define GRAY_5 4
#define GRAY_6 5
#define GRAY_7 6

//碰撞传感器 OUT 接口定义
#define BUTTON_1_OUT 2
#define BUTTON_2_OUT 3

//碰撞传感器 VCC 接口定义
#define BUTTON_1_VCC 45
#define BUTTON_2_VCC 46

//碰撞传感器标识定义
#define BUTTON_1 0
#define BUTTON_2 1

//HMC5883 的 I2C 地址
#define HMC5883_ADDRESS 0x1E 


class Sensor
{
public:
    Sensor();

    //使灰度传感器闪烁
    void FlashGraySensor(uint8_t gray_sensor_num);

    //灰度传感器判断下方是否为白色
    bool IsWhite(uint8_t gray_sensor_num);

    //灰度传感器灰度值偏离比例，即 (gray_gate - gray_val) / gray_gate
    float GrayDeviationRate(uint8_t gray_sensor_num);

    //碰撞传感器（开关）判断是否闭合
    bool IsPushed(uint8_t button_num);

    //开启 HMC5883 的 I2C 通讯
    void StartHMC5883(void);

    //返回朝向角
    float GetAngle(void);
};  


#endif