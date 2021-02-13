#ifndef SERVOTAICHI_H
#define SERVOTAICHI_H


#include <Arduino.h>


#define BAUD_RATE 9600


//发送部分的指令
#define FRAME_HEADER 0x55            //帧头
#define CMD_ACTION_GROUP_RUN 0x06    //运行动作组指令
#define CMD_ACTION_GROUP_STOP 0x07   //停止动作组运行指令
#define CMD_ACTION_GROUP_SPEED 0x0B  //设置动作组运行速度指令


//动作组
#define ACTION_RESET_NUM 99
#define ACTION_CATCH_NUM 100
#define ACTION_RELEASE_NUM 101


//默认动作组速度
#define SERVO_NORMAL_SPEED 100


class Servo
{
public:
    Servo();
    Servo(HardwareSerial &serial_num);

    void RunActionGroup(uint8_t action_num, uint16_t times);
    void StopActionGroup(void);
    void SetActionGroupSpeed(uint8_t action_num, uint16_t speed);
    void SetAllActionGroupSpeed(uint16_t speed);

    void Reset(uint16_t speed = SERVO_NORMAL_SPEED); //恢复初始状态，指定速度
    void Catch(uint16_t speed = SERVO_NORMAL_SPEED); //抓取，指定速度
    void Release(uint16_t speed = SERVO_NORMAL_SPEED); //释放，指定速度
    void StopAndReset(uint16_t speed = SERVO_NORMAL_SPEED); //停止舵机并恢复初始状态，指定速度

private:
    HardwareSerial *SerialX;
};


#endif