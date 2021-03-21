#ifndef SERVOTAICHI_H
#define SERVOTAICHI_H


#include <Arduino.h>


//注释以关闭调试功能
#define SERVO_DEBUG


//与舵机控制板连接串口
//使用 Mega 板 18 19 作为串口通信端口
#define SERVO_SERIAL_NUM NeoSerial1


//与舵机控制板串口通信波特率
#define SERVO_BAUD_RATE 9600


//爪子舵机 id
#define CLAW_SERVO_ID 15
//爪子打开时舵机位置
#define CLAW_OPEN_POSITION 500
//爪子打开用时
#define CLAW_OPEN_USE_TIME 1000


//发送部分的指令
#define FRAME_HEADER 0x55            //帧头
#define CMD_SERVO_MOVE 0x03          //舵机移动指令
#define CMD_ACTION_GROUP_RUN 0x06    //运行动作组指令
#define CMD_ACTION_GROUP_STOP 0x07   //停止动作组运行指令
#define CMD_ACTION_GROUP_SPEED 0x0B  //设置动作组运行速度指令


//动作组
#define ACTION_RESET_NUM 99
#define ACTION_DOWN_NUM 100
#define ACTION_CATCH_NUM 101
#define ACTION_RELEASE_NUM 102
#define ACTION_GAINDOWN_NUM 103
#define ACTION_GAINCATCH_NUM 104
#define ACTION_GAINUP_NUM 105


//默认动作组速度
#define SERVO_NORMAL_SPEED 1.0


class Servo
{
public:
    Servo();

    void BeginTransmit(unsigned long baud_rate = SERVO_BAUD_RATE); //打开串口

    void MoveServo(uint8_t servo_id, uint16_t position, uint16_t time); //控制单个舵机转动

    void RunActionGroup(uint8_t action_num, uint16_t times); //运行指定动作组
    void StopActionGroup(void); //停止动作组运行
    void SetActionGroupSpeed(uint8_t action_num, float speed); //设定指定动作组的运行速度
    void SetAllActionGroupSpeed(float speed); //设置所有动作组的运行速度

    void OpenClaw(void); //打开爪子

    void Reset(float speed = SERVO_NORMAL_SPEED); //恢复初始状态，指定速度
    void Down(float speed = SERVO_NORMAL_SPEED); //放下爪子，指定速度
    void Catch(float speed = SERVO_NORMAL_SPEED); //抓取，指定速度
    void Release(float speed = SERVO_NORMAL_SPEED); //释放，指定速度
    void GainDown(float speed = SERVO_NORMAL_SPEED); //增益点放下爪子，指定速度
    void GainCatch(float speed = SERVO_NORMAL_SPEED); //增益点抓取，指定速度
    void GainUp(float speed = SERVO_NORMAL_SPEED); //增益点抬升，指定速度
    void StopAndReset(float speed = SERVO_NORMAL_SPEED); //停止舵机并恢复初始状态，指定速度

private:
    NeoHWSerial* NeoSerialX;
};


#endif