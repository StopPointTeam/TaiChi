#include "servo.h"


#define GET_LOW_BYTE(A) (uint8_t)((A))
//宏函数 获得A的低八位
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)
//宏函数 获得A的高八位
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))
//宏函数 以A为高八位 B为低八位 合并为16位整形


Servo::Servo()
{
    SerialX = &Serial1; //默认使用 Mega 板 18 19 作为串口通信端口
    SerialX->begin(BAUD_RATE);
}


//指定串口通信端口
Servo::Servo(HardwareSerial& serial_num)
{
    SerialX = &serial_num;
    SerialX->begin(BAUD_RATE);
}


/*********************************************************************************
 * Function:  RunActionGroup
 * Description： 运行指定动作组
 * Parameters:   action_num:动作组序号, times:执行次数
 * Return:       无返回
 * Others:       times = 0 时无限循环
 *********************************************************************************/
void Servo::RunActionGroup(uint8_t action_num, uint16_t times)
{
    uint8_t buf[7];
    buf[0] = FRAME_HEADER; //填充帧头
    buf[1] = FRAME_HEADER;
    buf[2] = 5;                    //数据长度，数据帧除帧头部分数据字节数，此命令固定为5
    buf[3] = CMD_ACTION_GROUP_RUN; //填充运行动作组命令
    buf[4] = action_num;          //填充要运行的动作组号
    buf[5] = GET_LOW_BYTE(times);  //取得要运行次数的低八位
    buf[6] = GET_HIGH_BYTE(times); //取得要运行次数的高八位
    SerialX->write(buf, 7);        //发送数据帧
}


/*********************************************************************************
 * Function:  StopActionGroup
 * Description： 停止动作组运行
 * Parameters:   speed: 目标速度
 * Return:       无返回
 * Others:
 *********************************************************************************/
void Servo::StopActionGroup(void)
{
    uint8_t buf[4];
    buf[0] = FRAME_HEADER; //填充帧头
    buf[1] = FRAME_HEADER;
    buf[2] = 2;                     //数据长度，数据帧除帧头部分数据字节数，此命令固定为2
    buf[3] = CMD_ACTION_GROUP_STOP; //填充停止运行动作组命令

    SerialX->write(buf, 4); //发送数据帧
}


/*********************************************************************************
 * Function:  SetActionGroupSpeed
 * Description： 设定指定动作组的运行速度
 * Parameters:   action_num: 动作组序号 , speed:目标速度
 * Return:       无返回
 * Others:
 *********************************************************************************/
void Servo::SetActionGroupSpeed(uint8_t action_num, uint16_t speed)
{
    uint8_t buf[7];
    buf[0] = FRAME_HEADER; //填充帧头
    buf[1] = FRAME_HEADER;
    buf[2] = 5;                      //数据长度，数据帧除帧头部分数据字节数，此命令固定为5
    buf[3] = CMD_ACTION_GROUP_SPEED; //填充设置动作组速度命令
    buf[4] = action_num;            //填充要设置的动作组号
    buf[5] = GET_LOW_BYTE(speed);    //获得目标速度的低八位
    buf[6] = GET_HIGH_BYTE(speed);   //获得目标熟读的高八位

    SerialX->write(buf, 7); //发送数据帧
}


/*********************************************************************************
 * Function:  SetAllActionGroupSpeed
 * Description： 设置所有动作组的运行速度
 * Parameters:   speed: 目标速度
 * Return:       无返回
 * Others:
 *********************************************************************************/
void Servo::SetAllActionGroupSpeed(uint16_t speed)
{
    SetActionGroupSpeed(0xFF, speed); //调用动作组速度设定，组号为0xFF时设置所有组的速度
}


//恢复初始状态，指定速度
void Servo::Reset(uint16_t speed)
{
    SetActionGroupSpeed(ACTION_RESET_NUM, speed);
    RunActionGroup(ACTION_RESET_NUM, 1);
    SetActionGroupSpeed(ACTION_RESET_NUM, SERVO_NORMAL_SPEED);
}


//抓取，指定速度
void Servo::Catch(uint16_t speed)
{
    SetActionGroupSpeed(ACTION_CATCH_NUM, speed);
    RunActionGroup(ACTION_CATCH_NUM, 1);
    SetActionGroupSpeed(ACTION_CATCH_NUM, SERVO_NORMAL_SPEED);
}


//释放，指定速度
void Servo::Release(uint16_t speed)
{
    SetActionGroupSpeed(ACTION_RELEASE_NUM, speed);
    RunActionGroup(ACTION_RELEASE_NUM, 1);
    SetActionGroupSpeed(ACTION_RELEASE_NUM, SERVO_NORMAL_SPEED);
}


//停止舵机并恢复初始状态，指定速度
void Servo::StopAndReset(uint16_t speed)
{
    StopActionGroup();
    Reset(speed);
}