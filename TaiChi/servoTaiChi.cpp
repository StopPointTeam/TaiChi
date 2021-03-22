#include <NeoHWSerial.h>

#include "servoTaiChi.h"


//宏函数 获得A的低八位
#define GET_LOW_BYTE(A) (uint8_t)((A))
//宏函数 获得A的高八位
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)
//宏函数 以A为高八位 B为低八位 合并为16位整形
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))


//静态变量
NeoHWSerial* Servo::NeoSerialX = &SERVO_SERIAL_NUM;


Servo::Servo() {}


//打开串口
void Servo::BeginTransmit(unsigned long baud_rate)
{
    NeoSerialX->begin(baud_rate);
}


//控制单个舵机转动
//servo_id: 舵机ID, position: 目标位置, time: 转动时间
void Servo::MoveServo(uint8_t servo_id, uint16_t position, uint16_t time)
{
	uint8_t buf[11];
	buf[0] = FRAME_HEADER;                   //填充帧头
	buf[1] = FRAME_HEADER;
	buf[2] = 8;                              //数据长度=要控制舵机数*3+5，此处=1*3+5
	buf[3] = CMD_SERVO_MOVE;                 //填充舵机移动指令
	buf[4] = 1;                              //要控制的舵机个数
	buf[5] = GET_LOW_BYTE(time);             //填充时间的低八位
	buf[6] = GET_HIGH_BYTE(time);            //填充时间的高八位
	buf[7] = servo_id;                        //舵机ID
	buf[8] = GET_LOW_BYTE(position);         //填充目标位置的低八位
	buf[9] = GET_HIGH_BYTE(position);        //填充目标位置的高八位

	NeoSerialX->write(buf, 10);

    #ifdef SERVO_DEBUG
    //调试输出动作组执行信息
    NeoSerial.print("#SERVO:  MoveServo: "); NeoSerial.print((int)servo_id); 
    NeoSerial.print(" position: "); NeoSerial.print((int)position); 
    NeoSerial.print(" time: "); NeoSerial.println((int)time);
    #endif
}


//运行指定动作组
//action_num: 动作组序号, times: 执行次数
//times = 0 时无限循环
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

    NeoSerialX->write(buf, 7);        //发送数据帧

    #ifdef SERVO_DEBUG
    //调试输出动作组执行信息
    NeoSerial.print("#SERVO:  RunServoActionGroup: ");
    NeoSerial.println((int)action_num);
    #endif
}


//停止动作组运行
void Servo::StopActionGroup(void)
{
    uint8_t buf[4];
    buf[0] = FRAME_HEADER; //填充帧头
    buf[1] = FRAME_HEADER;
    buf[2] = 2;                     //数据长度，数据帧除帧头部分数据字节数，此命令固定为2
    buf[3] = CMD_ACTION_GROUP_STOP; //填充停止运行动作组命令

    NeoSerialX->write(buf, 4); //发送数据帧

    #ifdef SERVO_DEBUG
    //调试输出动作组停止信息
    NeoSerial.println("#SERVO:  StopServoActionGroup");
    #endif
}


//设定指定动作组的运行速度
//action_num: 动作组序号, speed: 目标速度
void Servo::SetActionGroupSpeed(uint8_t action_num, float speed)
{
    uint16_t speed_int = (uint16_t)(speed * 100.0);
    uint8_t buf[7];
    buf[0] = FRAME_HEADER; //填充帧头
    buf[1] = FRAME_HEADER;
    buf[2] = 5;                      //数据长度，数据帧除帧头部分数据字节数，此命令固定为5
    buf[3] = CMD_ACTION_GROUP_SPEED; //填充设置动作组速度命令
    buf[4] = action_num;            //填充要设置的动作组号
    buf[5] = GET_LOW_BYTE(speed_int);    //获得目标速度的低八位
    buf[6] = GET_HIGH_BYTE(speed_int);   //获得目标熟读的高八位

    NeoSerialX->write(buf, 7); //发送数据帧

    #ifdef SERVO_DEBUG
    //调试输出动作组速度设定信息
    NeoSerial.print("#SERVO:  SetServoActionGroupSpeed: ");
    NeoSerial.print((int)action_num);
    NeoSerial.print(" Speed: ");
    NeoSerial.println(speed);
    #endif
}


//设置所有动作组的运行速度
//speed: 目标速度
void Servo::SetAllActionGroupSpeed(float speed)
{
    SetActionGroupSpeed(0xFF, speed); //调用动作组速度设定，组号为0xFF时设置所有组的速度
}


//打开爪子
void Servo::OpenClaw(void) 
{
    MoveServo(CLAW_SERVO_ID, CLAW_OPEN_POSITION, CLAW_OPEN_USE_TIME);
}


//恢复初始状态，指定速度
void Servo::Reset(float speed)
{
    SetActionGroupSpeed(ACTION_RESET_NUM, speed);
    RunActionGroup(ACTION_RESET_NUM, 1);
}


//放下爪子，指定速度
void Servo::Down(float speed)
{
    SetActionGroupSpeed(ACTION_DOWN_NUM, speed);
    RunActionGroup(ACTION_DOWN_NUM, 1);
}


//抓取，指定速度
void Servo::Catch(float speed)
{
    SetActionGroupSpeed(ACTION_CATCH_NUM, speed);
    RunActionGroup(ACTION_CATCH_NUM, 1);
}


//释放，指定速度
void Servo::Release(float speed)
{
    SetActionGroupSpeed(ACTION_RELEASE_NUM, speed);
    RunActionGroup(ACTION_RELEASE_NUM, 1);
}


//增益点放下爪子，指定速度
void Servo::GainDown(float speed = SERVO_NORMAL_SPEED)
{
    SetActionGroupSpeed(ACTION_GAINDOWN_NUM, speed);
    RunActionGroup(ACTION_GAINDOWN_NUM, 1);
}


//增益点抓取，指定速度
void Servo::GainCatch(float speed = SERVO_NORMAL_SPEED)
{
    SetActionGroupSpeed(ACTION_GAINCATCH_NUM, speed);
    RunActionGroup(ACTION_GAINCATCH_NUM, 1);
}


//增益点抬升，指定速度
void Servo::GainUp(float speed)
{
    SetActionGroupSpeed(ACTION_GAINUP_NUM, speed);
    RunActionGroup(ACTION_GAINUP_NUM, 1);
}


//停止舵机并恢复初始状态，指定速度
void Servo::StopAndReset(float speed)
{
    StopActionGroup();
    Reset(speed);
}