#include <Arduino.h>
#include <EEPROM.h>
#include <NeoHWSerial.h>

#include "routeTaiChi.h" //路径库
#include "moveTaiChi.h" //轮胎运动库
#include "sensorTaiChi.h" //传感器库
#include "servoTaiChi.h" //舵机库
#include "radioTaiChi.h" //通信库


Route route; //路径实例
Move move; //轮胎运动实例
Sensor sensor; //传感器实例
Servo servo; //舵机实例
Radio radio; //通讯实例


//****************************************可调参数****************************************
//EEPROM 写入位置
#define EEPROM_ADDRESS 0x0

//是否关闭爪子状态检测功能
#define DISABLE_CLAW_CHECK

//抓取点移动用时
#define CATCH_MOVE_DELAY_TIME 500
//释放点向前移动用时
#define RELEASE_MOVE_FORWARD_DELAY_TIME 1200
//释放点暂停用时
#define STOP_DELAY_TIME 500
//释放点向后向前移动用时
#define RELEASE_MOVE_BACKWARD_AND_FORWARD_DELAY_TIME 480
//增益点向后向前移动用时
#define GAIN_MOVE_BACKWARD_AND_FORWARD_DELAY_TIME 750
//增益点稍稍向前移动推环用时
#define GAIN_MOVE_FORWARD_TO_PUSH_DELAY_TIME 200
//增益点稍稍向后移动抓取用时
#define GAIN_MOVE_BACKWARD_TO_CATCH_DELAY_TIME 100
//增益点稍稍向后移动拉环用时
#define GAIN_MOVE_BACKWARD_TO_UP_DELAY_TIME 65

//重置留时
#define RESET_DELAY_TIME 1600
//放下爪子用时
#define DOWN_DELAY_TIME 1600
//抓取留时
#define CATCH_DELAY_TIME 600
//释放留时
#define RELEASE_DELAY_TIME 3100
//增益放下爪子用时
#define GAINDOWN_DELAY_TIME 1600
//增益抓取用时
#define GAINCATCH_DELAY_TIME 1600
//增益抬起爪子用时
#define GAINUP_DELAY_TIME 1600

//最大抓取尝试次数
#define MAX_CATCH_TIMES 2

//（前进转）从触碰白线到开始转向延时
#define BEFORE_FORTURN_DELAY 670
//（后退转）从触碰白线到开始转向延时
#define BEFORE_BACKTURN_DELAY 500

//开始转到开始检测是否摆正的延时
#define BEFORE_CHECK_STRAIGHT_DELAY 700
//***************************************************************************************


//****************************************全局变量****************************************
//GND Pins
const uint8_t gnd_pins[8] = {12, 13, 32, 33, 34, 35, 36, 37};

//EEPROM 储存的调试数据
struct StroageInfo
{
    float real_angle_val[8];
    int gray_7_gate;
    int delay_time_after_turn;
} stroage_info;

//下一点绝对朝向角
float north_left_angle;
float north_right_angle;
float west_left_angle;
float west_right_angle;
float south_left_angle;
float south_right_angle;
float east_left_angle;
float east_right_angle;

//7 号传感器灰度临界值
int gray_7_gate;

//转向开始到结束的时间
int delay_time_after_turn;

//从 route 实例获取的点
Point passed_point;
Point next_point;
Point next_next_point;

#define FRONT_NEXT 0
#define BACK_NEXT 1
//下一点朝向
uint8_t next_position = FRONT_NEXT;

#define NORTH 0
#define WEST 1
#define SOUTH 2
#define EAST 3
//下一点绝对朝向
uint8_t current_real_direction = NORTH;

//爪子是否抓有环
bool is_claw_catch = false;

//爪子是否正常
bool is_claw_ok = true;

//底盘是否携带环
bool is_carry = false;

#define TURN_BY_TIME 0 //基于时间
#define TURN_BY_GRAY 1 //基于灰度传感器
#define TURN_BY_EARTH 2 //基于地磁场
//转向方式
uint8_t turn_method = TURN_BY_TIME;
//***************************************************************************************


//****************************************自定函数****************************************
//设置接口低电平作为额外地
void SetGNDPins(void);

//从 EEPROM 读取数据
void ReadFromEEPROM(void);

//在开始运行前依次检测各灰度传感器下方黑白是否正常，若不正常，异常传感器闪烁，程序不继续进行
void CheckGrayStatus(void);

//从 route 实例获取点
void GetPointFromRoute(void);

//计算方向，同时更改完成转向后相对下一点的朝向
uint8_t CalcDirection(void);

//由灰度比计算修正减速比
float CalcFixSpeedRate(float gray_deviation_rate);

#define FRONT_END 0
#define BACK_END 1
#define CATCH_END 2
#define RELEASE_END 3

#define ENABLE_FIX 0
#define DISABLE_FIX 1
//沿线直行，在触发条件后离开函数但不停止
void LineForward(uint8_t end_position, uint8_t type = ENABLE_FIX, float speed_rate = 1.0);

//沿线后退，在触发条件后离开函数但不停止
void LineBackward(uint8_t end_position, uint8_t type = ENABLE_FIX, float speed_rate = 1.0);

//直行或后退或转向，完成后离开函数但不停止。会自动跳过无需前往的释放点
void TurnDirection(float speed_rate = 1.0);

#define CATCH_TYPE_CATCH 0
#define CATCH_TYPE_GAIN 1
//抓取环，并判定是否抓取成功
bool CatchAndCheck(uint8_t type = CATCH_TYPE_CATCH, float speed = 1.0);

//打开爪子，并判断爪子是否能正常工作
bool OpenClawAndCheck(void);

//通讯消息处理函数
void HandleMessage(const char* message);
//***************************************************************************************


//****************************************调试相关****************************************
//注释以关闭调试功能
#define TAICHI_DEBUG

#ifdef TAICHI_DEBUG

#include "MemoryUsage.h"

#define NeoSerialDebug NeoSerial
#define DEBUG_BAUT_RATE 115200

int loop_time = 0;

//错误消息函数，用于在出现致命错误后结束程序
void DebugCanNotContinue(const char* message)
{
    move.Stop();
    
    NeoSerialDebug.print(F("#TAICHI: CAN NOT CONTINUE WHEN ")); NeoSerialDebug.println(message);
    NeoSerialDebug.print(F("#TAICHI: loop_time: ")); NeoSerialDebug.println(loop_time);
    NeoSerialDebug.print(F("#TAICHI: pass: [")); NeoSerialDebug.print(passed_point.x); NeoSerialDebug.print(F(", ")); NeoSerialDebug.print(passed_point.y); NeoSerialDebug.print(F("]"));
    NeoSerialDebug.print(F(" TYPE: ")); NeoSerialDebug.println((int)passed_point.type);
    NeoSerialDebug.print(F("#TAICHI: next: [")); NeoSerialDebug.print(next_point.x); NeoSerialDebug.print(F(", ")); NeoSerialDebug.print(next_point.y); NeoSerialDebug.print(F("]"));
    NeoSerialDebug.print(F(" next_position: ")); NeoSerialDebug.print((int)next_position);
    NeoSerialDebug.print(F(" TYPE: ")); NeoSerialDebug.println((int)next_point.type);
    NeoSerialDebug.print(F("#TAICHI: is_claw_catch: ")); NeoSerialDebug.print((int)is_claw_catch); NeoSerialDebug.print(F(" is_claw_ok: ")); NeoSerialDebug.println((int)is_claw_ok);
    SRamDisplay();

    while (1) {}
}
#endif
//***************************************************************************************


void setup()
{
    #ifdef TAICHI_DEBUG
    NeoSerialDebug.begin(DEBUG_BAUT_RATE);
    NeoSerialDebug.println(F("#TAICHI: ======================setup()====================="));
    SRamDisplay();
    #endif

    SetGNDPins();

    move.Stop();

    servo.BeginTransmit();
    servo.StopAndReset();

    radio.SetHandleMessageFunction(HandleMessage);
    radio.BeginTransmit();

    //从 EEPROM 读取数据
    ReadEEPROM();

    //开启 HMC5883 的 I2C 通讯
    sensor.StartHMC5883();

    //在开始运行前依次检测各灰度传感器下方黑白是否正常
    //CheckGrayStatus();

    while (1)
    {
        sensor.IsWhite(GRAY_1);
        sensor.IsWhite(GRAY_2);
        sensor.IsWhite(GRAY_3);
        sensor.IsWhite(GRAY_4);
        sensor.IsWhite(GRAY_5);
        sensor.IsWhite(GRAY_6);
    }

    //前往 0, 0
    //沿线直行，到后端传感器接触下一条线离开函数
    LineForward(BACK_END);

    //已越过 0, 0 正式进入循环
}


void loop()
{
    //从 route 实例获取点
    GetPointFromRoute();
    
    #ifdef TAICHI_DEBUG
    loop_time++;
    NeoSerialDebug.println(F("#TAICHI: ====================New loop()===================="));
    NeoSerialDebug.print(F("#TAICHI: loop_time: ")); NeoSerialDebug.println(loop_time);
    NeoSerialDebug.print(F("#TAICHI: pass: [")); NeoSerialDebug.print(passed_point.x); NeoSerialDebug.print(F(", ")); NeoSerialDebug.print(passed_point.y); NeoSerialDebug.print(F("]"));
    NeoSerialDebug.print(F(" TYPE: ")); NeoSerialDebug.println((int)passed_point.type);
    NeoSerialDebug.print(F("#TAICHI: next: [")); NeoSerialDebug.print(next_point.x); NeoSerialDebug.print(F(", ")); NeoSerialDebug.print(next_point.y); NeoSerialDebug.print(F("]"));
    NeoSerialDebug.print(F(" next_position: ")); NeoSerialDebug.print((int)next_position);
    NeoSerialDebug.print(F(" TYPE: ")); NeoSerialDebug.println((int)next_point.type);
    NeoSerialDebug.print(F("#TAICHI: is_claw_catch: ")); NeoSerialDebug.print((int)is_claw_catch); NeoSerialDebug.print(F(" is_claw_ok: ")); NeoSerialDebug.println((int)is_claw_ok);
    SRamDisplay();
    #endif

    //情况一：刚完整经过普通点，下一个点为普通点或携带点
    if (passed_point.type == NORMAL_POINT && (next_point.type == NORMAL_POINT || next_point.type == CARRY_POINT))   
    {
        if (next_position == FRONT_NEXT)
        {
            //沿线直行，到前端传感器接触下一条线离开函数
            LineForward(FRONT_END);

            //若下一点为携带点
            if (next_point.type == CARRY_POINT)
            {
                //底盘携带
                is_carry = true;

                //越过点成为普通点
                route.SetNextPointType(NORMAL_POINT);
            }
        }
        else
        {
            //沿线后退，到后端传感器接触下一条线离开函数
            LineBackward(BACK_END);
        }

        //继续直行或后退或转向
        TurnDirection();
    }
    //情况二：刚完整经过普通点，下一个点为抓取点
    else if (passed_point.type == NORMAL_POINT && next_point.type == CATCH_POINT)
    {
        if (!is_claw_catch && !is_claw_ok) //爪子上无环且状态不正常
        {  
            move.Stop(); //停止前进
            OpenClawAndCheck(); //再次检测爪子是否正常
        }

        if (!is_claw_catch && is_claw_ok) //爪子上无环且状态正常
        {
            move.Stop(); //停止前进

            servo.Down(); //放下爪子
            delay(DOWN_DELAY_TIME);
            
            //沿线直行，在抓取位置离开函数
            LineForward(CATCH_END);

            //停止前进
            move.Stop();

            //抓取
            if (!CatchAndCheck())
                is_carry = true; //抓取失败，视为底盘携带
        }
        else //未进行抓取
        {
            is_carry = true;
        }

        //继续沿线直行，到前端传感器接触下一条线离开函数
        LineForward(FRONT_END);

        //继续直行或后退或转向
        TurnDirection();

        //将越过的点视为普通点。因为即使抓取失败，环也会被携带在底盘内
        route.SetNextPointType(NORMAL_POINT);
    }
    //情况三：刚完整经过普通点，下一个点为释放点（使用机械臂）
    else if (passed_point.type == NORMAL_POINT && next_point.type == RELEASE_POINT)
    {
        //沿线直行，在释放柱离开函数
        LineForward(RELEASE_END);

        //停止前进
        move.Stop();
        delay(STOP_DELAY_TIME);

        //无循迹后退到真实释放位置
        move.Backward();
        delay(RELEASE_MOVE_BACKWARD_AND_FORWARD_DELAY_TIME);

        //停止后退
        move.Stop();

        //释放
        servo.Release();
        delay(RELEASE_DELAY_TIME); //释放留时
        is_claw_catch = false;

        //机械臂复原
        servo.Reset();

        //无循迹前进到释放柱
        move.Forward();
        delay(RELEASE_MOVE_BACKWARD_AND_FORWARD_DELAY_TIME);

        //下一点朝向为后
        next_position = BACK_NEXT;

        //停止前进
        move.Stop();
        delay(STOP_DELAY_TIME);
    }
    //情况四：刚完整经过释放点（使用机械臂）或增益抓取点，下一个点为普通点
    else if ((passed_point.type == RELEASE_POINT || passed_point.type == GAIN_POINT) && next_point.type == NORMAL_POINT)
    {
        //底盘携带清空
        is_carry = false;
        
        //沿线后退，到后端传感器接触下一条线离开函数
        LineBackward(BACK_END, DISABLE_FIX);

        //继续后退或转向
        TurnDirection();
    }
    //情况五：刚完整经过普通点，下一个点为释放点（从底盘）
    else if (passed_point.type == NORMAL_POINT && next_point.type == GETOUT_POINT)
    {
        //沿线直行，到前端传感器接触下一条线离开函数
        LineForward(FRONT_END);
        
        //沿线直行，到后端传感器接触下一条线离开函数
        LineForward(BACK_END);

        //停止前进
        move.Stop();

        //下一点朝向为后
        next_position = BACK_NEXT;
    }
    //情况六：刚完整经过释放点（从底盘），下一个点为普通点
    else if (passed_point.type == GETOUT_POINT && next_point.type == NORMAL_POINT)
    {
        //沿线后退，到前端传感器接触线离开函数
        LineBackward(FRONT_END, DISABLE_FIX);

        //底盘携带清空
        is_carry = false;
        
        //沿线后退，到后端传感器接触线离开函数
        LineBackward(BACK_END);

        //继续后退或转向
        TurnDirection();
    }
    //情况七：刚完整经过普通点，下一个点为增益抓取点
    else if (passed_point.type == NORMAL_POINT && next_point.type == GAIN_POINT)
    {
        if (!is_claw_ok) //爪子状态不正常
        {  
            move.Stop(); //停止前进
            OpenClawAndCheck(); //再次检测爪子是否正常
        }

        if (is_claw_ok) //爪子状态正常
        {
            //沿线直行，在释放柱（增益柱）离开函数
            LineForward(RELEASE_END);

            //停止前进
            move.Stop();
            delay(STOP_DELAY_TIME);

            //无循迹后退到真实抓取位置
            move.Backward();
            delay(GAIN_MOVE_BACKWARD_AND_FORWARD_DELAY_TIME);

            //停止后退
            move.Stop();

            //放下爪子
            servo.GainDown();
            delay(GAINDOWN_DELAY_TIME);

            //稍稍无循迹前进，向前推环，以约束环的位置
            move.Forward(0.5);
            delay(GAIN_MOVE_FORWARD_TO_PUSH_DELAY_TIME);

            //停止前进
            move.Stop();
            delay(STOP_DELAY_TIME);

            //稍稍无循迹后退，便于爪子抓取
            move.Backward(0.5);
            delay(GAIN_MOVE_BACKWARD_TO_CATCH_DELAY_TIME);

            //停止前进
            move.Stop();

            //抓取
            CatchAndCheck(CATCH_TYPE_GAIN);

            //稍稍无循迹后退，便于爪子抬起
            move.Backward(0.5);
            delay(GAIN_MOVE_BACKWARD_TO_UP_DELAY_TIME);

            //停止前进
            move.Stop();

            //抬起爪子
            servo.GainUp();
            delay(GAINUP_DELAY_TIME);

            //无循迹前进到释放柱（增益柱）
            move.Forward();
            delay(GAIN_MOVE_BACKWARD_AND_FORWARD_DELAY_TIME);
        }

        //下一点朝向为后
        next_position = BACK_NEXT;

        //停止前进
        move.Stop();
        delay(STOP_DELAY_TIME);
    }
    //出现错误
    else 
    {
        move.Stop();

        #ifdef TAICHI_DEBUG
        DebugCanNotContinue("CHOOSE LOOP");
        #endif
    }

    //更新位置，继续循环
    route.UpdatePosition();

    #ifdef TAICHI_DEBUG
    NeoSerialDebug.println(F("#TAICHI: ====================End loop()===================="));
    #endif
}


//设置接口低电平作为额外地
void SetGNDPins(void)
{
    uint8_t pin_num = sizeof(gnd_pins) / sizeof(uint8_t);

    for (uint8_t i = 0; i < pin_num; i++)
    {
        pinMode(gnd_pins[i], OUTPUT);
        digitalWrite(gnd_pins[i], LOW);
    }
}


//从 EEPROM 读取数据
void ReadEEPROM(void)
{
    //从 EEPROM 读取调试数据
    EEPROM.get(EEPROM_ADDRESS, stroage_info);

    //转向角度
    north_left_angle = stroage_info.real_angle_val[0];
    north_right_angle = stroage_info.real_angle_val[1];
    west_left_angle = stroage_info.real_angle_val[2];
    west_right_angle = stroage_info.real_angle_val[3];
    south_left_angle = stroage_info.real_angle_val[4];
    south_right_angle = stroage_info.real_angle_val[5];
    east_left_angle = stroage_info.real_angle_val[6];
    east_right_angle = stroage_info.real_angle_val[7];

    //转向灰度值
    sensor.SetGrayGate(GRAY_7, stroage_info.gray_7_gate);

    //转向时间
    delay_time_after_turn = stroage_info.delay_time_after_turn;

    #ifdef TAICHI_DEBUG
    NeoSerialDebug.println(F("#TAICHI: Data based on EEPROM: "));
    NeoSerialDebug.print(F("#TAICHI: north_left_angle: ")); NeoSerialDebug.println(north_left_angle);
    NeoSerialDebug.print(F("#TAICHI: north_right_angle: ")); NeoSerialDebug.println(north_right_angle);
    NeoSerialDebug.print(F("#TAICHI: west_left_angle: ")); NeoSerialDebug.println(west_left_angle);
    NeoSerialDebug.print(F("#TAICHI: west_right_angle: ")); NeoSerialDebug.println(west_right_angle);
    NeoSerialDebug.print(F("#TAICHI: south_left_angle: ")); NeoSerialDebug.println(south_left_angle);
    NeoSerialDebug.print(F("#TAICHI: south_right_angle: ")); NeoSerialDebug.println(south_right_angle);
    NeoSerialDebug.print(F("#TAICHI: east_left_angle: ")); NeoSerialDebug.println(east_left_angle);
    NeoSerialDebug.print(F("#TAICHI: east_right_angle: ")); NeoSerialDebug.println(east_right_angle);
    NeoSerialDebug.print(F("#TAICHI: gray_7_gate: ")); NeoSerialDebug.println(stroage_info.gray_7_gate);
    NeoSerialDebug.print(F("#TAICHI: delay_time_after_turn: ")); NeoSerialDebug.println(delay_time_after_turn);
    #endif
}


//在开始运行前依次检测各灰度传感器下方黑白是否正常，若不正常，异常传感器闪烁，程序不继续进行
void CheckGrayStatus(void)
{
    //若正常，1 2 5 6 号传感器检测到黑色，3 4 号传感器检测到白色
    //若一开始就采用灰度转弯，7 号也应检测到白色
    bool is_status_right = false;

    while (!is_status_right)
    {
        if (sensor.IsWhite(GRAY_1))
            sensor.FlashGraySensor(GRAY_1);
        else if (sensor.IsWhite(GRAY_2))
            sensor.FlashGraySensor(GRAY_2);
        else if (!sensor.IsWhite(GRAY_3))
            sensor.FlashGraySensor(GRAY_3);
        else if (!sensor.IsWhite(GRAY_4))
            sensor.FlashGraySensor(GRAY_4);
        else if (sensor.IsWhite(GRAY_5))
            sensor.FlashGraySensor(GRAY_5);
        else if (sensor.IsWhite(GRAY_6))
            sensor.FlashGraySensor(GRAY_6);
        else if (turn_method == TURN_BY_GRAY && !sensor.IsWhite(GRAY_7))
            sensor.FlashGraySensor(GRAY_7);
        else is_status_right = true;
    }

    #ifdef TAICHI_DEBUG
    NeoSerialDebug.println(F("#TAICHI: Gray Sensor Status OK!"));
    #endif
}


//从 route 实例获取点
void GetPointFromRoute(void)
{
    passed_point = route.GetPassedPoint();
    next_point = route.GetNextPoint();
    next_next_point = route.GetNextNextPoint();
}


//计算方向，同时更改完成转向后相对下一点的朝向
uint8_t CalcDirection(void)
{
    //计算第三点与第一点的相对坐标 rx0, ry0
    int8_t rx0 = next_next_point.x - passed_point.x;
    int8_t ry0 = next_next_point.y - passed_point.y;

    //计算当前小车朝向的方向向量 vx, vy
    int8_t vx = next_point.x - passed_point.x;
    int8_t vy = next_point.y - passed_point.y;

    //坐标旋转变换
    int8_t rx, ry;
    if (vx == 0 && vy == 1)
    {
        rx = rx0;
        ry = ry0;
    }
    else if (vx == -1 && vy == 0)
    {
        rx = ry0;
        ry = -rx0;
    }
    else if (vx == 0 && vy == -1)
    {
        rx = -rx0;
        ry = -ry0;
    }
    else if (vx == 1 && vy == 0)
    {
        rx = -ry0;
        ry = rx0;
    }
    #ifdef TAICHI_DEBUG
    else DebugCanNotContinue("CALC DIRECTION <1>"); //调试用
    #endif

    //判断行进方向
    if (rx == 0 && ry == 2)
    {
        if (next_position == FRONT_NEXT)
        {
            return FORWARD;
        }
        else
        {
            return BACKWARD;
        }
    }
    else if (rx == 0 && ry == 0)
    {
        if (next_position == FRONT_NEXT)
        {
            next_position = BACK_NEXT;
            return BACKWARD;
        }
        else
        {
            next_position = FRONT_NEXT;
            return FORWARD;
        }
    }
    else if (rx == -1 && ry == 1)
    {
        if (next_position == FRONT_NEXT)
        {
            return FORLEFTWARD;
        }
        else
        {
            next_position = FRONT_NEXT;
            return BACKLEFTWARD;
        }
    }
    else if (rx == 1 && ry == 1)
    {
        if (next_position == FRONT_NEXT)
        {
            return FORRIGHTWARD;
        }
        else
        {
            next_position = FRONT_NEXT;
            return BACKRIGHTWARD;
        }
    }
    #ifdef TAICHI_DEBUG
    else DebugCanNotContinue("CALC DIRECTION <2>"); //调试用
    #endif
}


//由灰度比计算修正减速比
float CalcFixSpeedRate(float gray_deviation_rate)
{
    return -50.0 * pow((gray_deviation_rate - 1.0), 2.0) + 1.0;
}


//沿线直行，在触发条件后离开函数但不停止
void LineForward(uint8_t end_position, uint8_t type, float speed_rate)
{
    #ifdef TAICHI_DEBUG
    //调试输出沿线直行状态
    NeoSerialDebug.print(F("#TAICHI: Line Forward"));
    NeoSerialDebug.print(F(" end_position: "));
    NeoSerialDebug.println((int)end_position);
    #endif

    move.Forward(speed_rate);

    //记录开始时间
    unsigned long begin_time = millis();
    //记录灰度传感器匹配情况
    bool gray_match_a = false;
    bool gray_match_b = false;

    while (1)
    {
        if (type == ENABLE_FIX)
        {
            if (!sensor.IsWhite(GRAY_3) && sensor.IsWhite(GRAY_4)) //左侧越线
            {
                move.ForRightward(speed_rate, CalcFixSpeedRate(sensor.GrayDeviationRate(GRAY_3)));
            }
            else if (sensor.IsWhite(GRAY_3) && !sensor.IsWhite(GRAY_4)) //右侧越线
            {
                move.ForLeftward(speed_rate, CalcFixSpeedRate(sensor.GrayDeviationRate(GRAY_4)));
            }
            else if (sensor.IsWhite(GRAY_3) && sensor.IsWhite(GRAY_4)) //在白线上
            {
                move.Forward(speed_rate);
            }
            else //均不符合
            {
                //move.Backward(LINE_FIND_SPEED_RATE);
            }
        }

        if (end_position == FRONT_END) //前端接触线离开函数
        {
            if (sensor.IsWhite(GRAY_1))
                gray_match_a = true;
            
            if (sensor.IsWhite(GRAY_2))
                gray_match_b = true;
        }
        else if (end_position == BACK_END) //后端接触线离开函数
        {
            if (sensor.IsWhite(GRAY_5))
                gray_match_a = true;
            
            if (sensor.IsWhite(GRAY_6))
                gray_match_b = true;
        }
        else if (end_position == CATCH_END) //到达抓取位置离开函数
        {
            if (millis() - begin_time > CATCH_MOVE_DELAY_TIME)
                break;
        }
        else //到达释放位置离开函数
        {
            if (millis() - begin_time > RELEASE_MOVE_FORWARD_DELAY_TIME)
                break;
        }

        //对应前端接触线离开函数和后端接触线离开函数的情况
        if (gray_match_a && gray_match_b)
            break;
    }

    #ifdef TAICHI_DEBUG
    //调试输出沿线直行结束
    NeoSerialDebug.println(F("#TAICHI: End Line Forward"));
    #endif
}


//沿线后退，在触发条件后离开函数但不停止
void LineBackward(uint8_t end_position, uint8_t type, float speed_rate)
{
    #ifdef TAICHI_DEBUG
    //调试输出沿线后退状态
    NeoSerialDebug.print(F("#TAICHI: Line Backward"));
    NeoSerialDebug.print(F(" end_position: "));
    NeoSerialDebug.println((int)end_position);
    #endif

    //记录灰度传感器匹配情况
    bool gray_match_a = false;
    bool gray_match_b = false;

    move.Backward(speed_rate);

    while (1)
    {
        if (type == ENABLE_FIX)
        {
            if (!sensor.IsWhite(GRAY_3) && sensor.IsWhite(GRAY_4)) //左侧越线
            {
                move.BackRightward(speed_rate, CalcFixSpeedRate(sensor.GrayDeviationRate(GRAY_3)));
            }
            else if (sensor.IsWhite(GRAY_3) && !sensor.IsWhite(GRAY_4)) //右侧越线
            {
                move.BackLeftward(speed_rate, CalcFixSpeedRate(sensor.GrayDeviationRate(GRAY_4)));
            }
            else if (sensor.IsWhite(GRAY_3) && sensor.IsWhite(GRAY_4)) //在白线上
            {
                move.Backward(speed_rate);
            }
            else //均不符合
            {
                //move.Forward(LINE_FIND_SPEED_RATE);
            }
        }

        if (end_position == FRONT_END) //前端接触线离开函数
        {
            if (sensor.IsWhite(GRAY_1))
                gray_match_a = true;
            
            if (sensor.IsWhite(GRAY_2)) 
                gray_match_b = true;
        }
        else //后端接触线离开函数
        {
            if (sensor.IsWhite(GRAY_5))
                gray_match_a = true;
            
            if (sensor.IsWhite(GRAY_6))
                gray_match_b = true;
        }

        //对应前端接触线离开函数和后端接触线离开函数的情况
        if (gray_match_a && gray_match_b)
            break;
    }

    #ifdef TAICHI_DEBUG
    //调试输出沿线后退结束
    NeoSerialDebug.println(F("#TAICHI: End Line Backward"));
    #endif
}


//直行或后退或转向，完成后离开函数但不停止。会自动跳过无需前往的释放点
void TurnDirection(float speed_rate)
{    
    //若下下点为释放点或增益抓取点，判断是否需要跳过
    if ((next_next_point.type == RELEASE_POINT && (!is_claw_catch || is_carry)) 
        || (next_next_point.type == GETOUT_POINT && !is_carry)
        || (next_next_point.type == GAIN_POINT && is_claw_catch)
        )
    {
        route.ChangeRoute(JUMP_DEAD_ROAD);
        GetPointFromRoute();

        #ifdef TAICHI_DEBUG
        //调试输出直行或后退或转向状态
        NeoSerialDebug.println(F("#TAICHI: JUMP THE RELEASE/GETOUT/GAIN POINT FOR STATUS REASON"));
        #endif
    }
    
    uint8_t direction = CalcDirection();
    
    #ifdef TAICHI_DEBUG
    //调试输出直行或后退或转向状态
    NeoSerialDebug.print(F("#TAICHI: Turn Direction"));
    NeoSerialDebug.print(F(" direction: "));
    NeoSerialDebug.println((int)direction);
    #endif
    
    if (direction == FORWARD) //继续直行
    {
        //沿线直行，到后端传感器接触线离开函数
        LineForward(BACK_END, speed_rate);
    }
    else if (direction == BACKWARD) //继续后退
    {
        //沿线后退，到前端传感器接触线离开函数
        LineBackward(FRONT_END, speed_rate);
    }
    else //继续转向
    {
        //等待小车旋转中心与十字中心重合
        if (direction == FORLEFTWARD || direction == FORRIGHTWARD)
            delay(BEFORE_FORTURN_DELAY);
        else delay(BEFORE_BACKTURN_DELAY);

        //旋转
        move.MoveDirection(direction, speed_rate);

        //角度相关计算
        float next_real_angle;
        uint8_t next_real_direction;
        if (direction == FORLEFTWARD || direction == BACKRIGHTWARD)
        {
            switch (current_real_direction)
            {
            case NORTH: next_real_direction = WEST; next_real_angle = west_left_angle; break;
            case WEST: next_real_direction = SOUTH; next_real_angle = south_left_angle; break;
            case SOUTH: next_real_direction = EAST; next_real_angle = east_left_angle; break;
            case EAST: next_real_direction = NORTH; next_real_angle = north_left_angle;
            }
        }
        else
        {
            switch (current_real_direction)
            {
            case NORTH: next_real_direction = EAST; next_real_angle = east_right_angle; break;
            case WEST: next_real_direction = NORTH; next_real_angle = north_right_angle; break;
            case SOUTH: next_real_direction = WEST; next_real_angle = west_right_angle; break;
            case EAST: next_real_direction = SOUTH; next_real_angle = south_right_angle;
            }
        }

        //以不同方式判定结束
        switch (turn_method)
        {
        case TURN_BY_TIME: //基于时间
        {
            delay(delay_time_after_turn);
        } break;

        case TURN_BY_GRAY: //基于灰度传感器
        {
            //等待一定时间后检测是否摆正
            delay(BEFORE_CHECK_STRAIGHT_DELAY);
            while (!sensor.IsWhite(GRAY_7)) {}
        } break;

        case TURN_BY_EARTH: //基于地磁场
        {
            //等待一定时间后检测是否摆正
            delay(BEFORE_CHECK_STRAIGHT_DELAY);
            
            if (direction == FORLEFTWARD || direction == BACKRIGHTWARD)
                while (sensor.GetAngle() < next_real_angle) {}
            else while (sensor.GetAngle() > next_real_angle) {}
        }
        }

        current_real_direction = next_real_direction;
    }

    #ifdef TAICHI_DEBUG
    //调试输出直行或后退或转向结束
    NeoSerialDebug.println(F("#TAICHI: End Turn Direction"));
    #endif
}


//抓取环，并判定是否抓取成功
bool CatchAndCheck(uint8_t type, float speed)
{
    //记录开始时间
    unsigned long begin_time = millis();

    //抓取延时
    int catch_delay_time;

    //抓取次数
    uint8_t catch_times = 0;

    if (type == CATCH_TYPE_CATCH)
        catch_delay_time = CATCH_DELAY_TIME;
    else catch_delay_time = GAINCATCH_DELAY_TIME;
    
    //执行抓取动作
    if (type == CATCH_TYPE_CATCH)
        servo.Catch(speed);
    else servo.GainCatch(speed);
    catch_times++;

    //等待完成动作
    while (millis() - begin_time < catch_delay_time)
    {
        #ifndef DISABLE_CLAW_CHECK
        if (sensor.IsPushed(BUTTON_2)) //开关 2 闭合，即爪子两端接触，说明抓取失败
        {
            #ifdef TAICHI_DEBUG
            //调试输出失败信息
            NeoSerialDebug.print(F("#TAICHI: **********FAIL CATCH!**********"));
            NeoSerialDebug.print(F(" catch_times: ")); NeoSerialDebug.println((int)catch_times);
            #endif

            if (type == CATCH_TYPE_GAIN)
                delay(millis() - begin_time - catch_delay_time); //等待运行完成，防止卡住

            //停止动作组运行
            servo.StopActionGroup();

            if (catch_times == MAX_CATCH_TIMES) //达到最大尝试次数，返回
            {
                servo.Reset();
                return false;
            }

            //打开爪子
            if(!OpenClawAndCheck()) //未能打开爪子
                return false;

            //更新时间
            begin_time = millis();

            //执行抓取动作
            if (type == CATCH_TYPE_CATCH)
                servo.Catch(speed);
            else servo.GainCatch(speed);
            catch_times++;
        }
        #endif
    }

    #ifndef DISABLE_CLAW_CHECK
    #ifdef TAICHI_DEBUG
    //调试输出成功信息
    NeoSerialDebug.println(F("#TAICHI: SUCCESS CATCH!"));
    #endif
    #endif

    #ifdef DISABLE_CLAW_CHECK
    #ifdef TAICHI_DEBUG
    //调试输出结束信息
    NeoSerialDebug.println(F("#TAICHI: CATCH WITHOUT CHECK!"));
    #endif
    #endif

    is_claw_catch = true;
    return true;
}


//打开爪子，并判断爪子是否能正常工作
bool OpenClawAndCheck(void)
{
    #ifdef DISABLE_CLAW_CHECK
    #ifdef TAICHI_DEBUG
    //调试输出信息
    NeoSerialDebug.println(F("#TAICHI: DISABLE CLAW CHECK!"));
    #endif

    is_claw_ok = true;
    return true;
    #endif
    
    //记录开始时间
    unsigned long begin_time = millis();

    //打开爪子
    servo.OpenClaw();

    //等待完成动作
    while (millis() - begin_time < CLAW_OPEN_USE_TIME)
    {
        if (!sensor.IsPushed(BUTTON_2)) //开关 2 打开，即爪子两端脱离接触，说明打开爪子成功
        {
            #ifdef TAICHI_DEBUG
            //调试输出成功信息
            NeoSerialDebug.println(F("#TAICHI: SUCCESS OPEN CLAW!"));
            #endif
            
            is_claw_ok = true;
            return true;
        }
    }

    #ifdef TAICHI_DEBUG
    //调试输出失败信息
    NeoSerialDebug.println(F("#TAICHI: $$$$$$$$$$FAIL CLAW!$$$$$$$$$$"));
    #endif
    
    is_claw_ok = false;
    return false;
}


//通讯消息处理函数
void HandleMessage(const char* message)
{
    radio.Send("Get the message: ");
    radio.Send(message);
}