#include "moveTaiChi.h" //轮胎运动库
#include "sensorTaiChi.h" //传感器库
#include "servoTaiChi.h" //舵机库


Move move; //轮胎运动实例
Sensor sensor; //传感器实例
Servo servo; //舵机实例


//****************************************运动路径****************************************
//坐标点操作定义
#define NORMAL_POINT 0 //普通点
#define CATCH_POINT 1 //抓取点
#define RELEASE_POINT 2 //释放点（使用机械臂）
#define CARRY_POINT 3 //携带点（从底盘）
#define GETOUT_POINT 4 //释放点（从底盘）
#define GAIN_POINT 5 //增益点

//坐标点数组定义
#define X 0
#define Y 1
#define TYPE 2

int8_t route[][3] = 
{
    {0, 0, NORMAL_POINT},
    {0, 1, NORMAL_POINT},
    {0, 2, NORMAL_POINT},
    {-1, 2, NORMAL_POINT},
    {-1, 1, CATCH_POINT},
    {0, 1, NORMAL_POINT},
    {0, 0, NORMAL_POINT},
    {-1, 0, RELEASE_POINT}
};
//***************************************************************************************


//****************************************可调参数****************************************
//是否关闭爪子状态检测功能
#define DISABLE_CLAW_CHECK

//抓取点移动用时
#define CATCH_MOVE_DELAY_TIME 500
//释放点向前移动用时
#define RELEASE_MOVE_FORWARD_DELAY_TIME 1000
//释放点暂停用时
#define RELEASE_STOP_DELAY_TIME 500
//释放点向后向前移动用时
#define RELEASE_MOVE_BACKWARD_AND_FORWARD_DELAY_TIME 460
//增益点向后向前移动用时
#define GAIN_MOVE_BACKWARD_AND_FORWARD_DELAY_TIME 460
//增益点稍稍向前移动用时
#define GAIN_MOVE_LITTLEFORWARD_DELAY_TIME 200

//重置留时
#define RESET_DELAY_TIME 1600
//放下爪子用时
#define DOWN_DELAY_TIME 1600
//抓取留时
#define CATCH_DELAY_TIME 2100
//释放留时
#define RELEASE_DELAY_TIME 3100
//增益放下爪子用时
#define GAINDOWN_DELAY_TIME 1600
//增益抓取用时
#define GAINCATCH_DELAY_TIME 3100

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
uint8_t gnd_pins[4] = {14, 15, 16, 17};

//数组位置标记
int passed_flag = 0;
int next_flag = 1;
int next_next_flag = 2;

//最大数组位置标记
int max_flag;

#define FRONT_NEXT 0
#define BACK_NEXT 1
//下一点朝向
uint8_t next_position = FRONT_NEXT;

//爪子是否抓有环
bool is_claw_catch = false;

//爪子是否正常
bool is_claw_ok = true;

//底盘是否携带环
bool is_carry = false;
//***************************************************************************************


//****************************************自定函数****************************************
//设置接口低电平作为额外地
void SetGNDPins(void);

//在开始运行前依次检测各灰度传感器下方黑白是否正常，若不正常，异常传感器闪烁，程序不继续进行
void CheckGrayStatus(void);

#define NO_JUMP 0
#define JUMP_DEAD_ROAD 1
//更新标记
void UpdateFlag(uint8_t jump_choice = NO_JUMP);

//计算方向，同时更改完成转向后相对下一点的朝向
uint8_t CalcDirection(void);

//由灰度比计算修正减速比
float CalcFixSpeedRate(float gray_deviation_rate);

#define FRONT_END 0
#define BACK_END 1
#define CATCH_END 2
#define RELEASE_END 3
//沿线直行，在触发条件后离开函数但不停止
void LineForward(uint8_t end_position, float speed_rate = 1.0);

//沿线后退，在触发条件后离开函数但不停止
void LineBackward(uint8_t end_position, float speed_rate = 1.0);

//直行或后退或转向，完成后离开函数但不停止。会自动跳过无需前往的释放点
void TurnDirection(float speed_rate = 1.0);

#define CATCH_TYPE_CATCH 0
#define CATCH_TYPE_GAIN 1
//抓取环，并判定是否抓取成功
bool CatchAndCheck(uint8_t type = CATCH_TYPE_CATCH, float speed = 1.0);

//打开爪子，并判断爪子是否能正常工作
bool OpenClawAndCheck(void);
//***************************************************************************************


//****************************************调试相关****************************************
//注释以关闭调试功能
#define TAICHI_DEBUG

#ifdef TAICHI_DEBUG

#define DEBUG_BAUT_RATE 115200
#define DEBUG_PAUSE_INTERRUPTNUM 2 //PIN 21
#define DEBUG_PAUSE_PIN 21

int loop_time = 0;

//中断函数，用于调试时暂停程序
void DebugPause(void)
{
    //记录之前的运动状态
    uint8_t current_direction = move.GetCurrentMove(); //当前运动状态
    float current_speed_rate = move.GetCurrentSpeedRate(); //当前运动速度比率
    float current_turn_speed_rate = move.GetCurrentTurnSpeedRate(); //当前转向时一侧减速的比率

    //停止运动
    move.Stop();
    
    //等待低电平结束
    while (digitalRead(DEBUG_PAUSE_PIN) == LOW) {}

    //恢复之前的运动
    move.MoveDirection(current_direction, current_speed_rate, current_turn_speed_rate);
}

//错误消息函数，用于在出现致命错误后结束程序
void DebugCanNotContinue(char* message)
{
    Serial.print("#TAICHI: CAN NOT CONTINUE WHEN "); Serial.println(message);
    Serial.print("#TAICHI: loop_time: "); Serial.println(loop_time);
    Serial.print("#TAICHI: pass: ["); Serial.print(route[passed_flag][X]); Serial.print(", "); Serial.print(route[passed_flag][Y]); Serial.print("]");
    Serial.print(" flag: "); Serial.print(passed_flag);
    Serial.print(" TYPE: "); Serial.println((int)route[passed_flag][TYPE]);
    Serial.print("#TAICHI: next: ["); Serial.print(route[next_flag][X]); Serial.print(", "); Serial.print(route[next_flag][Y]); Serial.print("]");
    Serial.print(" next_position: "); Serial.print((int)next_position);
    Serial.print(" TYPE: "); Serial.println((int)route[next_flag][TYPE]);
    Serial.print("#TAICHI: is_claw_catch: "); Serial.print((int)is_claw_catch); Serial.print(" is_claw_ok: "); Serial.println((int)is_claw_ok);

    while (1) {}
}
#endif
//***************************************************************************************


void setup()
{
    #ifdef TAICHI_DEBUG
    pinMode(DEBUG_PAUSE_PIN, INPUT_PULLUP);
    attachInterrupt(DEBUG_PAUSE_INTERRUPTNUM, DebugPause, LOW);
    Serial.begin(DEBUG_BAUT_RATE);
    Serial.println("#TAICHI: ======================setup()=====================");
    #endif

    SetGNDPins();

    move.Stop();
    servo.StopAndReset();

    //在开始运行前依次检测各灰度传感器下方黑白是否正常
    CheckGrayStatus();

    //计算最大数组下标
    max_flag = sizeof(route) / sizeof(route[0]) - 1;

    //前往 0, 0
    //沿线直行，到后端传感器接触下一条线离开函数
    LineForward(BACK_END);

    //已越过 0, 0 正式进入循环
}


void loop()
{
    #ifdef TAICHI_DEBUG
    loop_time++;
    Serial.println("#TAICHI: ====================New loop()====================");
    Serial.print("#TAICHI: loop_time: "); Serial.println(loop_time);
    Serial.print("#TAICHI: pass: ["); Serial.print(route[passed_flag][X]); Serial.print(", "); Serial.print(route[passed_flag][Y]); Serial.print("]");
    Serial.print(" flag: "); Serial.print(passed_flag);
    Serial.print(" TYPE: "); Serial.println((int)route[passed_flag][TYPE]);
    Serial.print("#TAICHI: next: ["); Serial.print(route[next_flag][X]); Serial.print(", "); Serial.print(route[next_flag][Y]); Serial.print("]");
    Serial.print(" next_position: "); Serial.print((int)next_position);
    Serial.print(" TYPE: "); Serial.println((int)route[next_flag][TYPE]);
    Serial.print("#TAICHI: is_claw_catch: "); Serial.print((int)is_claw_catch); Serial.print(" is_claw_ok: "); Serial.println((int)is_claw_ok);
    #endif

    int8_t& passed_flag_type = route[passed_flag][TYPE];
    int8_t& next_flag_type = route[next_flag][TYPE];
    int8_t& next_next_flag_type = route[next_next_flag][TYPE];

    //情况一：刚完整经过普通点，下一个点为普通点或携带点
    if (passed_flag_type == NORMAL_POINT && (next_flag_type == NORMAL_POINT || next_flag_type == CARRY_POINT))   
    {
        if (next_position == FRONT_NEXT)
        {
            //沿线直行，到前端传感器接触下一条线离开函数
            LineForward(FRONT_END);

            //若下一点为携带点
            if (next_flag_type == CARRY_POINT)
            {
                //底盘携带
                is_carry = true;

                //越过点成为普通点
                next_flag_type = NORMAL_POINT;
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
    else if (passed_flag_type == NORMAL_POINT && next_flag_type == CATCH_POINT)
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
        next_flag_type = NORMAL_POINT;
    }
    //情况三：刚完整经过普通点，下一个点为释放点（使用机械臂）
    else if (passed_flag_type == NORMAL_POINT && next_flag_type == RELEASE_POINT)
    {
        //沿线直行，在释放柱离开函数
        LineForward(RELEASE_END);

        //停止前进
        move.Stop();
        delay(RELEASE_STOP_DELAY_TIME);

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
        delay(RESET_DELAY_TIME);

        //无循迹前进到释放柱
        move.Forward();
        delay(RELEASE_MOVE_BACKWARD_AND_FORWARD_DELAY_TIME);

        //下一点朝向为后
        next_position = BACK_NEXT;

        //停止前进
        move.Stop();
        delay(RELEASE_STOP_DELAY_TIME);
    }
    //情况四：刚完整经过释放点（使用机械臂）或增益抓取点，下一个点为普通点
    else if ((passed_flag_type == RELEASE_POINT || passed_flag_type == GAIN_POINT) && next_flag_type == NORMAL_POINT)
    {
        //底盘携带清空
        is_carry = false;
        
        //沿线后退，到后端传感器接触下一条线离开函数
        LineBackward(BACK_END);

        //继续后退或转向
        TurnDirection();
    }
    //情况五：刚完整经过普通点，下一个点为释放点（从底盘）
    else if (passed_flag_type == NORMAL_POINT && next_flag_type == GETOUT_POINT)
    {
        //沿线直行，到后端传感器接触下一条线离开函数
        LineForward(BACK_END);

        //停止前进
        move.Stop();

        //下一点朝向为后
        next_position = BACK_NEXT;
    }
    //情况六：刚完整经过释放点（从底盘），下一个点为普通点
    else if (passed_flag_type == GETOUT_POINT && next_flag_type == NORMAL_POINT)
    {
        //沿线后退，到前端传感器接触线离开函数
        LineBackward(FRONT_END);

        //底盘携带清空
        is_carry = false;
        
        //沿线后退，到后端传感器接触线离开函数
        LineBackward(BACK_END);

        //继续后退或转向
        TurnDirection();
    }
    //情况七：刚完整经过普通点，下一个点为增益抓取点
    else if (passed_flag_type == NORMAL_POINT && next_flag_type == GAIN_POINT)
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
            delay(RELEASE_STOP_DELAY_TIME);

            //无循迹后退到真实抓取位置
            move.Backward();
            delay(GAIN_MOVE_BACKWARD_AND_FORWARD_DELAY_TIME);

            //停止后退
            move.Stop();

            //放下爪子
            servo.GainDown();
            delay(GAINDOWN_DELAY_TIME);

            //稍稍无循迹前进
            move.Forward();
            delay(GAIN_MOVE_LITTLEFORWARD_DELAY_TIME);

            //停止前进
            move.Stop();

            //抓取
            CatchAndCheck(CATCH_TYPE_GAIN);

            //无循迹前进到释放柱（增益柱）
            move.Forward();
            delay(GAIN_MOVE_BACKWARD_AND_FORWARD_DELAY_TIME);
        }

        //下一点朝向为后
        next_position = BACK_NEXT;

        //停止前进
        move.Stop();
        delay(RELEASE_STOP_DELAY_TIME);
    }
    //出现错误
    else 
    {
        move.Stop();

        #ifdef TAICHI_DEBUG
        DebugCanNotContinue("CHOOSE LOOP");
        #endif

        while (1) {}
    }

    //更新标记，继续循环
    UpdateFlag();

    #ifdef TAICHI_DEBUG
    Serial.println("#TAICHI: ====================End loop()====================");
    #endif
}


//设置接口低电平作为额外地
void SetGNDPins(void)
{
    int pin_num = sizeof(gnd_pins) / sizeof(uint8_t);

    for (int i = 0; i < pin_num; i++)
    {
        pinMode(gnd_pins[i], OUTPUT);
        digitalWrite(gnd_pins[i], LOW);
    }
}


//在开始运行前依次检测各灰度传感器下方黑白是否正常，若不正常，异常传感器闪烁，程序不继续进行
void CheckGrayStatus(void)
{
    //若正常，1 2 5 6 号传感器检测到黑色，3 4 7 号传感器检测到白色
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
        else if (!sensor.IsWhite(GRAY_7))
            sensor.FlashGraySensor(GRAY_7);
        else is_status_right = true;
    }

    #ifdef TAICHI_DEBUG
    Serial.println("#TAICHI: Gray Sensor Status OK!");
    #endif
}


//更新标记
void UpdateFlag(uint8_t jump_choice)
{
    if (jump_choice == NO_JUMP)
    {
        passed_flag = next_flag;

        next_flag = next_next_flag;
        
        if (++next_next_flag > max_flag)
            next_next_flag = 0;
    }
    else
    {
        int current_passed_flag = passed_flag;

        UpdateFlag(NO_JUMP);
        UpdateFlag(NO_JUMP);

        passed_flag = current_passed_flag;
    }
}


//计算方向，同时更改完成转向后相对下一点的朝向
uint8_t CalcDirection(void)
{
    //计算第三点与第一点的相对坐标 rx0, ry0
    int8_t rx0 = route[next_next_flag][X] - route[passed_flag][X];
    int8_t ry0 = route[next_next_flag][Y] - route[passed_flag][Y];

    //计算当前小车朝向的方向向量 vx, vy
    int8_t vx = route[next_flag][X] - route[passed_flag][X];
    int8_t vy = route[next_flag][Y] - route[passed_flag][Y];

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
    else DebugCanNotContinue("CALC DIRECTION <1>"); //调试用

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
    else DebugCanNotContinue("CALC DIRECTION <2>"); //调试用
}


//由灰度比计算修正减速比
float CalcFixSpeedRate(float gray_deviation_rate)
{
    return -50.0 * pow((gray_deviation_rate - 1.0), 2.0) + 1.0;
}


//沿线直行，在触发条件后离开函数但不停止
void LineForward(uint8_t end_position, float speed_rate)
{
    #ifdef TAICHI_DEBUG
    //调试输出沿线直行状态
    Serial.print("#TAICHI: Line Forward");
    Serial.print(" end_position: ");
    Serial.println((int)end_position);
    #endif

    //记录开始时间
    unsigned long begin_time = millis();
    //记录灰度传感器匹配次数
    uint8_t gray_match_time = 0;

    while(1)
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
        else //均不符合，则低速后退，尝试回到白线上
        {
            //move.Backward(LINE_FIND_SPEED_RATE);
        }

        if (end_position == FRONT_END) //前端接触线离开函数
        {
            if (sensor.IsWhite(GRAY_1) || sensor.IsWhite(GRAY_2))
                gray_match_time++;
        }
        else if (end_position == BACK_END) //后端接触线离开函数
        {
            if (sensor.IsWhite(GRAY_5) || sensor.IsWhite(GRAY_6))
                gray_match_time++;
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
        if (gray_match_time == 2)
            break;
    }

    #ifdef TAICHI_DEBUG
    //调试输出沿线直行结束
    Serial.println("#TAICHI: End Line Forward");
    #endif
}


//沿线后退，在触发条件后离开函数但不停止
void LineBackward(uint8_t end_position, float speed_rate)
{
    #ifdef TAICHI_DEBUG
    //调试输出沿线后退状态
    Serial.print("#TAICHI: Line Backward");
    Serial.print(" end_position: ");
    Serial.println((int)end_position);
    #endif

    //记录灰度传感器匹配次数
    uint8_t gray_match_time = 0;

    while(1)
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
        else //均不符合，则低速前进，尝试回到白线上
        {
            //move.Forward(LINE_FIND_SPEED_RATE);
        }

        if (end_position == FRONT_END) //前端接触线离开函数
        {
            if (sensor.IsWhite(GRAY_1) || sensor.IsWhite(GRAY_2))
                gray_match_time++;
        }
        else //后端接触线离开函数
        {
            if (sensor.IsWhite(GRAY_5) || sensor.IsWhite(GRAY_6))
                gray_match_time++;
        }

        //对应前端接触线离开函数和后端接触线离开函数的情况
        if (gray_match_time == 2)
            break;
    }

    #ifdef TAICHI_DEBUG
    //调试输出沿线后退结束
    Serial.println("#TAICHI: End Line Backward");
    #endif
}


//直行或后退或转向，完成后离开函数但不停止。会自动跳过无需前往的释放点
void TurnDirection(float speed_rate)
{    
    //若下下点为释放点或增益抓取点，判断是否需要跳过
    if ((route[next_next_flag][TYPE] == RELEASE_POINT && (!is_claw_catch || is_carry)) 
        || (route[next_next_flag][TYPE] == GETOUT_POINT && !is_carry)
        || (route[next_next_flag][TYPE] == GAIN_POINT && is_claw_catch)
        )
    {
        UpdateFlag(JUMP_DEAD_ROAD);

        #ifdef TAICHI_DEBUG
        //调试输出直行或后退或转向状态
        Serial.println("#TAICHI: JUMP THE RELEASE/GETOUT/GAIN POINT FOR STATUS REASON");
        #endif
    }
    
    uint8_t direction = CalcDirection();
    
    #ifdef TAICHI_DEBUG
    //调试输出直行或后退或转向状态
    Serial.print("#TAICHI: Turn Direction");
    Serial.print(" direction: ");
    Serial.println((int)direction);
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

        //等待一定时间后检测是否摆正
        delay(BEFORE_CHECK_STRAIGHT_DELAY);
        while (!sensor.IsWhite(GRAY_7)) {}
    }

    #ifdef TAICHI_DEBUG
    //调试输出直行或后退或转向结束
    Serial.println("#TAICHI: End Turn Direction");
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
        if (sensor.IsPushed(BUTTON_1)) //开关 1 闭合，即爪子两端接触，说明抓取失败
        {
            #ifdef TAICHI_DEBUG
            //调试输出失败信息
            Serial.print("#TAICHI: **********FAIL CATCH!**********");
            Serial.print(" catch_times: "); Serial.println((int)catch_times);
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
    Serial.println("#TAICHI: SUCCUESS CATCH!");
    #endif
    #endif

    #ifdef DISABLE_CLAW_CHECK
    #ifdef TAICHI_DEBUG
    //调试输出结束信息
    Serial.println("#TAICHI: CATCH WITHOUT CHECK!");
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
    Serial.println("#TAICHI: DISABLE CLAW CHECK!");
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
        if (!sensor.IsPushed(BUTTON_1)) //开关 1 打开，即爪子两端脱离接触，说明打开爪子成功
        {
            #ifdef TAICHI_DEBUG
            //调试输出成功信息
            Serial.println("#TAICHI: SUCCUESS OPEN CLAW!");
            #endif
            
            is_claw_ok = true;
            return true;
        }
    }

    #ifdef TAICHI_DEBUG
    //调试输出失败信息
    Serial.println("#TAICHI: $$$$$$$$$$FAIL CLAW!$$$$$$$$$$");
    #endif
    
    is_claw_ok = false;
    return false;
}