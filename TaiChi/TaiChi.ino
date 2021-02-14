#include "moveTaiChi.h" //轮胎运动库
#include "sensorTaiChi.h" //传感器库
#include "servoTaiChi.h" //舵机库


Move move; //轮胎运动实例
Sensor sensor; //传感器实例
Servo servo; //舵机实例


//****************************************运动路径****************************************
//坐标点操作定义
#define NORMAL_POINT 0
#define CATCH_POINT 1
#define RELEASE_POINT 2

//坐标点数组定义
#define X 0
#define Y 1
#define TYPE 2

int8_t route[][3] = 
{
    {0, 0, NORMAL_POINT},
    {0, 1, NORMAL_POINT},
    {1, 1, CATCH_POINT},
    {1, 2, NORMAL_POINT},
    {2, 2, NORMAL_POINT},
    {2, 3, RELEASE_POINT}
};
//***************************************************************************************


//****************************************可调参数****************************************
//重置留时
#define RESET_DELAY_TIME 10000
//抓取留时
#define CATCH_DELAY_TIME 10000
//释放留时
#define RELEASE_DELAY_TIME 10000

//传感器判断转向完成延时
#define TRUN_CHECK_DELAY 1000

//修正循迹时单侧减速比率
#define LINE_FIX_SPEED_RATE 0.8
//***************************************************************************************


//****************************************全局变量****************************************
//刚刚越过的点的数组位置标记
int passed_flag = 0;

#define FRONT_NEXT 0
#define BACK_NEXT 1
//下一点朝向
uint8_t next_position = FRONT_NEXT;
//***************************************************************************************


//****************************************自定函数****************************************
//计算方向
uint8_t CalcDirection(void);

#define FRONT_END 0
#define BACK_END 1
#define CATCH_END 2
#define RELEASE_END 3
//沿线直行，在触发条件后停止
void LineForward(uint8_t end_position, double speed_rate = 1.0);

//沿线后退，在触发条件后停止
void LineBackward(uint8_t end_position, double speed_rate = 1.0);

//直行或后退或转向
void TurnDirection(uint8_t direction, double speed_rate = 1.0);
//***************************************************************************************


void setup() 
{
    move.Stop();
    servo.StopAndReset();

    //前往 0, 0
    //沿线直行，到后端传感器接触下一条线为止
    LineForward(BACK_END);

    //已越过 0, 0 正式进入循环
}


void loop()
{
    //情况一：刚完整经过普通点，下一个点为普通点
    if (route[passed_flag][TYPE] == NORMAL_POINT && route[passed_flag + 1][TYPE] == NORMAL_POINT)   
    {
        uint8_t direction = CalcDirection();

        if (direction == FORWARD || direction == FORLEFTWARD || direction == FORRIGHTWARD)
        {
            //沿线直行，到前端传感器接触下一条线为止
            LineForward(FRONT_END);
        }
        else
        {
            //沿线后退，到后端传感器接触下一条线为止
            LineBackward(BACK_END);
        }

        //继续直行或后退或转向
        TurnDirection(direction);

        //更新标记，继续循环
        passed_flag++;
    }
    //情况二：刚完整经过普通点，下一个点为抓取点
    else if (route[passed_flag][TYPE] == NORMAL_POINT && route[passed_flag + 1][TYPE] == CATCH_POINT)
    {
        //沿线直行，在抓取位置停止
        LineForward(CATCH_END);

        //抓取
        servo.Catch();
        delay(CATCH_DELAY_TIME); //抓取留时

        //继续沿线直行，到前端传感器接触下一条线为止
        LineForward(FRONT_END);

        //继续直行或转向
        TurnDirection(CalcDirection());

        //抓取完成后，更新标记，将越过的点视为普通点，继续循环
        passed_flag++;
        route[passed_flag][TYPE] = NORMAL_POINT;
    }
    //情况三：刚完整经过普通点，下一个点为释放点
    else if (route[passed_flag][TYPE] == NORMAL_POINT && route[passed_flag + 1][TYPE] == RELEASE_POINT)
    {
        //沿线直行，在释放位置停止
        LineForward(RELEASE_END);

        //释放
        servo.Release();
        delay(RELEASE_DELAY_TIME); //释放留时

        //释放完成后，更新标记，下一点朝向为后，继续循环
        passed_flag++;
        next_position = BACK_NEXT;
    }
    //情况四：刚完整经过释放点，下一个点为普通点
    else if (route[passed_flag][TYPE] == RELEASE_POINT && route[passed_flag + 1][TYPE] == NORMAL_POINT)
    {
        uint8_t direction = CalcDirection();
        
        //沿线后退，到后端传感器接触下一条线为止
        LineBackward(BACK_END);

        //机械臂复原
        servo.Reset();
        delay(RESET_DELAY_TIME); //复原留时

        //继续后退或转向
        TurnDirection(direction);

        //更新标记，继续循环
        passed_flag++;
    }
    else move.Stop(); //DEBUG
}


//计算方向
uint8_t CalcDirection(void)
{
    //计算第三点与第一点的相对坐标 rx0, ry0
    int8_t rx0 = route[passed_flag + 2][X] - route[passed_flag][X];
    int8_t ry0 = route[passed_flag + 2][Y] - route[passed_flag][Y];

    //计算当前小车朝向的方向向量 vx, vy
    int8_t vx = route[passed_flag + 1][X] - route[passed_flag][X];
    int8_t vy = route[passed_flag + 1][Y] - route[passed_flag][Y];

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


    //判断行进方向
    if (rx == 0 && ry == 2)
    {
        if (next_position == FRONT_NEXT)
        {
            return FORWARD; //正对下一点
        }
        else
        { 
            return BACKWARD; //后退，背对下一点
        }
    }
    else if (rx == -1 && ry == 1)
    {
        if (next_position == FRONT_NEXT)
        {
            return FORLEFTWARD; //正对下一点
        }
        else
        {
            next_position = FRONT_NEXT; //向左后退，正对下一点
            return BACKLEFTWARD;
        }
    }
    else if (rx == 1 && ry == 1)
    {
        if (next_position == FRONT_NEXT)
        {
            return FORRIGHTWARD; //正对下一点
        }
        else
        {
            next_position = FRONT_NEXT; //向右后退，正对下一点
            return BACKRIGHTWARD;
        }
    }
    else return 255; //DEBUG
}


//沿线直行，在触发条件后停止
void LineForward(uint8_t end_position, double speed_rate)
{
    //记录开始时间
    int begin_time = micros();

    while(1)
    {
        if (!sensor.IsWhite(GRAY_3) && sensor.IsWhite(GRAY_4)) //左侧越线
        {
            move.ForRightward(speed_rate, LINE_FIX_SPEED_RATE);
        }
        else if (sensor.IsWhite(GRAY_3) && !sensor.IsWhite(GRAY_4)) //右侧越线
        {
            move.ForLeftward(speed_rate, LINE_FIX_SPEED_RATE);
        }
        else if (sensor.IsWhite(GRAY_3) && sensor.IsWhite(GRAY_4)) //在白线上
        {
            move.Forward(speed_rate);
        }
        else //均不符合，则低速后退，尝试回到白线上
        {
            move.Backward(50);
        }

        if (end_position == FRONT_END) //前端接触线为止
        {
            if (sensor.IsWhite(GRAY_1) && sensor.IsWhite(GRAY_2))
                break;
        }
        else if (end_position == BACK_END) //后端接触线为止
        {
            if (sensor.IsWhite(GRAY_5) && sensor.IsWhite(GRAY_6))
                break;
        }
        else if (end_position == CATCH_END) //到达抓取位置为止
        {
            if (micros() - begin_time > 2000)
                break;
        }
        else //到达释放位置为止
        {
            if (micros() - begin_time > 2000)
                break;            
        }
    }
}


//沿线后退，在触发条件后停止
void LineBackward(uint8_t end_position, double speed_rate)
{
    while(1)
    {
        if (!sensor.IsWhite(GRAY_3) && sensor.IsWhite(GRAY_4)) //左侧越线
        {
            move.BackRightward(speed_rate, LINE_FIX_SPEED_RATE);
        }
        else if (sensor.IsWhite(GRAY_3) && !sensor.IsWhite(GRAY_4)) //右侧越线
        {
            move.BackLeftward(speed_rate, LINE_FIX_SPEED_RATE);
        }
        else if (sensor.IsWhite(GRAY_3) && sensor.IsWhite(GRAY_4)) //在白线上
        {
            move.Backward(speed_rate);
        }
        else //均不符合，则低速前进，尝试回到白线上
        {
            move.Forward(50);
        }


        if (end_position == FRONT_END) //前端接触线为止
        {
            if (sensor.IsWhite(GRAY_1) && sensor.IsWhite(GRAY_2))
                break;
        }
        else //后端接触线为止
        {
            if (sensor.IsWhite(GRAY_5) && sensor.IsWhite(GRAY_6))
                break;
        }      
    }    
}


//直行或后退或转向
void TurnDirection(uint8_t direction, double speed_rate)
{    
    if (direction == FORWARD) //继续直行
    {
        //沿线直行，到后端传感器接触线为止
        LineForward(BACK_END, speed_rate);
    }
    else if (direction == BACKWARD) //继续后退
    {
        //沿线后退，到前端传感器接触线为止
        LineForward(FRONT_END, speed_rate);
    }
    else //继续转向
    {
        move.MoveDirection(direction, speed_rate);

        //传感器判断转向完成
        delay(TRUN_CHECK_DELAY); //延时后判断
        while(!(sensor.IsWhite(GRAY_3) && sensor.IsWhite(GRAY_4))) {}
    }
}