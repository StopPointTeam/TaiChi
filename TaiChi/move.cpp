#include <Arduino.h>

#include "move.h"


Move::Move()
{
    global_speed_rate = DEFAULT_GLOBAL_SPEED_RATE;
    current_move = STOP;
    current_speed_rate = 0;
}


Move::Move(int global_speed_rate)
{
    Move();
    
    this->global_speed_rate = global_speed_rate;
}


//设置全局速度比率
void Move::SetGlobalSpeedRate(int global_speed_rate)
{
    this->global_speed_rate = global_speed_rate;
}


//获取当前运动方向
uint8_t Move::GetCurrentMove(void)
{
    return current_move;
}


//获取当前运动速度比率
int Move::GetCurrentSpeedRate(void)
{
    return current_speed_rate;
}


//控制某个轮子转动
void Move::Wheel(uint8_t wheel, uint8_t rotation, int speed_rate)
{
    uint8_t pin_in1, pin_in2, pin_ena;

    switch (wheel)
    {
    case LEFT_A_WHEEL:
    {
        pin_in1 = LEFT_L298N_IN1;
        pin_in2 = LEFT_L298N_IN2;
        pin_ena = LEFT_L298N_ENA;
    } break;

    case LEFT_B_WHEEL:
    {
        pin_in1 = LEFT_L298N_IN3;
        pin_in2 = LEFT_L298N_IN4;
        pin_ena = LEFT_L298N_ENB;
    } break; 

    case RIGHT_A_WHEEL:
    {
        pin_in1 = RIGHT_L298N_IN1;
        pin_in2 = RIGHT_L298N_IN2;
        pin_ena = RIGHT_L298N_ENA;
    } break;

    case RIGHT_B_WHEEL:
    {
        pin_in1 = RIGHT_L298N_IN3;
        pin_in2 = RIGHT_L298N_IN4;
        pin_ena = RIGHT_L298N_ENB;
    }
    }

    if (rotation == STOP_ROTATION) //停止转动
    {
        digitalWrite(pin_in1, LOW);
        digitalWrite(pin_in2, LOW);
        return; //结束函数
    }
    
    analogWrite(pin_ena, speed_rate * global_speed_rate * 255.0 / 10000.0); //设置 PWM 波，即转速

    if (rotation == FORWARD_ROTATION) //向前转动
    {
        digitalWrite(pin_in1, LOW);
        digitalWrite(pin_in2, HIGH);
    }
    else //向后转动
    {
        digitalWrite(pin_in1, HIGH);
        digitalWrite(pin_in2, LOW);
    }
}


//前进
void Move::Forward(int speed_rate)
{
    Wheel(LEFT_A_WHEEL, FORWARD_ROTATION, speed_rate);
    Wheel(LEFT_B_WHEEL, FORWARD_ROTATION, speed_rate);
    Wheel(RIGHT_A_WHEEL, FORWARD_ROTATION, speed_rate);
    Wheel(RIGHT_B_WHEEL, FORWARD_ROTATION, speed_rate);

    current_move = FORWARD;
    current_speed_rate = speed_rate * global_speed_rate / 100.0;
}


//后退
void Move::Backward(int speed_rate)
{
    Wheel(LEFT_A_WHEEL, BACKWARD_ROTATION, speed_rate);
    Wheel(LEFT_B_WHEEL, BACKWARD_ROTATION, speed_rate);
    Wheel(RIGHT_A_WHEEL, BACKWARD_ROTATION, speed_rate);
    Wheel(RIGHT_B_WHEEL, BACKWARD_ROTATION, speed_rate);

    current_move = BACKWARD;    
    current_speed_rate = speed_rate * global_speed_rate / 100.0;
}


//向前左转
void Move::ForLeftward(int speed_rate)
{
    Wheel(LEFT_A_WHEEL, FORWARD_ROTATION, TRUN_SPEED_RATE * speed_rate / 100.0);
    Wheel(LEFT_B_WHEEL, FORWARD_ROTATION, TRUN_SPEED_RATE * speed_rate / 100.0);
    Wheel(RIGHT_A_WHEEL, FORWARD_ROTATION, speed_rate);
    Wheel(RIGHT_B_WHEEL, FORWARD_ROTATION, speed_rate);

    current_move = FORLEFTWARD;
    current_speed_rate = speed_rate * global_speed_rate / 100.0;
}


//向前右转
void Move::ForRightward(int speed_rate)
{
    Wheel(LEFT_A_WHEEL, FORWARD_ROTATION, speed_rate);
    Wheel(LEFT_B_WHEEL, FORWARD_ROTATION, speed_rate);
    Wheel(RIGHT_A_WHEEL, FORWARD_ROTATION, TRUN_SPEED_RATE * speed_rate / 100.0);
    Wheel(RIGHT_B_WHEEL, FORWARD_ROTATION, TRUN_SPEED_RATE * speed_rate / 100.0);

    current_move = FORRIGHTWARD;   
    current_speed_rate = speed_rate * global_speed_rate / 100.0;
}


//向后左转
void Move::BackLeftward(int speed_rate)
{
    Wheel(LEFT_A_WHEEL, BACKWARD_ROTATION, TRUN_SPEED_RATE * speed_rate / 100.0);
    Wheel(LEFT_B_WHEEL, BACKWARD_ROTATION, TRUN_SPEED_RATE * speed_rate / 100.0);
    Wheel(RIGHT_A_WHEEL, BACKWARD_ROTATION, speed_rate);
    Wheel(RIGHT_B_WHEEL, BACKWARD_ROTATION, speed_rate);

    current_move = BACKLEFTWARD;    
    current_speed_rate = speed_rate * global_speed_rate / 100.0;
}


//向后右转
void Move::BackRightward(int speed_rate)
{
    Wheel(LEFT_A_WHEEL, BACKWARD_ROTATION, speed_rate);
    Wheel(LEFT_B_WHEEL, BACKWARD_ROTATION, speed_rate);
    Wheel(RIGHT_A_WHEEL, BACKWARD_ROTATION, TRUN_SPEED_RATE * speed_rate / 100.0);
    Wheel(RIGHT_B_WHEEL, BACKWARD_ROTATION, TRUN_SPEED_RATE * speed_rate / 100.0);

    current_move = BACKRIGHTWARD;     
    current_speed_rate = speed_rate * global_speed_rate / 100.0;    
}


//制动
void Move::Stop(void)
{
    Wheel(LEFT_A_WHEEL, STOP_ROTATION);
    Wheel(LEFT_B_WHEEL, STOP_ROTATION);
    Wheel(RIGHT_A_WHEEL, STOP_ROTATION);
    Wheel(RIGHT_B_WHEEL, STOP_ROTATION);  

    current_move = STOP;
    current_speed_rate = 0;
}