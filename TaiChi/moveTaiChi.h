#ifndef MOVETAICHI_H
#define MOVETAICHI_H

//轮胎定义
#define LEFT_A_WHEEL 0
#define LEFT_B_WHEEL 1
#define RIGHT_A_WHEEL 2
#define RIGHT_B_WHEEL 3

//轮胎旋转方向定义，包括停止
#define FORWARD_ROTATION 0
#define BACKWARD_ROTATION 1
#define STOP_ROTATION 6

//运动方向、状态定义，与轮胎旋转方向定义兼容
#define FORWARD 0
#define BACKWARD 1
#define FORLEFTWARD 2
#define FORRIGHTWARD 3
#define BACKLEFTWARD 4
#define BACKRIGHTWARD 5
#define STOP 6

//默认全局速度比率
#define DEFAULT_GLOBAL_SPEED_RATE 1.0

//默认转向时一侧减速的比率
#define DEFAULT_TRUN_SPEED_RATE 0.5

//左侧 L298N 接口定义
#define LEFT_L298N_IN1 22
#define LEFT_L298N_IN2 23
#define LEFT_L298N_IN3 24
#define LEFT_L298N_IN4 25
#define LEFT_L298N_ENA 8
#define LEFT_L298N_ENB 9

//右侧 L298N 接口定义
#define RIGHT_L298N_IN1 26
#define RIGHT_L298N_IN2 27
#define RIGHT_L298N_IN3 28
#define RIGHT_L298N_IN4 29
#define RIGHT_L298N_ENA 10
#define RIGHT_L298N_ENB 11


class Move
{
public:
    Move();
    Move(double global_speed_rate);

    void SetGlobalSpeedRate(double global_speed_rate); //设置全局速度比率

    uint8_t GetCurrentMove(void); //获取当前运动方向
    double GetCurrentSpeedRate(void); //获取当前运动速度比率

    void Wheel(uint8_t wheel, uint8_t rotation, double speed_rate = 1.0); //控制某个轮子转动

    void Forward(double speed_rate = 1.0); //前进
    void Backward(double speed_rate = 1.0); //后退
    void ForLeftward(double speed_rate = 1.0, double turn_speed_rate = DEFAULT_TRUN_SPEED_RATE); //向前左转
    void ForRightward(double speed_rate = 1.0, double turn_speed_rate = DEFAULT_TRUN_SPEED_RATE); //向前右转
    void BackLeftward(double speed_rate = 1.0, double turn_speed_rate = DEFAULT_TRUN_SPEED_RATE); //向后左转
    void BackRightward(double speed_rate = 1.0, double turn_speed_rate = DEFAULT_TRUN_SPEED_RATE); //向后右转
    void Stop(void); //制动

    void MoveDirection(uint8_t direction, double speed_rate = 1.0, double turn_speed_rate = DEFAULT_TRUN_SPEED_RATE); //向某方向运动

private:
    double global_speed_rate; //全局速度比率
    uint8_t current_move; //当前运动状态
    double current_speed_rate; //当前运动速度比率
};


#endif