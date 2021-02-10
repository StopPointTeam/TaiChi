#ifndef MOVE_H
#define MOVE_H

//轮胎定义
#define LEFT_A_WHEEL 0
#define LEFT_B_WHEEL 1
#define RIGHT_A_WHEEL 2
#define RIGHT_B_WHEEL 3

//轮胎旋转方向定义，包括停止
#define FORWARD_ROTATION 0
#define BACKWARD_ROTATION 1
#define STOP_ROTATION 3

//运动状态定义
#define FORWARD 0
#define BACKWARD 1
#define FORLEFTWARD 2
#define BACKLEFTWARD 3
#define FORRIGHTWARD 4
#define BACKRIGHTWARD 5
#define STOP 6

//默认全局速度比率
#define DEFAULT_GLOBAL_SPEED_RATE 100

//转向时一侧减速的比率
#define TRUN_SPEED_RATE 50

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
    Move(int global_speed_rate);

    void SetGlobalSpeedRate(int global_speed_rate); //设置全局速度比率

    uint8_t GetCurrentMove(void); //获取当前运动方向
    int GetCurrentSpeedRate(void); //获取当前运动速度比率

    void Wheel(uint8_t wheel, uint8_t rotation, int speed_rate = 100); //控制某个轮子转动

    void Forward(int speed_rate = 100); //前进
    void Backward(int speed_rate = 100); //后退
    void ForLeftward(int speed_rate = 100); //向前左转
    void ForRightward(int speed_rate = 100); //向前右转
    void BackLeftward(int speed_rate = 100); //向后左转
    void BackRightward(int speed_rate = 100); //向后右转
    void Stop(void); //制动

private:
    int global_speed_rate; //全局速度比率
    uint8_t current_move; //当前运动状态
    int current_speed_rate; //当前运动速度比率
};


#endif