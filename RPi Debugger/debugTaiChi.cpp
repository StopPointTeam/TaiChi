#include <iostream>
#include <string>
#include <errno.h>
#include <time.h>
#include <stdlib.h>
#include <signal.h>
#include <wiringPi.h>
#include <wiringSerial.h>


using namespace std;


#define DEBUG_GPIO 0
#define DEBUG_BAUT_RATE 115200

//终端颜色
#define COLOR_RED   "\x1B[31m"
#define COLOR_GRN   "\x1B[32m"
#define COLOR_YEL   "\x1B[33m"
#define COLOR_BLU   "\x1B[34m"
#define COLOR_MAG   "\x1B[35m"
#define COLOR_CYN   "\x1B[36m"
#define COLOR_WHT   "\x1B[37m"
#define COLOR_RESET "\x1B[0m"


//向 Arduino 发送暂停命令
static void SendDebugPause(int siginal)
{
    digitalWrite(DEBUG_GPIO, LOW);
}


//向 Arduino 发送执行命令
static void SendDebugContinue(int siginal)
{
    digitalWrite(DEBUG_GPIO, HIGH);
    cout << '\n';
}


//获取当前时间字符串
string GetStrTime()
{
    time_t timep;
    time(&timep);
    char tmp[9];
    strftime(tmp, sizeof(tmp), "%H:%M:%S", localtime(&timep));
    return tmp;
}


int main(void)
{
    //注册 linux 终端信号
    signal(SIGINT, SendDebugPause); //Ctrl + C
    signal(SIGTSTP, SendDebugContinue); //Ctrl + Z

    //初始化树莓派 GPIO
    wiringPiSetup();
    pinMode(DEBUG_GPIO, OUTPUT);

    //暂停执行，等待发送执行命令
    digitalWrite(DEBUG_GPIO, LOW);

    //打开串口
	int fd = serialOpen("/dev/ttyACM0", DEBUG_BAUT_RATE);
	while (1)
	{
        string line_head = GetStrTime() + " -> ";
        string line_info;
    
        while (1)
        {
            if (serialDataAvail(fd) > 0) //串口接收到内容
            {
                char serial_get_c = serialGetchar(fd);
                line_info += serial_get_c;

                if (serial_get_c == '\n')
                {
                    string line_info_head = line_info.substr(0, 8);
                    string line_type;

                    //根据调试信息类型选择对应的颜色
                    if (line_info_head == "#TAICHI:")
                    {
                        line_type = COLOR_YEL;
                    }
                    else if (line_info_head == "#MOVE:  ")
                    {
                        line_type = COLOR_RED;
                    }
                    else if (line_info_head == "#SENSOR:")
                    {
                        line_type = COLOR_GRN;
                    }
                    else if (line_info_head == "#SERVO: ")
                    {
                        line_type = COLOR_BLU;
                    }

                    cout << line_head + line_type + line_info + COLOR_RESET;
                    break;
                }
            }
        }
	}

	return 0;
}