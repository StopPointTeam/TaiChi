#include <Arduino.h>
#include <NeoHWSerial.h>

#include "radioTaiChi.h"


//静态变量
HandleMessageFunction Radio::hm_func;
NeoHWSerial* Radio::NeoSerialX = &RADIO_SERIAL_NUM;


Radio::Radio()
{
    EnableReceiveInterrupt();
}


//打开串口
void Radio::BeginTransmit(unsigned long baud_rate)
{
    NeoSerialX->begin(baud_rate);
}


//发送
void Radio::Send(char* message, uint8_t send_type, uint8_t send_times)
{
    //强制发送，禁用接收中断，防止发送被打断
    if (send_type == FORCE_SEND)
        DisableReceiveInterrupt();

    //生成完整通信包
    char full_message[FULL_MESSAGE_SIZE];
    uint8_t message_length = strlen(message);
    char check_str[CHECK_STR_LENGTH + 1];

    message_length = message_length < MAX_REAL_MESSAGE_SIZE ? message_length : MAX_REAL_MESSAGE_SIZE;

    itoa(message_length, check_str, 10);

    for (uint8_t i = 0, j = 0, k = 0; i < FULL_MESSAGE_SIZE; i++)
    {
        if (i < BLANK_CHAR_LENGTH)
        {
            full_message[i] = BLANK_CHAR;
        }
        else if (i == BLANK_CHAR_LENGTH)
        {
            full_message[i] = START_CHAR;
        }
        else if (j < message_length && j < MAX_REAL_MESSAGE_SIZE)
        {
            full_message[i] = message[j];
            j++;
        }
        else if (j == message_length || j == MAX_REAL_MESSAGE_SIZE)
        {
            full_message[i] = CHECK_CHAR;
            j++;
        }
        else if (k < CHECK_STR_LENGTH)
        {
            full_message[i] = check_str[k];
            k++;
            
            if (check_str[k] == '\0')
                k = CHECK_STR_LENGTH;
        }
        else
        {
            full_message[i] = END_CHAR;
        }
    }

    //发送完整通信包
    for (uint8_t times = 0; times < send_times; times++)
    {
        NeoSerialX->print(full_message);
    }

    #ifdef RADIO_DEBUG
    NeoSerial.print("#RADIO:  SEND: ");
    full_message[FULL_MESSAGE_SIZE - 1] = '\0';
    NeoSerial.print(full_message);
    NeoSerial.print(" TIMES: ");
    NeoSerial.println(send_times);
    #endif

    if (send_type == FORCE_SEND)
        EnableReceiveInterrupt();
}


//设置接收回调函数
void Radio::SetHandleMessageFunction(HandleMessageFunction hm_func)
{
    Radio::hm_func = hm_func;
}


//禁用接收中断
void Radio::DisableReceiveInterrupt() //禁用接收中断
{
    NeoSerialX->detachInterrupt();
}


//恢复接收中断
void Radio::EnableReceiveInterrupt() //恢复接收中断
{
    NeoSerialX->attachInterrupt(Receive);
}


//接收，使用中断触发
bool Radio::Receive(uint8_t ch, uint8_t status)
{
    static long begin_time = millis();
    static bool is_start_record = false;
    static bool is_check_record = false;
    static bool is_end_record = false;
    static char message[FULL_MESSAGE_SIZE];
    static char check_str[CHECK_STR_LENGTH + 1];
    static uint8_t i = 0, j = 0;

    //若上一次中断已在 100 ms前，可以认为当前是一次新的传输
    if (millis() - begin_time > 100)
    {
        begin_time = millis(); //更新时间
        
        is_start_record = false;
        is_check_record = false;
        is_end_record = false;
    }
    else
    {
        begin_time = millis(); //更新时间

        switch (ch)
        {
        case START_CHAR:
        {
            if (is_end_record == false)
            {
                is_start_record = true;
                is_check_record = false;
                
                //重置字符串
                for (uint8_t k = 0; k < i; k++)
                    message[k] = '\0';
                for (uint8_t k = 0; k < j; k++)
                    check_str[k] = '\0';

                i = 0;
                j = 0;
            }
        } break;

        case CHECK_CHAR:
        {
            if (is_end_record == false && is_start_record == true)
            {
                is_start_record = false;
                is_check_record = true;
            }
        } break;

        case END_CHAR:
        {
            if (is_end_record == false && is_check_record == true)
            {
                is_check_record = false;

                if (j <= CHECK_STR_LENGTH)
                    check_str[j] = '\0';

                if (atoi(check_str) == strlen(message)) //位数校验成功
                {
                    is_end_record = true;
                    
                    #ifdef RADIO_DEBUG
                    NeoSerial.print("#RADIO:  RECEIVE: ");
                    NeoSerial.println(message);
                    #endif
                    
                    hm_func(message);
                }
                else //位数校验失败
                {
                    #ifdef RADIO_DEBUG
                    NeoSerial.println("#RADIO:  RECEIVE CHECK FAIL!");
                    #endif
                }
            }
        } break;

        default:
        {
            if (is_start_record == true)
            {
                if (i < FULL_MESSAGE_SIZE)
                    message[i] = ch;
                i++;
            }
            else if (is_check_record == true)
            {
                if (j <= CHECK_STR_LENGTH)
                    check_str[j] = ch;
                j++;
            }
        }
        }
    }
    
    return false;
}