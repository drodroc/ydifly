#include "ydifly.h"
#include "CRSF.h"

ydifly_control_t ydifly;

static void YDIFlyInit( void )
{
    ydifly.remote = CRSF::ChannelData;
}

void YDIFlyControl( unsigned long now_time_ms )
{
    static unsigned long last_time_ms_control = now_time_ms;    // 上一次时间
    static unsigned long last_time_ms_debug = now_time_ms;    // 上一次时间
    static unsigned long last_time_ms = now_time_ms;    // 上一次时间

    /*输入保护*/
    if( last_time_ms > now_time_ms )    // 判断是否时间溢出，即超出49.7天
    {
        last_time_ms = now_time_ms;             // 时间溢出重新计数
        last_time_ms_control = now_time_ms;     // 时间溢出重新计数
        last_time_ms_debug = now_time_ms;       // 时间溢出重新计数
    }

    if( ydifly.init_flag == 0 )     // 如果还没有初始化
    {
        ydifly.init_flag = 1;       // 设置flag
        YDIFlyInit();
    }
    else    // 已经完成初始化，启动任务
    {
        /*舵机周期控制*/
        if( now_time_ms - last_time_ms_control >= YDIFLY_SERVO_CONTROL_CYCLE )
        {
            last_time_ms_control += YDIFLY_SERVO_CONTROL_CYCLE;     // 时间补全

            YDIFlyFlyingControl( now_time_ms );
        }

        /*调试周期控制*/
        if( now_time_ms - last_time_ms_debug >= YDIFLY_SERVO_DEBUG_CYCLE )
        {
            last_time_ms_debug += YDIFLY_SERVO_DEBUG_CYCLE;       // 时间补全
            
            Serial.printf("\r\n%d,%d,%d,%d\r\n", ydifly.remote[YDIFLY_REMOTE_LX],ydifly.remote[YDIFLY_REMOTE_LY],ydifly.remote[YDIFLY_REMOTE_RX],ydifly.remote[YDIFLY_REMOTE_RY]);
        }
    }

    last_time_ms = now_time_ms;
}

static void YDIFlyFlyingControl( unsigned long now_time_ms )
{
    float angle_l, angle_r;
}

extern void startWaveform8266(uint8_t pin, uint32_t timeHighUS, uint32_t timeLowUS);
static void YDIFlyServoAngleControl( ydifly_servo_name_e servo_name, float angle_set )
{
    /*舵机控制角度和PWM占空比换算*/
    float time_hight_us;
    time_hight_us = 500.0f + 2000.0f*angle_set/180.0f;   // 角度和PWM高电平时间换算

    /*对应角度的舵机控制*/
    switch (servo_name)
    {
    case SERVO_L:
        startWaveform8266(YDIFLY_SERVO_L_PIN, time_hight_us, 20000);
        break;
    case SERVO_R:
        startWaveform8266(YDIFLY_SERVO_R_PIN, time_hight_us, 20000);
        break;
    case SERVO_B:
        startWaveform8266(YDIFLY_SERVO_B_PIN, time_hight_us, 20000);
        break;
    default:
        break;
    }
}
