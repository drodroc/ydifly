#ifndef __YDIFLY_H

#define __YDIFLY_H

#include <Arduino.h>

/******************** 基本参数 ******************* */
#define YDIFLY_SERVO_L_PIN                  10      // 引脚设置
#define YDIFLY_SERVO_R_PIN                  1       // 引脚设置

#define YDIFLY_REMOTE_LX                    3       // 左X轴摇杆
#define YDIFLY_REMOTE_LY                    2       // 左Y轴摇杆
#define YDIFLY_REMOTE_RX                    0       // 右X轴摇杆
#define YDIFLY_REMOTE_RY                    1       // 右Y轴摇杆
#define YDIFLY_REMOTE_SWA                   4       // SWA拨杆
#define YDIFLY_REMOTE_SWB                   5       // SWB拨杆
#define YDIFLY_REMOTE_SWC                   6       // SWC拨杆
#define YDIFLY_REMOTE_SWD                   7       // SWD拨杆

#define YDIFLY_REMOTE_JOY_MID               992     // 遥控摇杆的中间值

/******************** 舵机参数设置 ******************* */
#define YDIFLY_SERVO_ANGLE_L_INIT           90
#define YDIFLY_SERVO_ANGLE_R_INIT           90
#define YDIFLY_SERVO_ANGLE_L_MAX            180
#define YDIFLY_SERVO_ANGLE_L_MIN            0
#define YDIFLY_SERVO_ANGLE_R_MAX            180
#define YDIFLY_SERVO_ANGLE_R_MIN            0

/******************** 舵机方向设置 ******************* */
#define YDIFLY_SERVO_L_DIR                  0       // 左舵机摆动方向，0表示正向，1表示反向
#define YDIFLY_SERVO_R_DIR                  1       // 右舵机摆动方向，0表示正向，1表示反向

/******************** 遥控控制系数设置 ******************* */
#define YDIFLY_FACTOR_FREQ                  0.05f
#define YDIFLY_FACTOR_YAW                   0.02f
#define YDIFLY_FACTOR_PITCH                 0.05f
#define YDIFLY_FACTOR_AMP                   0.05f
#define YDIFLY_FACTOR_OFFSET                0.05f

#define YDIFLY_FACTOR_FILTER                0.2f

/******************** 翅膀扑翼周期设置 ******************* */
#define YDIFLY_CYCLE_MIN                    100
#define YDIFLY_CYCLE_MAX                    2000

/******************** 翅膀扑翼幅度设置 ******************* */
#define YDIFLY_AMP0                         60      // 扑翼幅度为 ±60°
#define YDIFLY_AMP1                         70      // 扑翼幅度为 ±70°
#define YDIFLY_AMP2                         80      // 扑翼幅度为 ±80°

/******************** 翅膀上拍下拍速度差 ******************* */
#define YDIFLY_SPEED_DIFF                   0       // 速度差需要在 -YDIFLY_CONTROL_CYCLE~YDIFLY_CONTROL_CYCLE 之间

/******************** 任务控制周期参数 ******************* */
#define YDIFLY_CONTROL_CYCLE                50     // 舵机的控制周期，ms
#define YDIFLY_DEBUG_CYCLE                  100    // debug的控制周期，ms


typedef enum
{
    SERVO_L,    // 左翅膀舵机
    SERVO_R,    // 右翅膀舵机
}ydifly_servo_name_e;


typedef struct
{
    uint32_t *raw;                  // 遥控原始数据
    float amp;                      // 扑翼幅值
    float freq;                     // 扑翼频率
    float offset;                   // 舵机中间值偏移
    float yaw;                      // 偏航角度控制
    float pitch;                    // 俯仰角度控制
    uint8_t swa;                    // SWA 信号，0和2
    uint8_t swb;                    // SWB 信号，0、1、2
    uint8_t swc;                    // SWC 信号，0、1、2
    uint8_t swd;                    // SWD 信号，0、1、2
}ydifly_remote_cmd_t;


typedef struct
{
    uint8_t init_flag;              // 初始化标志位，0表示还没初始化，1则表示已经完成初始化
    ydifly_remote_cmd_t remote;     // 遥控相关参数
    ydifly_remote_cmd_t remote_last;// 上一次遥控参数
}ydifly_control_t;



void YDIFlyControl( unsigned long now_time_ms );

static void YDIFlyServoSinControl( float l_angle_max, float l_angle_min, float r_angle_max, float r_angle_min, float T, float speed_diff );
static void YDIFlyRemoteDecode( ydifly_remote_cmd_t* remote );
static void YDIFlyInit( void );
static void YDIFlyServoAngleControl( ydifly_servo_name_e servo_name, float angle_set );


#endif //__YDIFLY_H
