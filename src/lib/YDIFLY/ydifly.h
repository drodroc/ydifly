#ifndef __YDIFLY_H

#define __YDIFLY_H

#include <Arduino.h>

/******************** 基本参数 ******************* */
#define YDIFLY_SERVO_L_PIN                  10      // 引脚设置
#define YDIFLY_SERVO_R_PIN                  3       // 引脚设置
#define YDIFLY_SERVO_B_PIN                  1       // 引脚设置

#define YDIFLY_REMOTE_LX                    0       // 左X轴摇杆
#define YDIFLY_REMOTE_LY                    1       // 左Y轴摇杆
#define YDIFLY_REMOTE_RX                    2       // 右X轴摇杆
#define YDIFLY_REMOTE_RY                    3       // 右Y轴摇杆
#define YDIFLY_REMOTE_SWA                   4       // SWA拨杆
#define YDIFLY_REMOTE_SWB                   5       // SWB拨杆
#define YDIFLY_REMOTE_SWC                   6       // SWC拨杆
#define YDIFLY_REMOTE_SWD                   7       // SWD拨杆

/******************** 控制周期参数 ******************* */
#define YDIFLY_SERVO_CONTROL_CYCLE          50     // 舵机的控制周期，ms
#define YDIFLY_SERVO_DEBUG_CYCLE            100      // debug的控制周期，ms

/******************** 控制周期参数 ******************* */


typedef enum
{
    SERVO_L,    // 左翅膀舵机
    SERVO_R,    // 右翅膀舵机
    SERVO_B,    // 备用舵机
}ydifly_servo_name_e;


typedef struct
{
    uint32_t *remote;               // 遥控数据
    uint8_t init_flag;              // 初始化标志位，0表示还没初始化，1则表示已经完成初始化
}ydifly_control_t;



void YDIFlyControl( unsigned long now_time_ms );

static void YDIFlyFlyingControl( unsigned long now_time_ms );
static void YDIFlyInit( void );
static void YDIFlyServoAngleControl( ydifly_servo_name_e servo_name, float angle_set );


#endif //__YDIFLY_H
