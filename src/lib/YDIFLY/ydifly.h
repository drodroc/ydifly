#ifndef __YDIFLY_H

#define __YDIFLY_H

#include <Arduino.h>


/******************** 基本配置 **********************/
#define YDIFLY_SERVO_L_PIN                  10          // 舵机 L 引脚设置
#define YDIFLY_SERVO_R_PIN                  3           // 舵机 R 引脚设置
#define YDIFLY_SERVO_B_PIN                  1           // 舵机 B 引脚设置







typedef enum
{
    SERVO_L,
    SERVO_R,
    SERVO_B,
}ydifly_servo_name_e;

typedef struct
{
    uint32_t *remote;
    uint8_t init_flag;
}ydifly_control_t;


void YDIFlyServoControl( unsigned long now_time_ms );

static void YDIFlyServoControlInit( void );




#endif //__YDIFLY_H

