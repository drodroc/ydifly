#include "ydifly.h"
#include "CRSF.h"


static void YDIFlyServoControlInit( void )
{
    
}

void YDIFlyServoControl( unsigned long now_time_ms )
{
    unsigned long last_time_ms = now_time_ms;           // 定义上一次进入时的时间
    unsigned long last_time_control_ms = now_time_ms;   // 定义上一次进入时控制循环的时间
    unsigned long last_time_debug_ms = now_time_ms;     // 定义上一次进入时debug循环的时间

    /* 时间溢出保护 */
    if( last_time_ms > now_time_ms )    // 如果溢出了，即超过了46.7天
    {
        last_time_ms = now_time_ms;             // 时间重新赋值
        last_time_control_ms = now_time_ms;     // 时间重新赋值
        last_time_debug_ms = now_time_ms;       // 时间重新赋值
    }

    /* 控制循环 */
    if( now_time_ms - last_time_control_ms > )
    {

    }


    /* debug循环 */

}













