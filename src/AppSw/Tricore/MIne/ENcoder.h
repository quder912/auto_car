#ifndef _ENCODER_H_
#define _ENCODER_H_

#include "hanshu.h"
#include "allinit.h"
#include "All_Struct.h"




//extern unsigned int i_enc;



/*
 * @use 获取编码器数值显示、蓝牙并返回
 */
signed short GetENC(void);



/*
 * @use 将err进行pid计算并返回
 */
//int ENC_pid(signed short enc,Dian *speed,PID_E *pid);





/*
 * @use 将speed 通过pid赋予电机
 */
//void Setspeed(Dian *speed,PID_E *pid);




/*
 * @use 速度赋予"Dian"结构体
 *
 */
void Change_speed(Dian *speed , int speedba ,int speedai);


/*
 * @use 电机停止供电
 */
void Stop_car(void);






#endif
