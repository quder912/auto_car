#ifndef _ENCODER_H_
#define _ENCODER_H_

#include "hanshu.h"
#include "allinit.h"
#include "All_Struct.h"




//extern unsigned int i_enc;



/*
 * @use ��ȡ��������ֵ��ʾ������������
 */
signed short GetENC(void);



/*
 * @use ��err����pid���㲢����
 */
//int ENC_pid(signed short enc,Dian *speed,PID_E *pid);





/*
 * @use ��speed ͨ��pid������
 */
//void Setspeed(Dian *speed,PID_E *pid);




/*
 * @use �ٶȸ���"Dian"�ṹ��
 *
 */
void Change_speed(Dian *speed , int speedba ,int speedai);


/*
 * @use ���ֹͣ����
 */
void Stop_car(void);






#endif
