#ifndef _HANSHU_H_
#define _HANSHU_H_

#include "LQ_ADC.h"
#include "LQ_GPIO.h"
#include "LQ_STM.h"
#include "LQ_GTM.h"
#include "math.h"
#include "LQ_OLED096.h"
#include "stdbool.h"
#include "stdio.h"
#include "LQ_UART.h"
#include "LQ_GPT12_ENC.h"
#include "allinit.h"
#include "All_Struct.h"
#include "ENcoder.h"





/*
 * @note 只支持点按
 */
unsigned int scan_key(GPIO_Name_t Pin);



/*
 * @use 判断斑马线
 */
uint8 Garage_check(GPIO_Name_t Pin1 ,GPIO_Name_t Pin2);


/*
 * @return 1/0
 */
uint8 num2char(float num, char *array);




/*
 * @use 蜂鸣器响1
 */
void buzzer(_Bool bu);




#endif
