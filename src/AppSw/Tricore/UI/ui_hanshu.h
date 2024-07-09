#ifndef _UI_HANSHU_H_
#define _UI_HANSHU_H_

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




#define key_in 0x01
#define key_out 0x02
#define key_up 0x04
#define key_down 0x08
#define key_left 0x10
#define key_right 0x20


#define key_in_IO       P20_13
#define key_out_IO      P15_5
#define key_up_IO       P15_3
#define key_down_IO     P15_0
#define key_left_IO     P15_2
#define key_right_IO    P15_4


uint8 key_scan(uint8 KEY);
void OLED_Write68(unsigned char x,unsigned char y, char ch[] , _Bool mode);


#endif
