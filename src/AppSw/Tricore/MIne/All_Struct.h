
/****************结构体文件****************************/

#ifndef _ALL_STRUCT_H_
#define _ALL_STRUCT_H_




typedef struct
{
    float Err1;
    float Err2;
    float Err3;

    float Err;
    float Err_sum[150];
    float Err_b;
    float Errsum;

    unsigned long p;
    unsigned long i;
    unsigned long d;

    float PID_OUT;

}PID_I;

typedef struct
{
     short p;
     short i;
     short d;

     float err;
     float err_b;
     float err_bb;
     float err_sum[100];
     float errsum;


}PID_E;


typedef enum
{
  Need_judge,
  No_road,      //严重冲出跑道且无法校正则停车
  Out_of_way,   //已经冲出跑道
  Lose_left,    //丢线
  Lose_right,
  Cross,        //十字交叉
  Straight,//直道
  Curve, //弯道
  Ramp,//上坡
  LeftRing,//左圆环
  RightRing,//出右圆环
  LeftOutRing,//出左圆环
  RightOutRing,//右圆环
  Three_L,
//  Three_f_cu,
  Three_R,
//  Three_s_cu,
  Finish,//入库标志
  OutGarage,//出库标志
  InGarage

 //为了程序方便设置的，表示需要更进一步的判断
}Road;


typedef struct 
{

    unsigned long duty_duo;
    unsigned long duty_dian;

    short L1_real; //左45
    short L2_real; //左横
    short L3_real; //左竖
    short L4_real; //右竖
    short L5_real; //右横
    short L6_real; //右45

    unsigned short L1_max;//检测到的最大值
    unsigned short L2_max;
    unsigned short L3_max;
    unsigned short L4_max;
    unsigned short L5_max;
    unsigned short L6_max;

    float L1_once;//一次归一化后
    float L2_once;
    float L3_once;
    float L4_once;
    float L5_once;
    float L6_once;

    float L1_twice; //两次归一化后
    float L2_twice;
    float L3_twice;
    float L4_twice;
    float L5_twice;
    float L6_twice;

} DG_state;

typedef struct
{
        int speedbase;
        int speedaim;
} Dian;



#endif
