
/****************�ṹ���ļ�****************************/

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
  No_road,      //���س���ܵ����޷�У����ͣ��
  Out_of_way,   //�Ѿ�����ܵ�
  Lose_left,    //����
  Lose_right,
  Cross,        //ʮ�ֽ���
  Straight,//ֱ��
  Curve, //���
  Ramp,//����
  LeftRing,//��Բ��
  RightRing,//����Բ��
  LeftOutRing,//����Բ��
  RightOutRing,//��Բ��
  Three_L,
//  Three_f_cu,
  Three_R,
//  Three_s_cu,
  Finish,//����־
  OutGarage,//�����־
  InGarage

 //Ϊ�˳��򷽱����õģ���ʾ��Ҫ����һ�����ж�
}Road;


typedef struct 
{

    unsigned long duty_duo;
    unsigned long duty_dian;

    short L1_real; //��45
    short L2_real; //���
    short L3_real; //����
    short L4_real; //����
    short L5_real; //�Һ�
    short L6_real; //��45

    unsigned short L1_max;//��⵽�����ֵ
    unsigned short L2_max;
    unsigned short L3_max;
    unsigned short L4_max;
    unsigned short L5_max;
    unsigned short L6_max;

    float L1_once;//һ�ι�һ����
    float L2_once;
    float L3_once;
    float L4_once;
    float L5_once;
    float L6_once;

    float L1_twice; //���ι�һ����
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
