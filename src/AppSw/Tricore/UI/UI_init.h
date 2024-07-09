
#ifndef _UI_VAR_CAM_
#define _UI_VAR_CAM_



typedef struct
{
      unsigned char p;

      unsigned char i;

      unsigned char d;

} PID_C;

typedef struct
{
      unsigned short p;

      unsigned short i;

      unsigned short d;

} PID_ENC;


typedef struct
{
        PID_ENC Dst;
        PID_C Dcu;

 //       float errcamcs;

        PID_ENC Dian;

        float k_nihe;
        float b_nihe;

        unsigned short D_max;

        unsigned short D_min;

        unsigned short D_middle;


        unsigned short speedslow;

        float speedadd;

        float speedD;

        PID_C Ist;

        short err_istcu;

        _Bool Ring_direction[6];

        unsigned short Ring_sumIN;

        unsigned short Ring_turnover;

        unsigned short Ring_inmax;

        unsigned short Ring_speedin;

        unsigned short Ring_over;

        unsigned short Ramp_delay;

        unsigned char befringindelay[6];

        PID_C Icu;

        unsigned char bangbangplus;
        unsigned char bangbangminus;

        unsigned short bangpluspwm;
        unsigned short bangminuspwm;

        unsigned short speed_camp;

        unsigned char delay_threein0;

        unsigned short delay_threeall;

        _Bool Three_direction[2];

        _Bool Outga_direction;

        unsigned char delay_Out;

        unsigned char delay_In;

        unsigned char delay_threeout0;

        unsigned char delay_outgarmid;

//        float Three_upmin;

//        float Three_sidediff;

        unsigned char errlinebegin;

        unsigned short Three_speedin;

        float speederrp;
        float speederrd;

        unsigned short Dcup;
        unsigned short Dcud;
        unsigned short errcamcust;

        unsigned short angle_Ring[4];
        unsigned short angle_Threein[2];
        unsigned short angle_Threeout[2];
        unsigned short angle_Outgar;
        unsigned short angle_Ingar;

        unsigned short speed_Ring;

        unsigned char speed_finishin;
        unsigned char speed_finished;

        unsigned short Three_speedout;
        unsigned char finished_time;
        unsigned char stop_sum;
        unsigned char Ring_sum8;

        unsigned char delay_threein1;
        unsigned char delay_threeout1;

        unsigned char stop_times;

} CAM_var;


typedef struct
{
        short aim_speed;

        short now_speed;

        short speed_err;

        short speed_err_b;
        short speed_err_bb;

        short give_speed;

        short give_speed_b;

        unsigned char white;

} CAM_run;



typedef struct
{
     unsigned short Inleft;

     unsigned short Invertical;

     unsigned short Inlevelr;

     unsigned short Inlevell;

     unsigned short Inright;

    unsigned short Inleftmax;

    unsigned short Inverticalmax;

    unsigned short Inlevellmax;

    unsigned short Inlevelrmax;

    unsigned short Inrightmax;

    float leftonce;

//    float verticalonce;

    float levelronce;

    float levellonce;

    float rightonce;

    float lefttwice;

    float righttwice;

    float levelrtwice;

    float levelltwice;

    short err;

    short err_b;

    short PID_OUT;


} CAM_Inductance;








typedef struct
{
        float temp0;
        float temp1;
        float temp2;
        float temp;

        float A_Left;
        float K_Left;
        float B_Left;

        float A_Right;
        float K_Right;
        float B_Right;

        float A_Center;
        float K_Center;
        float B_Center;

        float Sum_XY;
        float Sum_XX;
        float Sum_XXX;
        float Sum_XXXX;
        float Sum_XXY;
        float Sum_X;
        float Sum_Y;

        short ValidCnt;

        float Aver_X;
        float Aver_Y;
        float K_yicimid;
        short baohuzhi;




} Fit_line;


typedef struct
{
        unsigned char(*judge_ring)(CAM_Inductance *indu);

        unsigned char(*judge_ramp)(void);

        unsigned char(*judge_three)(void);

        unsigned char(*judge_ringout)(void);

        unsigned char(*judge_finish)(void);


} cam_judge;


typedef struct
{
        void(*ring)(CAM_var * pla ,CAM_Inductance *indu , Fit_line *linec);

        unsigned char (*camp)(CAM_Inductance *ind,CAM_var *pla);

        void(*Three)(CAM_var * pla,CAM_Inductance *indu , Fit_line *linec);

        void(*Out)(void);

        void(*Finished)(void);

}cam_jurun;

typedef struct
{
        void(*IN2steer)(void);
        void(*cam2steer)(void);

} To_Steer;


typedef struct
{
        void(*getimagin)(void);
        void(*getinducrance)(CAM_Inductance *ind);

} Getinfo;


typedef struct
{
   unsigned char(*part_1)(void);
   unsigned char(*part_2)(void);
   unsigned char(*part_3)(void);
   void(*part_4)(void);
   void(*part_5)(CAM_var *pla);
   void(*part_6)(void);
   void(*part_7)(void);
   unsigned char(*part_8)(void);
   unsigned char(*part_9)(void);
   void(*part_10)(Fit_line *li);
   void(*part_11)(Fit_line *li);
   unsigned char(*part_12)(void);
   void(*part_13)(void);

} Subs_proc;



enum PLANNUM
{
    plans0,
    plans1,
    plans2,
    plans3,
};

void Test_SPI(void);







#endif
