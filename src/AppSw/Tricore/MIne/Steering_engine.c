#include "Steering_engine.h"


/*
 * @use  转向固定角度
 *
 * @note 已设最大最小限
 */
void turnTO(int dir)
{
  //  dir=(dir<duty_duo_min)? duty_duo_min : ( (dir>=duty_duo_max) ? duty_duo_max:dir);
    ATOM_PWM_SetDuty(IfxGtm_ATOM0_1_TOUT31_P33_9_OUT, dir, 200);
}

