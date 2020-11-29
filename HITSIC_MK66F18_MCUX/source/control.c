/*
 * control.c
 *
 *  Created on: 2020年11月27日
 *      Author: 86188
 */

#include "control.h"

/* ******************** 中断函数 ******************** */
float error_1=0;
float error_2=0;

void SERVO_GetPid(void)
{
    float pwm_error=0;
    error_1=get_error();
    pwm_error=servo_kp*error_1+servo_ki*(error_1-error_2);
    servo_pwm=servo_mid+pwm_error;
    if(servo_pwm<6.4)
        servo_pwm=6.4;
    else if(servo_pwm>8.2)
        servo_pwm=8.2;

    error_2=error_1;
};
void SERVO_Run(void *_userData)
{
    SCFTM_PWM_ChangeHiRes(FTM3,kFTM_Chnl_7,50,servo_pwm);
}

void MOTOR_Run(void *_userData)
{
    SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_0,20000,spdR_Output);//右轮正转
    SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_1,20000,0);
    SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_2,20000,spdL_Output);//左轮正转
    SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_3,20000,0);
    if(zebra_flag == 1 && line_type == 4)//斑马线停车
    {
        spdR_Output = 0;
        spdL_Output = 0;
    }
//    if(get_error()==0)
//    {
//        SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_0,20000,0);
//
//        SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_2,20000,0);
//    }
    Speed_Ctrl();
}

float spdL_error , spdR_error;
float spdL_error1 = 0.0, spdR_error1 = 0.0;
float spdL_error2 = 0.0, spdR_error2 = 0.0;
void MOTOR_GetPid(float spdL_goal,float spdR_goal,float spdL_now,float spdR_now)
{
    float spdL_error=0 , spdR_error=0;
    spdL_error1=spdL_goal - spdL_now;
    spdR_error1=spdR_goal - spdR_now;
    spdL_error = spdL_kp * (spdL_error1 - spdL_error2) + spdL_ki * spdL_error1;
    spdR_error = spdR_kp * (spdR_error1 - spdR_error2) + spdR_ki * spdR_error1;

    spdL_error2=spdL_error1;
    spdR_error2=spdR_error1;
}

/* ******************** 速度环 ******************** */

pidCtrl_t spdL_Pid;
pidCtrl_t spdR_Pid;

float spdL_now = 0.0f, spdR_now = 0.0f;

float spdL_Output = 20; ///< 速度环输出
float spdR_Output = 20;
float spdL_goal = 0.0, spdR_goal = 0.0;
float spdL_K = 0.0, spdR_K = 0.0;
float motor_speed = 40;
float spdL_aaa=1.0, spdR_aaa=1.0;
void Speed_Ctrl(void)   ///< 速度环主函数
{
    spdR_now = -((float)SCFTM_GetSpeed(FTM2))*spdR_aaa;
    SCFTM_ClearSpeed(FTM2);
    spdL_now = ((float)SCFTM_GetSpeed(FTM1))*spdL_aaa;
    SCFTM_ClearSpeed(FTM1);
//    if(servo_pwm-servo_mid > 0.15)
//    {
//        spdL_goal = motor_speed*(1-spdL_K*CTRL_SpdFix(servo_pwm-servo_mid));
//        spdR_goal = motor_speed;
//    }
//    else if(servo_pwm-servo_mid < -0.15)
//    {
//        spdL_goal = motor_speed;
//        spdR_goal = motor_speed*(1-spdR_K*CTRL_SpdFix(servo_pwm-servo_mid));
//    }
//    else
//    {
//        spdL_goal = motor_speed;
//        spdR_goal = motor_speed;
//    }

    spdL_goal = motor_speed;
    spdR_goal = motor_speed;
    MOTOR_GetPid(spdL_goal,spdR_goal,spdL_now,spdR_now);
    spdL_Output += spdL_error;
    spdR_Output += spdR_error;

    if(spdL_Output > MAX_Speed)
        spdL_Output = MAX_Speed;
    if(spdL_Output < -MAX_Speed)
        spdL_Output = -MAX_Speed;
    if(spdR_Output > MAX_Speed)
        spdR_Output = MAX_Speed;
    if(spdR_Output < -MAX_Speed)
        spdR_Output = -MAX_Speed;

}

float WIFI[5];
void WIFItransport(void)
{
    WIFI[0] = motor_speed;
    WIFI[1] = spdR_now;
    WIFI[2] = spdL_now;
    WIFI[3] = ((float)SCFTM_GetSpeed(FTM2));
    WIFI[4] = ((float)SCFTM_GetSpeed(FTM1));
    SCHOST_VarUpload(WIFI, 5);//上位机
}

float CTRL_SpdFix(float x)
{
    //添加拟合函数
}


