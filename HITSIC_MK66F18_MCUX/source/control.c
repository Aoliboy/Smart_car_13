/*
 * control.c
 *
 *  Created on: 2020年11月27日
 *      Author: 86188
 */

#include "control.h"
int aaa =0;
int zebra_stop_flag=0;
int out_protect = 0;//赛道保护标志
/* ******************** 菜单函数 ******************** */
void Ctrl_MENU_DataSetUp(void)
{
    MENU_ListInsert(menu_menuRoot, MENU_ItemConstruct(nullType, NULL, " ", 0, 0));
    //TODO: 在这里添加子菜单和菜单项
    static menu_list_t *Testlist;
    //添加舵机调参菜单
    Testlist = MENU_ListConstruct("Servo", 5, menu_menuRoot);
    MENU_ListInsert(menu_menuRoot, MENU_ItemConstruct(menuType, Testlist, "Servo", 0, 0));
    MENU_ListInsert(Testlist, MENU_ItemConstruct(varfType, &servo_kp, "kp", 10, menuItem_data_global));
    MENU_ListInsert(Testlist, MENU_ItemConstruct(varfType, &servo_ki, "kd", 11, menuItem_data_global));
    MENU_ListInsert(Testlist, MENU_ItemConstruct(varfType, &servo_mid, "mid", 12, menuItem_data_global));
    //MENU_ListInsert(Testlist, MENU_ItemConstruct(varfType, &servo_pwm, "servo_pwm", 13, menuItem_data_global));
    //添加图像调参菜单
    Testlist = MENU_ListConstruct("Image", 4, menu_menuRoot);
    MENU_ListInsert(menu_menuRoot, MENU_ItemConstruct(menuType, Testlist, "Image", 0, 0));
    //MENU_ListInsert(Testlist, MENU_ItemConstruct(variType, &imageTH, "imageTH", 13, menuItem_data_global));
    MENU_ListInsert(Testlist, MENU_ItemConstruct(variType, &threshold, "threshold", 14, menuItem_data_global));
    MENU_ListInsert(Testlist, MENU_ItemConstruct(variType, &foresight, "foresight", 15, menuItem_data_global));
    //添加速度调参菜单
    Testlist = MENU_ListConstruct("Speed", 10, menu_menuRoot);
    MENU_ListInsert(menu_menuRoot, MENU_ItemConstruct(menuType, Testlist, "Speed", 0, 0));
    MENU_ListInsert(Testlist, MENU_ItemConstruct(varfType, &motor_speed, "motor_speed", 16, menuItem_data_global));
    MENU_ListInsert(Testlist, MENU_ItemConstruct(varfType, &spdL_K, "spdL_K", 17, menuItem_data_global));
    MENU_ListInsert(Testlist, MENU_ItemConstruct(varfType, &spdR_K, "spdR_K", 18, menuItem_data_global));
    MENU_ListInsert(Testlist, MENU_ItemConstruct(varfType, &spdL_kp, "spdL.kp", 31,menuItem_data_region));
    MENU_ListInsert(Testlist, MENU_ItemConstruct(varfType, &spdL_ki, "spdL.ki", 32,menuItem_data_region));
    MENU_ListInsert(Testlist, MENU_ItemConstruct(varfType, &spdR_kp, "spdR.kp", 33,menuItem_data_region));
    MENU_ListInsert(Testlist, MENU_ItemConstruct(varfType, &spdR_ki, "spdR.ki", 34,menuItem_data_region));
    MENU_ListInsert(Testlist, MENU_ItemConstruct(varfType, &spdL_aaa, "spdL_aaa", 35,menuItem_data_region));
    MENU_ListInsert(Testlist, MENU_ItemConstruct(varfType, &spdR_aaa, "spdR_aaa", 36,menuItem_data_region));
    //添加编码器显示菜单
    Testlist = MENU_ListConstruct("test", 12, menu_menuRoot);
    MENU_ListInsert(menu_menuRoot, MENU_ItemConstruct(menuType, Testlist, "test", 0, 0));
    MENU_ListInsert(Testlist, MENU_ItemConstruct(varfType, &servo_pwm, "servo_pwm", 0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
    MENU_ListInsert(Testlist, MENU_ItemConstruct(varfType, &spdL_now, "spdL_now", 0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
    MENU_ListInsert(Testlist, MENU_ItemConstruct(varfType, &spdR_now, "spdR_now", 0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
    MENU_ListInsert(Testlist, MENU_ItemConstruct(varfType, &spdL_Output, "spdL_Output", 0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
    MENU_ListInsert(Testlist, MENU_ItemConstruct(varfType, &spdR_Output, "spdR_Output", 0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
    MENU_ListInsert(Testlist, MENU_ItemConstruct(varfType, &spdL_goal, "spdL_goal", 0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
    MENU_ListInsert(Testlist, MENU_ItemConstruct(varfType, &spdR_goal, "spdR_goal", 0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
    MENU_ListInsert(Testlist, MENU_ItemConstruct(variType, &zebra_flag, "zebra_flag", 0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
    MENU_ListInsert(Testlist, MENU_ItemConstruct(variType, &zebra_stop_flag, "zebra_stop", 0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
    MENU_ListInsert(Testlist, MENU_ItemConstruct(variType, &out_protect, "out_protect", 0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
    MENU_ListInsert(Testlist, MENU_ItemConstruct(variType, &aaa, "aaa", 0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
}

/* ******************** 中断函数 ******************** */
float error_1=0;
float error_2=0;
float servo_mid=7.3;
float servo_kp=0.015,servo_ki=0.01;
float servo_pwm=7.3;
void SERVO_GetPid(void)
{
    float pwm_error=0;
    error_1=er;
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
    if(zebra_stop_flag==1 )//斑马线停车
    {
        SCFTM_PWM_ChangeHiRes(FTM3,kFTM_Chnl_7,50,7.3);
    }
}

void MOTOR_Run(void *_userData)
{
    aaa++;
    if(aaa>1000)
    {
        Speed_Ctrl();//速度环
        if(spdR_Output > 0)//反转保护
        {
            SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_0,20000,spdR_Output);//右轮正转
            SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_1,20000,0);
        }
        else
        {
            SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_0,20000,0);
            SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_1,20000,-spdR_Output);
        }
        if(spdL_Output > 0)
        {
            SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_2,20000,spdL_Output);//左轮正转
            SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_3,20000,0);
        }
        else
        {
            SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_2,20000,0);
            SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_3,20000,-spdL_Output);
        }

        spdR_now = -((float)SCFTM_GetSpeed(FTM2))*spdR_aaa;
        SCFTM_ClearSpeed(FTM2);//上传路程时需注释
        spdL_now = ((float)SCFTM_GetSpeed(FTM1))*spdL_aaa;
        SCFTM_ClearSpeed(FTM1);//上传路程时需注释

        if(zebra_stop_flag==1 )//斑马线停车
        {
            motor_speed = 0;
        }

        if(er == 255)//出赛道保护
        {
            SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_0,20000,0);
            SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_1,20000,0);
            SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_2,20000,0);
            SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_3,20000,0);
            out_protect = 1;
        }
    }
    if(aaa>2000)
    {
        aaa=2000;
    }
}

float spdL_kp=0,spdR_kp=0;
float spdL_ki=0,spdR_ki=0;
float spdL_error=0 , spdR_error=0;
float spdL_error1 = 0, spdR_error1 = 0;
float spdL_error2 = 0, spdR_error2 = 0;
void MOTOR_GetPid(float spdL_goal,float spdR_goal,float spdL_now,float spdR_now)
{
    spdL_error1=spdL_goal - spdL_now;
    spdR_error1=spdR_goal - spdR_now;
    spdL_error = spdL_kp * (spdL_error1 - spdL_error2) + spdL_ki * spdL_error1;
    spdR_error = spdR_kp * (spdR_error1 - spdR_error2) + spdR_ki * spdR_error1;

    spdL_error2=spdL_error1;
    spdR_error2=spdR_error1;
}

/* ******************** 速度环 ******************** */
float motor_speed=30;
float spdL_now=0 , spdR_now=0;
float spdL_goal=0 , spdR_goal=0 ;
float spdL_Output=0 , spdR_Output=0;
float spdL_K = 0.0, spdR_K = 0.0;
float spdL_aaa=1.0, spdR_aaa=1.0;
float distanceRL, distanceR, distanceL;
void Speed_Ctrl(void)   ///< 速度环主函数
{
    if(servo_pwm-servo_mid > 0.2)//差速
    {
        spdL_goal = motor_speed*(1-spdL_K*Spd_Fix(servo_pwm));
        spdR_goal = motor_speed*(1+spdL_K*Spd_Fix(servo_pwm));
    }
    else if(servo_pwm-servo_mid < -0.2)
    {
        spdL_goal = motor_speed*(1+spdR_K*Spd_Fix(servo_pwm));
        spdR_goal = motor_speed*(1-spdR_K*Spd_Fix(servo_pwm));
    }
    else
    {
        spdL_goal = motor_speed;
        spdR_goal = motor_speed;
    }
//    if(servo_pwm-servo_mid > 0.2 || servo_pwm-servo_mid < -0.2)//差速
//    {
//        spdL_goal = motor_speed*SpdL_Fix(servo_pwm);
//        spdR_goal = motor_speed*SpdR_Fix(servo_pwm);
//    }
//    else
//    {
//        spdL_goal = motor_speed;
//        spdR_goal = motor_speed;
//    }


    spdL_Output= spdL_goal;
    spdR_Output= spdR_goal;

    MOTOR_GetPid(spdL_goal,spdR_goal,spdL_now,spdR_now);
    spdL_Output += spdL_error;
    spdR_Output += spdR_error;
    if(spdL_Output > 75)//电机限幅
       {
           spdL_Output = 75;
       }
       if(spdL_Output < -75)
       {
           spdL_Output = -75;
       }
       if(spdR_Output > 75)
       {
           spdR_Output = 75;
       }
       if(spdR_Output < -75)
       {
           spdR_Output = -75;
       }
}

//float SpdL_Fix(float x)
//{
//        return  (-0.0725*x*x*x + 1.5931*x*x - 11.928*x + 31.358);
//}
//
//float SpdR_Fix(float x)
//{
//        return  (0.0725*x*x*x - 1.5931*x*x + 11.928*x - 29.358);
//}

float Spd_Fix(float x)
{
    if(x > 7.35)
    {
        return (0.0563*x*x*x - 1.2396*x*x + 9.3495*x - 24.115);
    }
    else if(x < 7.25)
    {
        return  (-0.0563*x*x*x + 1.2396*x*x - 9.3495*x + 24.115);
    }
    return 0;
}

/* ******************** WIFI传输 ******************** */
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




