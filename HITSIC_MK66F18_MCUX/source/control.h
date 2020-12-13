/*
 * control.h
 *
 *  Created on: 2020年11月27日
 *      Author: 86188
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#include "hitsic_common.h"
#include "inc_stdlib.hpp"

#include "sys_pitmgr.hpp"
#include "sys_extint.hpp"
#include "drv_imu_invensense.hpp"
#include "lib_pidctrl.h"

#include "app_menu.hpp"
#include "sc_ftm.h"
#include "drv_disp_ssd1306_port.hpp"

#include "image.h"
#include "sc_host.h"

#define MAX_Speed 50

void Ctrl_MENU_DataSetUp(void);

extern int aaa;
extern int zebra_stop_flag;
extern int out_protect; //赛道保护
extern float servo_mid;
extern float servo_kp,servo_ki;
extern float servo_pwm;

extern float spdL_kp,spdR_kp;
extern float spdL_ki,spdR_ki;


extern float motor_speed;
extern float spdL_now , spdR_now;
extern float spdL_goal , spdR_goal ;
extern float spdL_Output , spdR_Output;
extern float spdL_K, spdR_K;
extern float spdL_aaa, spdR_aaa;
extern float distanceRL, distanceR, distanceL;

void SERVO_GetPid(void);
void SERVO_Run(void *_userData);
void MOTOR_GetPid(float spdL_goal,float spdR_goal,float spdL_now,float spdR_now);
void MOTOR_Run(void *_userData);

float Spd_Fix(float x);
void Speed_Ctrl(void);

extern float WIFI[5];
void WIFItransport(void);

#endif /* CONTROL_H_ */
