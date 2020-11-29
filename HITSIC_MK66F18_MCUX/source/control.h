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

static float servo_mid=7.45;
static float servo_kp=0.015,servo_ki=0.01;
static float servo_pwm=7.45;

extern float motor_speed;

extern float spdL_now , spdR_now;
extern float spdL_error , spdR_Rerror;
extern float spdL_Output , spdR_Output;
static float spdL_kp=0,spdR_kp=0;
static float spdL_ki=10,spdR_ki=10;
extern float spdL_K, spdR_K;
extern float spdL_aaa, spdR_aaa;
extern float distanceRL, distanceR, distanceL;

void SERVO_GetPid(void);
void SERVO_Run(void *_userData);
void MOTOR_GetPid(float spdL_goal,float spdR_goal,float spdL_now,float spdR_now);
void MOTOR_Run(void *_userData);

extern float WIFI[5];
void WIFItransport(void);

float CTRL_SpdFix(float x);
void Speed_Ctrl(void);
float PIDCTRL_DeltaPIGain(pidCtrl_t *_pid);

#endif /* CONTROL_H_ */
