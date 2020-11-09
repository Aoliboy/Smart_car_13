/**
 * Copyright 2018 - 2020 HITSIC
 * All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*
 * sc_ftm.h
 *
 *  Created on: 2020年10月27日
 *      Author: 孙唯
 */

#ifndef SC_FTM_H_
#define SC_FTM_H_

#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_ftm.h"
#include "clock_config.h"
#include "pin_mux.h"


/**********************************************************************************************************************
*  @brief      ftm设定占空比      用于PWM模式
*  @param      base            ftm模块选择
*  @param      chnlNumber      ftm通道选择
*  @param      dutyFre         ftm频率
*  @return     void
*  @since      v1.0
*  Sample usage:          SCFTM_PWM_Change(FTM0,kFTM_Chnl_1,20000,10U);
**********************************************************************************************************************/
void SCFTM_PWM_Change(FTM_Type *base, ftm_chnl_t chnlNumber, uint32_t dutyFreq, uint8_t dutyCycle);

/**********************************************************************************************************************
*  @brief      ftm设定占空比(高精度版本)      用于PWM模式
*  @param      base            ftm模块选择
*  @param      chnlNumber      ftm通道选择
*  @param      dutyFre         ftm频率
*  @return     void
*  @since      v1.0
*  Sample usage:          SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_1,20000,10.5);
**********************************************************************************************************************/
void SCFTM_PWM_ChangeHiRes(FTM_Type *base, ftm_chnl_t chnlNumber, uint32_t dutyFreq, float dutyCycle);

/**********************************************************************************************************************
*  @brief      ftm获取编码器计数值
*  @param      base            ftm模块选择
*  @return     int16_t         编码器计数值
*  @since      v1.0
*  Sample usage:          SCFTM_GetSpeed(FTM1);
**********************************************************************************************************************/
int16_t SCFTM_GetSpeed(FTM_Type *base);

/**********************************************************************************************************************
*  @brief      ftm清除编码器计数值
*  @param      base            ftm模块选择
*  @return     void
*  @since      v1.0
*  Sample usage:          SCFTM_ClearSpeed(FTM1);
**********************************************************************************************************************/
void SCFTM_ClearSpeed(FTM_Type *base);



#endif /* SC_FTM_H_ */
