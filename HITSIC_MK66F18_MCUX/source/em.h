#ifndef DEV_EM_H_
#define DEV_EM_H_

#include "hitsic_common.h"
#include "inc_stdlib.hpp"

#include "sys_pitmgr.hpp"
#include "sys_extint.hpp"
#include "drv_imu_invensense.hpp"
#include "lib_pidctrl.h"

#include "app_menu.hpp"
#include "sc_ftm.h"
#include "sc_adc.h"
#include "drv_disp_ssd1306_port.hpp"

#define SampleTimes 25
#define ChannelTimes 8
extern float LV[ChannelTimes];
extern float AD[ChannelTimes];

void EM_LVSample(void);
void EM_LVGetVal(void);
void swap(uint32_t * a, uint32_t * b);
float EM_ErrorUpdate(void);

#endif /* DEV_EM_HPP_ */
