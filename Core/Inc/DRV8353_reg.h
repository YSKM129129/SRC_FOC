/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          :  drv_DRV835X.h
 * Description        :  DRV835X driver 
 ******************************************************************************
 * @attention
 *
* COPYRIGHT:    Copyright (c) 2025
* CREATED BY:   ming fei.tang
* DATE:         January 04th, 2025
 ******************************************************************************
 */
/* USER CODE END Header */
 
/* Includes ------------------------------------------------------------------*/
#ifndef __DRV_DRV835X_REG_H
#define __DRV_DRV835X_REG_H
 
#ifdef __cplusplus
extern "C"
{
#endif
 
#include "main.h"
    
/// Registers ///
#define FSR1             0x0     /// Fault Status Register 1
#define FSR2             0x1     /// Fault Status Register 2
#define DCR              0x2     /// Drive Control Register
#define HSR              0x3     /// Gate Drive HS Register 
#define LSR              0x4     /// Gate Drive LS Register  
#define OCPCR            0x5     /// OCP Control Register    
#define CSACR            0x6     /// CSA Control Register    
#define DFGCR            0x7     /// Driver Configuration Register 
 
/// Drive Control Fields ///
#define DIS_CPUV_EN         0x0     /// Charge pump UVLO fault
#define DIS_CPUV_DIS        0x1
#define DIS_GDF_EN          0x0     /// Gate drive fauilt
#define DIS_GDF_DIS         0x1
#define OTW_REP_EN          0x1     /// Over temp warning reported on nFAULT/FAULT bit
#define OTW_REP_DIS         0x0
 
#define PWM_MODE_6X         0x0     /// PWM Input Modes
#define PWM_MODE_3X         0x1
#define PWM_MODE_1X         0x2
#define PWM_MODE_IND        0x3
 
#define PWM_1X_COM_SYNC     0x0     /// 1x PWM Mode synchronou rectification
#define PWM_1X_COM_ASYNC    0x1
 
#define PWM_1X_DIR_0        0x0     /// In 1x PWM mode this bit is ORed with the INHC (DIR) input
#define PWM_1X_DIR_1        0x1
 
/// Gate Drive HS Fields ///
#define LOCK_ON             0x6
#define LOCK_OFF            0x3
 
#define IDRIVEP_HS_50MA     0x0     /// Gate drive high side turn on current (DRV8353F)
#define IDRIVEP_HS_50MA_ALT 0x1
#define IDRIVEP_HS_100MA    0x2
#define IDRIVEP_HS_150MA    0x3
#define IDRIVEP_HS_300MA    0x4
#define IDRIVEP_HS_350MA    0x5
#define IDRIVEP_HS_400MA    0x6
#define IDRIVEP_HS_450MA    0x7
#define IDRIVEP_HS_550MA    0x8
#define IDRIVEP_HS_600MA    0x9
#define IDRIVEP_HS_650MA    0xA
#define IDRIVEP_HS_700MA    0xB
#define IDRIVEP_HS_850MA    0xC
#define IDRIVEP_HS_900MA    0xD
#define IDRIVEP_HS_950MA    0xE
#define IDRIVEP_HS_1000MA   0xF
 
#define IDRIVEN_HS_100MA     0x0     /// High side turn off current (DRV8353F)
#define IDRIVEN_HS_100MA_ALT 0x1
#define IDRIVEN_HS_200MA     0x2
#define IDRIVEN_HS_300MA     0x3
#define IDRIVEN_HS_600MA     0x4
#define IDRIVEN_HS_700MA     0x5
#define IDRIVEN_HS_800MA     0x6
#define IDRIVEN_HS_900MA     0x7
#define IDRIVEN_HS_1100MA    0x8
#define IDRIVEN_HS_1200MA    0x9
#define IDRIVEN_HS_1300MA    0xA
#define IDRIVEN_HS_1400MA    0xB
#define IDRIVEN_HS_1700MA    0xC
#define IDRIVEN_HS_1800MA    0xD
#define IDRIVEN_HS_1900MA    0xE
#define IDRIVEN_HS_2000MA   0xF
 
/// Gate Drive LS Fields : Gate Drive LS Register (address = 0x04h)
#define TDRIVE_500NS        0x0     /// Peak gate-current drive time
#define TDRIVE_1000NS       0x1
#define TDRIVE_2000NS       0x2
#define TDRIVE_4000NS       0x3
 
#define IDRIVEP_LS_50MA     0x0     /// Gate drive low side turn on current (DRV8353F)
#define IDRIVEP_LS_50MA_ALT 0x1
#define IDRIVEP_LS_100MA    0x2
#define IDRIVEP_LS_150MA    0x3
#define IDRIVEP_LS_300MA    0x4
#define IDRIVEP_LS_350MA    0x5
#define IDRIVEP_LS_400MA    0x6
#define IDRIVEP_LS_450MA    0x7
#define IDRIVEP_LS_550MA    0x8
#define IDRIVEP_LS_600MA    0x9
#define IDRIVEP_LS_650MA    0xA
#define IDRIVEP_LS_700MA    0xB
#define IDRIVEP_LS_850MA    0xC
#define IDRIVEP_LS_900MA    0xD
#define IDRIVEP_LS_950MA    0xE
#define IDRIVEP_LS_1000MA   0xF
 
#define IDRIVEN_LS_100MA     0x0     /// Low side turn off current (DRV8353F)
#define IDRIVEN_LS_100MA_ALT 0x1
#define IDRIVEN_LS_200MA     0x2
#define IDRIVEN_LS_300MA     0x3
#define IDRIVEN_LS_600MA     0x4
#define IDRIVEN_LS_700MA     0x5
#define IDRIVEN_LS_800MA     0x6
#define IDRIVEN_LS_900MA     0x7
#define IDRIVEN_LS_1100MA    0x8
#define IDRIVEN_LS_1200MA    0x9
#define IDRIVEN_LS_1300MA    0xA
#define IDRIVEN_LS_1400MA    0xB
#define IDRIVEN_LS_1700MA    0xC
#define IDRIVEN_LS_1800MA    0xD
#define IDRIVEN_LS_1900MA    0xE
#define IDRIVEN_LS_2000MA   0xF
 
#define  PWM_GIVER_ENABLE    0x1
#define  PWM_GIVER_DISABLE   0x0
 
 
/// OCP Control Fields ///
#define TRETRY_8MS          0x0     /// VDS OCP and SEN OCP retry time (DRV8353F)
#define TRETRY_50US         0x1
 
#define DEADTIME_50NS       0x0     /// Deadtime
#define DEADTIME_100NS      0x1
#define DEADTIME_200NS      0x2
#define DEADTIME_400NS      0x3
 
#define OCP_LATCH           0x0     /// OCP Mode
#define OCP_RETRY           0x1
#define OCP_REPORT          0x2
#define OCP_NONE            0x3
 
#define OCP_DEG_1US         0x0     /// OCP Deglitch Time (DRV8353F)
#define OCP_DEG_2US         0x1
#define OCP_DEG_4US         0x2
#define OCP_DEG_8US         0x3
 
#define VDS_LVL_0_06        0x0
#define VDS_LVL_0_07        0x1
#define VDS_LVL_0_08        0x2
#define VDS_LVL_0_09        0x3
#define VDS_LVL_0_1         0x4
#define VDS_LVL_0_2         0x5
#define VDS_LVL_0_3         0x6
#define VDS_LVL_0_4         0x7
#define VDS_LVL_0_5         0x8
#define VDS_LVL_0_6         0x9
#define VDS_LVL_0_7         0xA
#define VDS_LVL_0_8         0xB
#define VDS_LVL_0_9         0xC
#define VDS_LVL_1_0         0xD
#define VDS_LVL_1_5         0xE
#define VDS_LVL_2_0         0xF
 
/// CSA Control Fields ///
#define CSA_FET_SP          0x0     /// Current sense amplifier positive input
#define CSA_FET_SH          0x1
 
#define VREF_DIV_1          0x0     /// Amplifier reference voltage is VREV/1
#define VREF_DIV_2          0x1     /// Amplifier reference voltage is VREV/2
 
#define CSA_GAIN_5          0x0     /// Current sensor gain
#define CSA_GAIN_10         0x1
#define CSA_GAIN_20         0x2
#define CSA_GAIN_40         0x3
 
#define DIS_SEN_EN          0x0     /// Overcurrent Fault
#define DIS_SEN_DIS         0x1
 
#define SEN_LVL_0_25        0x0     /// Sense OCP voltage level
#define SEN_LVL_0_5         0x1
#define SEN_LVL_0_75        0x2
#define SEN_LVL_1_0         0x3
    
    
    
 
#ifdef __cplusplus
}
#endif
 
 
#endif  /* __DRV_DRV835X_REG_H */
