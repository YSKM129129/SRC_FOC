/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          :  drv_DRV835X.c
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
#include "DRV8353.h"
#include "gpio.h"
 
/* Private macro -------------------------------------------------------------*/
extern SPI_HandleTypeDef        hspi3;
 
/* Private define ------------------------------------------------------------*/
#define TIME_OUT                100
#define DEFAULT_GAIN            10
#define DRV835X_SPI_Handle      hspi3
 
// DRV8353 SPI CS PIN 
#define DRV835X_CS_EN           HAL_GPIO_WritePin(DRV_CS_GPIO_Port,DRV_CS_Pin,GPIO_PIN_RESET)
#define DRV835X_CS_DIS          HAL_GPIO_WritePin(DRV_CS_GPIO_Port,DRV_CS_Pin,GPIO_PIN_SET)
 
// DRV8353 ENABLE PIN 
/*
    Gate driver enable. When this pin is logic low the device goes to a low power sleep mode. 
    An 8 to 40-µs low pulse can be used to reset fault conditions.
*/
#define DRV835X_ENABLE_LOW      HAL_GPIO_WritePin(DRV_ENBLE_GPIO_Port,DRV_ENBLE_Pin,GPIO_PIN_RESET)
#define DRV835X_ENABLE_HIGH     HAL_GPIO_WritePin(DRV_ENBLE_GPIO_Port,DRV_ENBLE_Pin,GPIO_PIN_SET)
 
// DRV8353 PWML PIN: INLA INLB INLC 
/*
   Low-side gate driver control input. This pin controls the output of the low-side gate driver.
*/
#define DRV835X_PWML_LOW        HAL_GPIO_WritePin(DRV_cotr_GPIO_Port,DRV_cotr_Pin,GPIO_PIN_RESET)
#define DRV835X_PWML_HIGH       HAL_GPIO_WritePin(DRV_cotr_GPIO_Port,DRV_cotr_Pin,GPIO_PIN_SET)
 
/* Private variables ---------------------------------------------------------*/
Stru_DRV835X_Status stru_DRV835X_Status;
Stru_DRV835X stru_DRV8353Obj;
 
StruDRV835XCfgPara stru_config = 
{
    // Driver Control Register (address = 0x02h)
   .PWM_MODE = PWM_MODE_3X,
    
    // CSA Control Register (DRV8353 and DRV8353R Only) (address = 0x06h)
   .SEN_LVL = SEN_LVL_0_25,   //  00b = Sense OCP 0.25 V
   .CSA_GAIN = CSA_GAIN_10,   //  01b = 10-V/V shunt amplifier gain
   .VREF_DIV = VREF_DIV_2,    //  1b = Sense amplifier reference voltage is VREF divided by 2
    
    // OCP Control Register (address = 0x05h)
   .VDS_LVL =  VDS_LVL_0_94,
   .OCP_DEG =  OCP_DEG_6US,
   .OCP_MODE = OCP_REPORT,
   .DEAD_TIME = DEADTIME_400NS,
    
   //Gate Drive HS Register (address = 0x03h)
   .IDRIVEP_HS = IDRIVEP_HS_1000MA,
   .IDRIVEN_HS = IDRIVEN_HS_2000MA,
   .LOCK = LOCK_OFF,
    
   // Gate Drive LS Register (address = 0x04h) 
   .IDRIVEN_LS = IDRIVEN_LS_2000MA,
   .IDRIVEP_LS = IDRIVEP_LS_1000MA,
   .TDRIVE = TDRIVE_4000NS,
   .CBC = PWM_GIVER_ENABLE,  // 1b = For VDS_OCP and SEN_OCP, the fault is cleared when
                             // a new PWM input is given or after tRETRY
};
 
/* Private function prototypes -----------------------------------------------*/
static uint16_t read_reg(uint16_t address);
static uint16_t write_reg(uint16_t address, uint16_t data);
 
 
void DRV835X_updateCfgPara( void )
{
    uint16_t data;
 
    stru_DRV8353Obj.drvCtrl_obj.data = read_reg( DCR );
    stru_DRV8353Obj.drvCsa_obj.data = read_reg( CSACR );
    stru_DRV8353Obj.drvCfg_obj.data = read_reg( DFGCR );
 
    stru_DRV8353Obj.drvGateHS_obj.data = read_reg( HSR );
    stru_DRV8353Obj.drvGateLS_obj.data = read_reg( LSR );
    stru_DRV8353Obj.drvOcp_obj.data = read_reg( OCPCR );
 
    stru_DRV8353Obj.faultStatusReg1_obj.data = read_reg( FSR1 );
    stru_DRV8353Obj.faultStatusReg2_obj.data = read_reg( FSR2 );
 
    // Driver Control Register (address = 0x02h)
    stru_DRV8353Obj.drvCtrl_obj.ctrlRegObj.PWM_MODE  = stru_config.PWM_MODE;
    data = stru_DRV8353Obj.drvCtrl_obj.data;
    write_reg( DCR, data);
 
    //Gate Drive HS Register (address = 0x03h)
    stru_DRV8353Obj.drvGateHS_obj.gateHSRegObj.IDRIVEP_HS = stru_config.IDRIVEP_HS;
    stru_DRV8353Obj.drvGateHS_obj.gateHSRegObj.IDRIVEN_HS = stru_config.IDRIVEN_HS;
    stru_DRV8353Obj.drvGateHS_obj.gateHSRegObj.LOCK = stru_config.LOCK;
    data = stru_DRV8353Obj.drvGateHS_obj.data;
    write_reg( HSR, data);
 
    // Gate Drive LS Register (address = 0x04h) 
    stru_DRV8353Obj.drvGateLS_obj.gateLSRegObj.IDRIVEN_LS = stru_config.IDRIVEN_LS;
    stru_DRV8353Obj.drvGateLS_obj.gateLSRegObj.IDRIVEP_LS = stru_config.IDRIVEP_LS;
    stru_DRV8353Obj.drvGateLS_obj.gateLSRegObj.TDRIVE = stru_config.TDRIVE;
    stru_DRV8353Obj.drvGateLS_obj.gateLSRegObj.CBC = stru_config.CBC;
    data = stru_DRV8353Obj.drvGateLS_obj.data;
    write_reg( LSR, data);
 
    // OCP Control Register (address = 0x05h)
    stru_DRV8353Obj.drvOcp_obj.ocpObj.VDS_LVL =  stru_config.VDS_LVL;
    stru_DRV8353Obj.drvOcp_obj.ocpObj.OCP_DEG =  stru_config.OCP_DEG;
    stru_DRV8353Obj.drvOcp_obj.ocpObj.OCP_MODE = stru_config.OCP_MODE;
    stru_DRV8353Obj.drvOcp_obj.ocpObj.DEAD_TIME = stru_config.DEAD_TIME;
    data = stru_DRV8353Obj.drvOcp_obj.data;
    write_reg( OCPCR, data);
 
    // CSA Control Register (DRV8353 and DRV8353R Only) (address = 0x06h)
    stru_DRV8353Obj.drvCsa_obj.csaObj.SEN_LVL  = stru_config.SEN_LVL;
    stru_DRV8353Obj.drvCsa_obj.csaObj.CSA_GAIN = stru_config.CSA_GAIN;
    stru_DRV8353Obj.drvCsa_obj.csaObj.VREF_DIV = stru_config.VREF_DIV;
    data = stru_DRV8353Obj.drvCtrl_obj.data;
    write_reg( CSACR, data);
}
 
 
void DRV835X_Init( void )
{
    DRV835X_ENABLE_LOW;
    HAL_Delay(100);
    DRV835X_ENABLE_HIGH;
    HAL_Delay(100);
    
    // SET PWML to low
    DRV835X_PWML_LOW;
    HAL_Delay(200);
    
    DRV835X_updateCfgPara();
}
 
 
void DRV835X_read_FaultStatusReg1(void)
{
    stru_DRV8353Obj.faultStatusReg1_obj.data = read_reg( FSR1 );
}
 
void DRV835X_read_FaultStatusReg2(void)
{
    stru_DRV8353Obj.faultStatusReg2_obj.data = read_reg( FSR2 );
}
 
 
static uint16_t read_reg(uint16_t address)
{
    uint16_t data;
    Input_WrReg stru_Input_WrRegObj;
    
    stru_Input_WrRegObj.inputRegObj.WR =  R_MODE;
    stru_Input_WrRegObj.inputRegObj.ADDRESS = address;
    data = stru_Input_WrRegObj.data;
    
    DRV835X_CS_EN;
    HAL_SPI_Transmit(&DRV835X_SPI_Handle, (uint8_t *)&data, 1,TIME_OUT);
    DRV835X_CS_DIS;
    HAL_Delay(1);
    
    DRV835X_CS_EN;
    HAL_SPI_Receive(&DRV835X_SPI_Handle, (uint8_t *)&data, 1, TIME_OUT);
    DRV835X_CS_DIS;
    HAL_Delay(1);
    
    return (data & 0x7FF);
}
 
static uint16_t write_reg(uint16_t address, uint16_t data)
{
    Input_WrReg stru_Input_WrRegObj;
    
    stru_Input_WrRegObj.inputRegObj.WR =  W_MODE;
    stru_Input_WrRegObj.inputRegObj.ADDRESS = address;
    stru_Input_WrRegObj.inputRegObj.DATA = data;
    
    data = stru_Input_WrRegObj.data;
    do
    {
        DRV835X_CS_EN;
        HAL_SPI_Transmit(&DRV835X_SPI_Handle, (uint8_t *)&data, 1, TIME_OUT);
        DRV835X_CS_DIS;
        HAL_Delay(1);
    }while (read_reg(address) != (data & 0x7FF));
    
    return 0;
}
 
 
/* End of this file */
 