/**************************************************************************//**
 * @file     LaserDrv.c
 * @version  V1.0
 * @Date     21/01/14
 * @brief    Cremotech LASER Driver Source File
 *
 * @note
 * Copyright (C) 2013 ASU Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include "serial_communication.h"

uint16_t g_RedCurrent = 0x0000;
uint16_t g_GreenCurrent = 0x0000;
uint16_t g_BlueCurrent = 0x0000;

extern struct Projector_parameter  g_projector_para;

#define TPL1401_RLED_I2C_ADDR 0x90
#define TPL1401_GLED_I2C_ADDR 0x92
#define TPL1401_BLED_I2C_ADDR 0x94

#define RETRY_CNT 5
uint8_t TPL1401_WriteI2C_Byte(uint8_t DevAddr, uint8_t RegAddr, uint16_t data)
{
		uint8_t ret, retry_cnt = RETRY_CNT;
		uint8_t d[2];

		d[0] = (data & 0xff00) >> 8 ;
		d[1] = data & 0xff;
	
		while(retry_cnt --) {
			ret = HAL_I2C_Mem_Write(&hi2c2, (uint16_t)DevAddr, (uint16_t)RegAddr, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&d, 2, 300);
			if(ret == HAL_OK) break;
		}
		
		if(ret != HAL_OK)
		{		
			if(TPL1401_RLED_I2C_ADDR == DevAddr)
				printf("TPL1401_WriteI2C_Byte Red 0x%x->0x%x  faild. ret=%d\r\n",RegAddr, data, ret);
			if(TPL1401_GLED_I2C_ADDR == DevAddr)
				printf("TPL1401_WriteI2C_Byte Green 0x%x->0x%x  faild. ret=%d\r\n",RegAddr, data, ret);
			if(TPL1401_BLED_I2C_ADDR == DevAddr)
				printf("TPL1401_WriteI2C_Byte Blue 0x%x->0x%x  faild. ret=%d\r\n",RegAddr, data, ret);			
		}
		return ret;
}

uint16_t TPL1401_ReadI2C_Byte(uint8_t DevAddr, uint8_t RegAddr)
{
	uint8_t ret,retry_cnt=RETRY_CNT;
	uint8_t d[2] = {0};
	
	while(retry_cnt --) {	
		ret = HAL_I2C_Mem_Read(&hi2c2, (uint16_t)DevAddr, (uint16_t)RegAddr, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&d, 2, 300);
		if(ret == HAL_OK) break;
	}
	if(ret != HAL_OK) 
	{
		printf("TPL1401_ReadI2C_Byte 0x%x faild. ret=%d\r\n", RegAddr, ret);
		return 0xffff;
	}
	
	return (d[0] << 8) + d[1];
}

#define MAX_RGB_VAL   	96
#define MIN_RGB_VAL   	10
#define DEFAULT_R_VAL   96
#define DEFAULT_G_VAL   70
#define DEFAULT_B_VAL   70
void A100_SetPwm(uint8_t DevAddr, uint8_t data)
{
	uint16_t reg_data = (MAX_RGB_VAL - data) << 4;

	TPL1401_WriteI2C_Byte(DevAddr, 0xD1, 0x01E0);//0x11E5
	HAL_Delay(1);
	TPL1401_WriteI2C_Byte(DevAddr, 0x21, reg_data);
	HAL_Delay(1);
	TPL1401_WriteI2C_Byte(DevAddr, 0xD3, 0x0010);	
	HAL_Delay(1);
}


uint8_t A100_SetGreenCurrent(uint16_t GreenCurrent)
{
  if(GreenCurrent > MAX_RGB_VAL)
  {
			GreenCurrent  = MAX_RGB_VAL;
  }
  if(GreenCurrent < MIN_RGB_VAL)
  {
			GreenCurrent  = MIN_RGB_VAL;
  }
	
	g_GreenCurrent = GreenCurrent;
	A100_SetPwm(TPL1401_GLED_I2C_ADDR, g_GreenCurrent);
  return 0;
}

uint8_t A100_SetBlueCurrent(uint16_t BlueCurrent) 
{
  if(BlueCurrent > MAX_RGB_VAL)
  {
			BlueCurrent = MAX_RGB_VAL;
  }
  if(BlueCurrent < MIN_RGB_VAL)
  {
			BlueCurrent = MIN_RGB_VAL;
  }
	
	g_BlueCurrent = BlueCurrent;
	A100_SetPwm(TPL1401_BLED_I2C_ADDR, g_BlueCurrent);
  return 0;
}


uint8_t A100_SetRedCurrent(uint16_t RedCurrent)
{

	if(RedCurrent > MAX_RGB_VAL)
  {
			RedCurrent  = MAX_RGB_VAL;
  }
	
	if(RedCurrent < MIN_RGB_VAL)
  {
			RedCurrent  = MIN_RGB_VAL;
  }
	
	g_RedCurrent = RedCurrent;
	A100_SetPwm(TPL1401_RLED_I2C_ADDR, g_RedCurrent);

  return 0;
}

void A100_SetRGBCurrent(void)
{
	uint16_t RedCurrent;
	uint16_t GreenCurrent;	
	uint16_t BlueCurrent;	

	
	if(g_projector_para.projector_tuning.tuning_valid == 1)
	{
		RedCurrent 	= g_projector_para.projector_tuning.cct[g_projector_para.projector_tuning.cct_index].briness[g_projector_para.projector_tuning.briness_index].red_current ;
		GreenCurrent = g_projector_para.projector_tuning.cct[g_projector_para.projector_tuning.cct_index].briness[g_projector_para.projector_tuning.briness_index].green_current ;
		BlueCurrent 	= g_projector_para.projector_tuning.cct[g_projector_para.projector_tuning.cct_index].briness[g_projector_para.projector_tuning.briness_index].blue_current ;
	}

	else if (g_projector_para.current_valid == 1)
	{
		RedCurrent 	= g_projector_para.red_current;
		GreenCurrent = g_projector_para.green_current;
		BlueCurrent 	= g_projector_para.blue_current;
	}
	else
	{
		RedCurrent 	= DEFAULT_R_VAL;
		GreenCurrent = DEFAULT_G_VAL;
		BlueCurrent 	= DEFAULT_B_VAL;
	}

	A100_SetRedCurrent(RedCurrent);
  A100_SetGreenCurrent(GreenCurrent);
  A100_SetBlueCurrent(BlueCurrent);
	
#if 0
	{
		uint16_t status = 0;
		
		status = TPL1401_ReadI2C_Byte(TPL1401_RLED_I2C_ADDR, 0xD0);
		printf("Get Red Led TPL1401 Status:0x%x\r\n", status);
		status = TPL1401_ReadI2C_Byte(TPL1401_RLED_I2C_ADDR, 0xD1);
		printf("Get Red Led TPL1401 0xD1:0x%x\r\n", status);		
		status = TPL1401_ReadI2C_Byte(TPL1401_RLED_I2C_ADDR, 0xD3);
		printf("Get Red Led TPL1401 0xD3:0x%x\r\n", status);		
		status = TPL1401_ReadI2C_Byte(TPL1401_RLED_I2C_ADDR, 0x21);
		printf("Get Red Led TPL1401 0x21:0x%x\r\n", status);		
		
		status = TPL1401_ReadI2C_Byte(TPL1401_GLED_I2C_ADDR, 0xD0);
		printf("Get Green Led TPL1401 Status:0x%x\r\n", status);	
		status = TPL1401_ReadI2C_Byte(TPL1401_GLED_I2C_ADDR, 0xD1);
		printf("Get Green Led TPL1401 0xD1:0x%x\r\n", status);		
		status = TPL1401_ReadI2C_Byte(TPL1401_GLED_I2C_ADDR, 0xD3);
		printf("Get Green Led TPL1401 0xD3:0x%x\r\n", status);	
		status = TPL1401_ReadI2C_Byte(TPL1401_GLED_I2C_ADDR, 0x21);
		printf("Get Green Led TPL1401 0x21:0x%x\r\n", status);	
		
		status = TPL1401_ReadI2C_Byte(TPL1401_BLED_I2C_ADDR, 0xD0);
		printf("Get Blue Led TPL1401 Status:0x%x\r\n", status);
		status = TPL1401_ReadI2C_Byte(TPL1401_BLED_I2C_ADDR, 0xD1);
		printf("Get Blue Led TPL1401 0xD1:0x%x\r\n", status);	
		status = TPL1401_ReadI2C_Byte(TPL1401_BLED_I2C_ADDR, 0xD3);
		printf("Get Blue Led TPL1401 0xD3:0x%x\r\n", status);	
		status = TPL1401_ReadI2C_Byte(TPL1401_BLED_I2C_ADDR, 0x21);
		printf("Get Blue Led TPL1401 0x21:0x%x\r\n", status);		
	}
#endif	
	
}
