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

uint16_t g_RGBCurrent[3];
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

uint8_t GetRGBCurrent(uint8_t rgb)
{
	return (uint8_t)g_RGBCurrent[rgb];
	//return MAX_RGB_VAL - (TPL1401_ReadI2C_Byte(TPL1401_RLED_I2C_ADDR + (rgb << 1), 0x21) >> 4);
}

uint8_t SetPwm(uint8_t DevAddr, uint8_t data)
{
	uint8_t ret;
	uint16_t reg_data = (MAX_RGB_VAL - data) << 4;

	ret = TPL1401_WriteI2C_Byte(DevAddr, 0xD1, 0x01E0);//0x11E5
	if(ret != HAL_OK) return ret;
	HAL_Delay(1);
	ret = TPL1401_WriteI2C_Byte(DevAddr, 0x21, reg_data);
	if(ret != HAL_OK) return ret;
	HAL_Delay(1);
	ret = TPL1401_WriteI2C_Byte(DevAddr, 0xD3, 0x0010);	
	if(ret != HAL_OK) return ret;
	HAL_Delay(1);
	
	return ret;
}


uint8_t SetRedCurrent(uint16_t RedCurrent)
{
	g_RGBCurrent[0] = RedCurrent;
	
	if(RedCurrent > MAX_RGB_VAL)
  {
			RedCurrent  = MAX_RGB_VAL;
  }
	
	if(RedCurrent < MIN_RGB_VAL)
  {
			RedCurrent  = MIN_RGB_VAL;
  }
	
	return SetPwm(TPL1401_RLED_I2C_ADDR, RedCurrent);
}

uint8_t SetGreenCurrent(uint16_t GreenCurrent)
{
	g_RGBCurrent[1] = GreenCurrent;
	
  if(GreenCurrent > MAX_RGB_VAL)
  {
			GreenCurrent  = MAX_RGB_VAL;
  }
  if(GreenCurrent < MIN_RGB_VAL)
  {
			GreenCurrent  = MIN_RGB_VAL;
  }
	
	return SetPwm(TPL1401_GLED_I2C_ADDR, GreenCurrent);
}

uint8_t SetBlueCurrent(uint16_t BlueCurrent) 
{
	g_RGBCurrent[2] = BlueCurrent;
	
  if(BlueCurrent > MAX_RGB_VAL)
  {
			BlueCurrent = MAX_RGB_VAL;
  }
  if(BlueCurrent < MIN_RGB_VAL)
  {
			BlueCurrent = MIN_RGB_VAL;
  }
	
	return SetPwm(TPL1401_BLED_I2C_ADDR, BlueCurrent);
}

void SetRGBCurrent(void)
{
	uint16_t RedCurrent;
	uint16_t GreenCurrent;	
	uint16_t BlueCurrent;	

	
	if(g_projector_para.current.valid == PARAMETER_VALID)
	{
		RedCurrent 	= g_projector_para.current.r;
		GreenCurrent = g_projector_para.current.g;
		BlueCurrent 	= g_projector_para.current.b;
	}
	else
	{
		RedCurrent 	= DEFAULT_R_VAL;
		GreenCurrent = DEFAULT_G_VAL;
		BlueCurrent 	= DEFAULT_B_VAL;
	}
	//printf("rgb:0x%x 0x%x 0x%x \r\n",RedCurrent, GreenCurrent, BlueCurrent);
	SetRedCurrent(RedCurrent);
  SetGreenCurrent(GreenCurrent);
  SetBlueCurrent(BlueCurrent);
	
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
