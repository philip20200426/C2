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
extern struct Projector_Color_Temp * g_pColorTemp;

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

#define MAX_RED_VAL   		92 //0x0640
#define MAX_GREEN_VAL   	92
#define MAX_BLUE_VAL   		92

#define MIN_RED_VAL   		10  //0x00a0
#define MIN_GREEN_VAL   	10
#define MIN_BLUE_VAL   		10

#define DEFAULT_R_VAL   	76
#define DEFAULT_G_VAL   	76
#define DEFAULT_B_VAL   	75

uint8_t GetRGBCurrent(uint8_t rgb)
{
	//return (uint8_t)g_RGBCurrent[rgb];
	
	if(rgb == 0)
		return MAX_RED_VAL - (TPL1401_ReadI2C_Byte(TPL1401_RLED_I2C_ADDR + (rgb << 1), 0x21) >> 4);
	else if(rgb == 1)
		return MAX_GREEN_VAL - (TPL1401_ReadI2C_Byte(TPL1401_RLED_I2C_ADDR + (rgb << 1), 0x21) >> 4);
	else if(rgb == 2)
		return MAX_BLUE_VAL - (TPL1401_ReadI2C_Byte(TPL1401_RLED_I2C_ADDR + (rgb << 1), 0x21) >> 4);
	
	return 0xff;
}



uint8_t SetPwm(uint8_t DevAddr, uint16_t data)
{
	uint8_t ret;
	
	//printf("SetPwm: 0x%x -> 0x%x \r\n", DevAddr, data);

	ret = TPL1401_WriteI2C_Byte(DevAddr, 0xD1, 0x01E0);//0x11E5
	if(ret != HAL_OK) return ret;
	HAL_Delay(1);
	ret = TPL1401_WriteI2C_Byte(DevAddr, 0x21, data);
	if(ret != HAL_OK) return ret;
	HAL_Delay(1);
	ret = TPL1401_WriteI2C_Byte(DevAddr, 0xD3, 0x0010);	
	if(ret != HAL_OK) return ret;
	HAL_Delay(1);
	
	return ret;
}

#define MAX_DOWN_VAL  10
uint8_t SetPwmPre(uint8_t DevAddr, uint16_t data)
{
	uint8_t reg_val;
	
	if(TPL1401_RLED_I2C_ADDR == DevAddr)
		reg_val = GetRGBCurrent(0);
	else if(TPL1401_GLED_I2C_ADDR == DevAddr)
		reg_val = GetRGBCurrent(1);
	else if(TPL1401_BLED_I2C_ADDR == DevAddr)
		reg_val = GetRGBCurrent(2);	
	
	if(data >= reg_val) return data;
	
	while((reg_val - data) > MAX_DOWN_VAL)
	{
		reg_val = reg_val - MAX_DOWN_VAL;
		if(TPL1401_RLED_I2C_ADDR == DevAddr)
			SetPwm(TPL1401_RLED_I2C_ADDR, (MAX_RED_VAL - reg_val) << 4);
		else if(TPL1401_GLED_I2C_ADDR == DevAddr)
			SetPwm(TPL1401_GLED_I2C_ADDR, (MAX_GREEN_VAL - reg_val) << 4);
		else if(TPL1401_BLED_I2C_ADDR == DevAddr)
			SetPwm(TPL1401_BLED_I2C_ADDR, (MAX_BLUE_VAL - reg_val) << 4);
		
		HAL_Delay(10);
	}
	
	return data;
}
uint8_t SetRedCurrent(uint16_t RedCurrent)
{
	g_RGBCurrent[0] = RedCurrent;
	
	if(RedCurrent > MAX_RED_VAL)
  {
			RedCurrent  = MAX_RED_VAL;
  }	
	
	if(RedCurrent < MIN_RED_VAL)
  {
			RedCurrent  = MIN_RED_VAL;
  }
	
	RedCurrent = SetPwmPre(TPL1401_RLED_I2C_ADDR, RedCurrent);
	
	return SetPwm(TPL1401_RLED_I2C_ADDR, (MAX_RED_VAL - RedCurrent) << 4);
}

uint8_t SetGreenCurrent(uint16_t GreenCurrent)
{
	g_RGBCurrent[1] = GreenCurrent;
	
  if(GreenCurrent > MAX_GREEN_VAL)
  {
			GreenCurrent  = MAX_GREEN_VAL;
  }
	
  if(GreenCurrent < MIN_GREEN_VAL)
  {
			GreenCurrent  = MIN_GREEN_VAL;
  }

	GreenCurrent = SetPwmPre(TPL1401_GLED_I2C_ADDR, GreenCurrent);	
	return SetPwm(TPL1401_GLED_I2C_ADDR, (MAX_GREEN_VAL - GreenCurrent) << 4);
}

uint8_t SetBlueCurrent(uint16_t BlueCurrent) 
{
	g_RGBCurrent[2] = BlueCurrent;
	
  if(BlueCurrent > MAX_BLUE_VAL)
  {
			BlueCurrent = MAX_BLUE_VAL;
  }
	
  if(BlueCurrent < MIN_BLUE_VAL)
  {
			BlueCurrent = MIN_BLUE_VAL;
  }
	BlueCurrent = SetPwmPre(TPL1401_BLED_I2C_ADDR, BlueCurrent);	
	return SetPwm(TPL1401_BLED_I2C_ADDR, (MAX_BLUE_VAL - BlueCurrent) << 4);
}

#define WAIT_12V_TIMEOUT 	30  //3s
void SetRGBCurrent(void)
{
	uint16_t RedCurrent;
	uint16_t GreenCurrent;	
	uint16_t BlueCurrent;	
	uint8_t retry_cnt = WAIT_12V_TIMEOUT;
	//MB_GPIO0
	while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_15) != GPIO_PIN_SET && retry_cnt--)
	{
		HAL_Delay(100);
	}

	if(g_pColorTemp->current.valid == PARAMETER_VALID)
	{
		RedCurrent 	= g_pColorTemp->current.r;
		GreenCurrent = g_pColorTemp->current.g;
		BlueCurrent 	= g_pColorTemp->current.b;
	}
	else
	{
		RedCurrent 	= DEFAULT_R_VAL;
		GreenCurrent = DEFAULT_G_VAL;
		BlueCurrent 	= DEFAULT_B_VAL;
	}

	if(retry_cnt == 255)
	{
		printf("set rgb:%d %d %d  retry_cnt timeout. \r\n",RedCurrent, GreenCurrent, BlueCurrent);
	} else {
		printf("set rgb:%d %d %d  retry_cnt:%d \r\n",RedCurrent, GreenCurrent, BlueCurrent, WAIT_12V_TIMEOUT - retry_cnt);
	}
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
