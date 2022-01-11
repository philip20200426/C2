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

uint8_t g_laser_mode = UNKNOWN_MODE;
uint16_t g_RGBCurrent[3];
extern struct Projector_Color_Temp * g_pColorTemp;

#define TPL1401_RLED_I2C_ADDR 0x90
#define TPL1401_GLED_I2C_ADDR 0x92
#define TPL1401_BLED_I2C_ADDR 0x94

#define RETRY_CNT 3
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

#define DEFAULT_R_VAL   	80
#define DEFAULT_G_VAL   	80
#define DEFAULT_B_VAL   	80

#define MAX_RED_VAL_MCU   		92 
#define MAX_GREEN_VAL_MCU   	88
#define MAX_BLUE_VAL_MCU   		92

#define MIN_RED_VAL_MCU   		1  
#define MIN_GREEN_VAL_MCU   	1
#define MIN_BLUE_VAL_MCU   		1

#define DEFAULT_R_VAL_MCU   	80
#define DEFAULT_G_VAL_MCU   	80
#define DEFAULT_B_VAL_MCU   	80
uint8_t GetRGBCurrent(uint8_t rgb)
{
	uint8_t val;
	
	if(rgb == 0)
	{
		val = MAX_RED_VAL - (TPL1401_ReadI2C_Byte(TPL1401_RLED_I2C_ADDR + (rgb << 1), 0x21) >> 4);
		if(val > MIN_RED_VAL && val < MAX_RED_VAL)
			return val;
		else 
			return g_RGBCurrent[0];
	}
	else if(rgb == 1)
	{
		val = MAX_GREEN_VAL - (TPL1401_ReadI2C_Byte(TPL1401_RLED_I2C_ADDR + (rgb << 1), 0x21) >> 4);
		if(val > MIN_GREEN_VAL && val < MAX_GREEN_VAL)
			return val;
		else 
			return g_RGBCurrent[1];
	}
	else if(rgb == 2)
	{
		val = MAX_BLUE_VAL - (TPL1401_ReadI2C_Byte(TPL1401_RLED_I2C_ADDR + (rgb << 1), 0x21) >> 4);
		if(val > MIN_BLUE_VAL && val < MAX_BLUE_VAL)
			return val;
		else 
			return g_RGBCurrent[2];
	}

	return 0xff;
}



uint8_t SetPwm(uint8_t DevAddr, uint16_t data)
{
	uint8_t ret;
	
	//printf("SetPwm: 0x%x -> 0x%x \r\n", DevAddr, data);

	ret = TPL1401_WriteI2C_Byte(DevAddr, 0xD1, 0x01E0);//0x11E5
	if(ret != HAL_OK) return ret;
	//HAL_Delay(1);
	ret = TPL1401_WriteI2C_Byte(DevAddr, 0x21, data);
	if(ret != HAL_OK) return ret;
	//HAL_Delay(1);
	ret = TPL1401_WriteI2C_Byte(DevAddr, 0xD3, 0x0010);	
	if(ret != HAL_OK) return ret;
	//HAL_Delay(1);
	
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
		
		HAL_Delay(15);
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
/***********************************************************************************************************************/
#ifdef CONFIG_MCU_CURRENT
#define MCU_I2C_ADDR 0x96
uint8_t ConvertTo100(uint8_t val)
{
	return (MCU_I2C_ADDR * val) / 100;
}

uint8_t Mcu_WriteI2C_rgb(uint8_t DevAddr, uint8_t RegAddr, uint8_t r,  uint8_t g, uint8_t b)
{
		uint8_t ret, retry_cnt = RETRY_CNT;
		uint8_t d[3];

		d[0] = r;
		d[1] = b;
		d[2] = g;
	
		while(retry_cnt --) {
			ret = HAL_I2C_Mem_Write(&hi2c2, (uint16_t)DevAddr, (uint16_t)RegAddr, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&d, 3, 300);
			if(ret == HAL_OK) break;
		}
		if(ret != HAL_OK)
		{		
			printf("Mcu_WriteI2C_rgb rgb:%d %d %d faild. ret=%d\r\n", r, g, b, ret);			
		}
		
		return ret;
}


uint8_t GetRGBCurrent_Mcu(uint8_t* rgb, uint8_t index)
{
	uint8_t ret,retry_cnt=RETRY_CNT;

	while(retry_cnt --) {	
		ret = HAL_I2C_Mem_Read(&hi2c2, MCU_I2C_ADDR, index, I2C_MEMADD_SIZE_8BIT, rgb, 1, 300);
		if(ret == HAL_OK) break;
	}
	if(ret != HAL_OK) 
	{
		printf("GetRGBCurrent_Mcu faild. ret=%d\r\n", ret);
	}

	if(index == 0 && (*rgb == MIN_RED_VAL_MCU || *rgb == MAX_RED_VAL_MCU))
		*rgb = g_RGBCurrent[0];

	if(index == 2 && (*rgb == MIN_GREEN_VAL_MCU || *rgb == MAX_GREEN_VAL_MCU))
		*rgb = g_RGBCurrent[1];

	if(index == 1 && (*rgb == MIN_BLUE_VAL_MCU || *rgb == MAX_BLUE_VAL_MCU))
		*rgb = g_RGBCurrent[2];
	
	return ret;
}

uint8_t SetRGBCurrent_Mcu(uint8_t r, uint8_t g, uint8_t b)
{
	g_RGBCurrent[0] = r;
	g_RGBCurrent[1] = g;
	g_RGBCurrent[2] = b;
	
  if(r > MAX_RED_VAL_MCU)
			r = MAX_RED_VAL_MCU;
  if(r < MIN_RED_VAL_MCU)
			r = MIN_RED_VAL_MCU;
  if(g > MAX_GREEN_VAL_MCU)
			g = MAX_GREEN_VAL_MCU;
  if(g < MIN_GREEN_VAL_MCU)
			g = MIN_GREEN_VAL_MCU;
  if(b > MAX_BLUE_VAL_MCU)
			b = MAX_BLUE_VAL_MCU;
  if(b < MIN_BLUE_VAL_MCU)
			b = MIN_BLUE_VAL_MCU;
	
	return Mcu_WriteI2C_rgb(MCU_I2C_ADDR, 0, r, g , b);
}
/***********************************************************************************************************************/
void Check_LaserDrv_Mode(void)
{
		uint8_t ret;
		uint8_t val;
	
	  ret = (TPL1401_ReadI2C_Byte(TPL1401_RLED_I2C_ADDR + (0 << 1), 0x21) >> 4);//read red led
		if(ret != 0xff)
		{
			g_laser_mode = TPL1401_MODE;
			printf("Check_LaserDrv_Mode: %d \r\n", g_laser_mode);
			return;
		} else if(GetRGBCurrent_Mcu(&val, 0) == HAL_OK){
			g_laser_mode = MCU_MODE;
			printf("Check_LaserDrv_Mode: red-> %d \r\n", val);
			#if 0
			GetRGBCurrent_Mcu(&val, 1);
			HAL_Delay(1);
			printf("Check_LaserDrv_Mode: green-> %d \r\n", val);
			GetRGBCurrent_Mcu(&val, 2);
			HAL_Delay(1);
			printf("Check_LaserDrv_Mode: blue-> %d \r\n", val);	
			#endif
			printf("Check_LaserDrv_Mode: %d \r\n", g_laser_mode);
		}
		HAL_Delay(1);
}
#endif

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
#ifdef CONFIG_MCU_CURRENT
		if(g_laser_mode == MCU_MODE)
		{
			RedCurrent 	= DEFAULT_R_VAL_MCU;
			GreenCurrent = DEFAULT_G_VAL_MCU;
			BlueCurrent 	= DEFAULT_B_VAL_MCU;
		}
		else 
#endif
		{
			RedCurrent 	= DEFAULT_R_VAL;
			GreenCurrent = DEFAULT_G_VAL;
			BlueCurrent 	= DEFAULT_B_VAL;			
		}
	}

	if(retry_cnt == 255)
	{
		printf("set rgb:%d %d %d  retry_cnt timeout. \r\n",RedCurrent, GreenCurrent, BlueCurrent);
	} else {
		printf("set rgb:%d %d %d  retry_cnt:%d \r\n",RedCurrent, GreenCurrent, BlueCurrent, WAIT_12V_TIMEOUT - retry_cnt);
	}
#ifdef CONFIG_MCU_CURRENT
	if(g_laser_mode == TPL1401_MODE)
#endif
	{
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
#ifdef CONFIG_MCU_CURRENT
	else if(g_laser_mode == MCU_MODE) {
		SetRGBCurrent_Mcu(RedCurrent, GreenCurrent, BlueCurrent);
	} else {
		printf("Not found current i2c controler. \r\n");	
	}
#endif
}


/***********************************************************************************************************************/

