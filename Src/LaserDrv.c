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
#define RETRY_CNT 5

#ifndef CONFIG_AD5316R_CURRENT
/***********************************************************************************************************************/
#define TPL1401_RLED_I2C_ADDR 0x90
#define TPL1401_GLED_I2C_ADDR 0x92
#define TPL1401_BLED_I2C_ADDR 0x94

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

#define DEFAULT_R_VAL   	92
#define DEFAULT_G_VAL   	75
#define DEFAULT_B_VAL   	73

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
#else
/**************************************************************AD5316R**************************************************/
#define AD5316R_I2C_ADDR 								0x1C 	//A1-1  A0-0
#define AD5316R_CMD_NONE  							0
#define AD5316R_CMD_WRITE_REG_LDAC  		1
#define AD5316R_CMD_UPDATE_LDAC  				2
#define AD5316R_CMD_WRITE_UPDATE_LDAC  	3
#define AD5316R_CMD_POWER						  	4
#define AD5316R_CMD_MASK_LADC						5
#define AD5316R_CMD_SW_REST							6
#define AD5316R_CMD_VOL_REF							7

#define AD5316R_ADC_ADDR_OUTA						1
#define AD5316R_ADC_ADDR_OUTB						2
#define AD5316R_ADC_ADDR_OUTC						4


#define MAX_RED_VAL_AD5316R   		200
#define MAX_GREEN_VAL_AD5316R   	250
#define MAX_BLUE_VAL_AD5316R   		255

#define MIN_RED_VAL_AD5316R   		10
#define MIN_GREEN_VAL_AD5316R   	10
#define MIN_BLUE_VAL_AD5316R   		10

#define DEFAULT_R_VAL_AD5316R   	60
#define DEFAULT_G_VAL_AD5316R   	60
#define DEFAULT_B_VAL_AD5316R   	60

uint8_t AD5316R_WriteI2C_Byte(uint8_t RegAddr, uint16_t data)
{
	uint8_t ret, retry_cnt = RETRY_CNT;
	uint8_t d[2];

	d[0] = ((data << 6) & 0xff00) >> 8 ;
	d[1] = (data << 6 ) & 0xff;

	while(retry_cnt --) {
		ret = HAL_I2C_Mem_Write(&hi2c2, (uint16_t)AD5316R_I2C_ADDR, (uint16_t)RegAddr, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&d, 2, 300);
		if(ret == HAL_OK) break;
	}
	if(ret != HAL_OK) 
	{
		printf("AD5316R_WriteI2C_Byte 0x%x->0x%x faild. ret=%d\r\n", RegAddr, data, ret);
	}	else {
		printf("AD5316R_WriteI2C_Byte 0x%x->0x%x OK. \r\n", RegAddr, data);
	}		
	
	return ret;
}

uint16_t AD5316R_ReadI2C_Byte(uint8_t RegAddr)
{
	uint8_t ret,retry_cnt=RETRY_CNT;
	uint8_t d[2] = {0};
	uint16_t data;
	
	while(retry_cnt --) {	
		ret = HAL_I2C_Mem_Read(&hi2c2, (uint16_t)AD5316R_I2C_ADDR, (uint16_t)RegAddr, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&d, 2, 300);
		if(ret == HAL_OK) break;
	}
	if(ret != HAL_OK) 
	{
		printf("AD5316R_ReadI2C_Byte 0x%x faild. ret=%d\r\n", RegAddr, ret);
		return 0xffff;
	}
	
	data = (d[0] << 8) + d[1];
	
	return (data >>6);
}

void AD5316R_PowerUpDac(void)  //powerup outa outb outc
{
	uint8_t command_byte, data;
	
	command_byte = (AD5316R_CMD_POWER << 4);
	data = 0xc0;
	AD5316R_WriteI2C_Byte(command_byte, data);	
}

void AD5316R_EnableVolRef(void)
{
	uint8_t command_byte, data;
	
	command_byte = (AD5316R_CMD_VOL_REF << 4);
	data = 0;
	AD5316R_WriteI2C_Byte(command_byte, data);	
}

void AD5316R_DisableVolRef(void)
{
	uint8_t command_byte, data;
	
	command_byte = (AD5316R_CMD_VOL_REF << 4);
	data = 1;
	AD5316R_WriteI2C_Byte(command_byte, data);	
}

uint8_t AD5316R_SetRedPwm(uint32_t data)
{
	uint8_t command_byte;
	
	data = (data * MAX_RED_VAL_AD5316R)/100;
	command_byte = (AD5316R_CMD_WRITE_UPDATE_LDAC << 4) + AD5316R_ADC_ADDR_OUTA;
	return AD5316R_WriteI2C_Byte(command_byte, data);
}

uint8_t AD5316R_SetGreenPwm(uint32_t data)
{
	uint8_t command_byte;
	
	data = (data * MAX_GREEN_VAL_AD5316R)/100;	
	command_byte = (AD5316R_CMD_WRITE_UPDATE_LDAC << 4) + AD5316R_ADC_ADDR_OUTB;
	return AD5316R_WriteI2C_Byte(command_byte, data);	
}

uint8_t AD5316R_SetBluePwm(uint32_t data)
{
	uint8_t command_byte;
	
	data = (data * MAX_BLUE_VAL_AD5316R)/100;
	command_byte = (AD5316R_CMD_WRITE_UPDATE_LDAC << 4) + AD5316R_ADC_ADDR_OUTC;
	return AD5316R_WriteI2C_Byte(command_byte, data);	
}

uint8_t SetRedCurrent_AD5316R(uint16_t RedCurrent)
{
	g_RGBCurrent[0] = RedCurrent;
	
	if(RedCurrent > MAX_RED_VAL_AD5316R)
  {
			RedCurrent  = MAX_RED_VAL_AD5316R;
  }	
	
	if(RedCurrent < MIN_RED_VAL_AD5316R)
  {
			RedCurrent  = MIN_RED_VAL_AD5316R;
  }
	
	return AD5316R_SetRedPwm(RedCurrent);
}


uint8_t GetRGBCurrent_AD5316R(uint8_t rgb)
{
#if 1
	return (uint8_t)g_RGBCurrent[rgb];
#else
	if(rgb == 0)
		return AD5316R_ReadI2C_Byte((AD5316R_CMD_WRITE_UPDATE_LDAC << 4) + AD5316R_ADC_ADDR_OUTA);
	else if(rgb == 1)
		return AD5316R_ReadI2C_Byte((AD5316R_CMD_WRITE_UPDATE_LDAC << 4) + AD5316R_ADC_ADDR_OUTB);
	else if(rgb == 2)
		return AD5316R_ReadI2C_Byte((AD5316R_CMD_WRITE_UPDATE_LDAC << 4) + AD5316R_ADC_ADDR_OUTC);
	
	return 0xff;
#endif
}

uint8_t SetGreenCurrent_AD5316R(uint16_t GreenCurrent)
{
	g_RGBCurrent[1] = GreenCurrent;
	
  if(GreenCurrent > MAX_GREEN_VAL_AD5316R)
  {
			GreenCurrent  = MAX_GREEN_VAL_AD5316R;
  }
	
  if(GreenCurrent < MIN_GREEN_VAL_AD5316R)
  {
			GreenCurrent  = MIN_GREEN_VAL_AD5316R;
  }

	return AD5316R_SetGreenPwm(GreenCurrent);
}

uint8_t SetBlueCurrent_AD5316R(uint16_t BlueCurrent) 
{
	g_RGBCurrent[2] = BlueCurrent;
	
  if(BlueCurrent > MAX_BLUE_VAL_AD5316R)
  {
			BlueCurrent = MAX_BLUE_VAL_AD5316R;
  }
	
  if(BlueCurrent < MIN_BLUE_VAL_AD5316R)
  {
			BlueCurrent = MIN_BLUE_VAL_AD5316R;
  }
	
	return AD5316R_SetBluePwm(BlueCurrent);
}

void SetRGBCurrentAD5316R(void)
{
	uint16_t RedCurrent;
	uint16_t GreenCurrent;	
	uint16_t BlueCurrent;	

	if(g_pColorTemp->current.valid == PARAMETER_VALID)
	{
		RedCurrent 	= g_pColorTemp->current.r;
		GreenCurrent = g_pColorTemp->current.g;
		BlueCurrent 	= g_pColorTemp->current.b;
	}
	else
	{
		RedCurrent 	= DEFAULT_R_VAL_AD5316R;
		GreenCurrent = DEFAULT_G_VAL_AD5316R;
		BlueCurrent 	= DEFAULT_B_VAL_AD5316R;
	}

	SetRedCurrent_AD5316R(RedCurrent);
  SetGreenCurrent_AD5316R(GreenCurrent);
  SetBlueCurrent_AD5316R(BlueCurrent);
	printf("SetRGBCurrentAD5316R: valid %d : %d %d %d \r\n", g_pColorTemp->current.valid, RedCurrent, GreenCurrent, BlueCurrent);
	
}
#endif
 

