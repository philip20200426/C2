/**
  ******************************************************************************
  * File Name          : serial_communication.c
  * Description        : This file provides code for serial communication
  *                      of uart0.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "serial_communication.h"

/* Exported functions prototypes ---------------------------------------------*/
extern uint16_t GetLd_RT_Temp(uint16_t adc_val);
extern uint16_t GetLcos_RT_Temp(uint16_t adc_val);
extern uint8_t ReadReg_10983(uint8_t RegAddr);
extern uint8_t GetRGBCurrent(uint8_t rgb);

void Uart_Send_Response(uint16_t command, uint8_t* data, uint8_t size );
uint8_t GetFan1Speed(void);
uint8_t GetFan2Speed(void);
uint8_t GetFan3Speed(void);
uint8_t GetFan4Speed(void);
uint8_t GetFan5Speed(void);
uint8_t GetThreePMotorSpeed(void);
/* Private variables ---------------------------------------------------------*/
uint16_t g_fan12_speed,g_fan34_speed,g_fan5_speed;
struct Projector_parameter  g_projector_para;

volatile uint8_t UartReceiveRxBuffer[UART_BUFFER_MAX_SIZE] = {0};
volatile uint8_t UartTempBuffer[UART_BUFFER_MAX_SIZE] = {0};
volatile uint8_t UartCommandBuffer[UART_BUFFER_MAX_SIZE] = {0};
volatile uint8_t UartReceiveLength = 0;
volatile uint8_t UartTempLength = 0;
volatile uint8_t UartCommandLength = 0;
/* sony tools  ---------------------------------------------------------*/
struct asu_date asu_rec_data;
int  data_len = 0;
char receive_buffer[40];
char R_0[15] = {114,32,48,13,13,10,32,79,75,32,50,48,66,13,10};
char R_1[13] = {114,32,49,13,13,10,32,79,75,32,49,13,10};
char R_2[15] = {82,32,50,13,13,10,32,79,75,32,50,52,49,13,10};
char OK_0[8] =  {13,13,10,32,79,75,13,10};
char OK_1[10] = {13,13,10,32,79,75,32,48,13,10};
char OK_3[10] = {13,13,10,32,79,75,32,50,13,10};
char OK_7[10] = {13,13,10,32,79,75,32,54,13,10};
char flag_0 = 0;

extern volatile _Bool g_FanMode;
extern volatile _Bool Flag_MatMode;
extern uint16_t g_RGBCurrent[3];

/* Private function prototypes -----------------------------------------------*/
unsigned char CharToHex(unsigned char bHex)
{
	if((bHex>='0')&&(bHex<='9'))
	{
		bHex -= 0x30;
	}
	else if((bHex>='a')&&(bHex<='f'))
	{
		bHex -= 0x57;
	}
	else if((bHex>='A')&&(bHex<='F'))
	{
		bHex -= 0x37;
	}
	return bHex;
}

unsigned char HexToChar(unsigned char bHex)
{
	unsigned char bHex_ret;
	//if((bHex>=0x0)&&(bHex<=0x9))
	if(bHex<=0x9)
	{
		bHex += 0x30;
	}
	else
	{
		bHex += 0x57;
	}
	bHex_ret =  bHex;
	return bHex_ret;
}

int read_evm(char* buf,int len, int* para0_len,int* para1_len)
{
	int i, read_len,flag;
	char read_char,flag0,flag1;
	read_len = 0;
	*para0_len = 0;
	*para1_len = 0;
	flag = 1;
	for(i=0; i < len;i++)
	{
		read_len = read_len + 1;
		read_char = buf[i];

		if(i==0)
		{
			flag0 = read_char;
		}
		if(i==1)
		{
			flag1 = read_char;
		}
		if((flag0 == 'h' && flag1 == 'r') || (flag0 == 'p' && flag1 == 'r'))
		{
			if((read_len>3) && (flag==1))
			{
				if(read_char == 13)
				{
					flag = 0;
				}
				else
				{
					*para0_len = *para0_len + 1;
				}
			}
		}

		else if((flag0 == 'h' && flag1 == 'w') || (flag0 == 'p' && flag1 == 'w'))
		{
			if((read_len>3) && (flag==1))
			{
				if(read_char == 32)
				{
					flag = 2;
				}
				else
				{
					*para0_len = *para0_len + 1;
				}
			}
			if(flag==2 && read_char!=32)
			{
				if(read_char == 13)
				{
					flag = 3;
				}
				else
				{
					*para1_len = *para1_len + 1;
				}
			}
		}
		else
		{
			if((read_len>5) && (flag==1))
			{
				if(read_char == 13)
				{
					flag = 0;
				}
				else
				{
					*para0_len = *para0_len + 1;
				}
			}
		}	
		if(read_char==10)
			break;
	}
	return read_len;
}

void para_Analysis(char* buf,int para0_len,int para1_len)
{

	if(((buf[2]=='8') && (buf[3]=='8')) || ((buf[2]=='8') && (buf[3]=='e')))
	{
		asu_rec_data.chip_addr = CharToHex(buf[5])*16 + CharToHex(buf[6]);
	}
	if(((buf[2]=='8') && (buf[3]=='9')) || ((buf[2]=='8') && (buf[3]=='f')))
	{
		if(para0_len == 1)
			asu_rec_data.reg_addr = CharToHex(buf[5]);
		if(para0_len == 2)
			asu_rec_data.reg_addr = CharToHex(buf[5])*16 + CharToHex(buf[6]);
		if(para0_len == 3)
			asu_rec_data.reg_addr = CharToHex(buf[5])*16*16 + CharToHex(buf[6])*16+CharToHex(buf[7]);
		if(para0_len == 4)
			asu_rec_data.reg_addr = CharToHex(buf[5])*16*16*16 + CharToHex(buf[6])*16*16 + CharToHex(buf[7])*16 + CharToHex(buf[8]);
	}
	if(((buf[2]=='8') && (buf[3]=='b'))||((buf[2]=='8') && (buf[3]=='B')) || ((buf[2]=='9') && (buf[3]=='1')))
	{
		if(para0_len == 1)
			asu_rec_data.reg_value = CharToHex(buf[5]);
		if(para0_len == 2)
			asu_rec_data.reg_value = CharToHex(buf[5])*16 + CharToHex(buf[6]);
		if(para0_len == 3)
			asu_rec_data.reg_value = CharToHex(buf[5])*16*16 + CharToHex(buf[6])*16+CharToHex(buf[7]);
		if(para0_len == 4)
			asu_rec_data.reg_value = CharToHex(buf[5])*16*16*16 + CharToHex(buf[6])*16*16 + CharToHex(buf[7])*16 + CharToHex(buf[8]);
		asu_rec_data.w_r = 1 ;

	}
	if(((buf[2]=='9') && (buf[3]=='2')) || ((buf[2]=='8') && (buf[3]=='c'))) 
	{
		asu_rec_data.w_r = 2 ;
	}

	if((buf[0]=='p') && (buf[1]=='r'))
	{
		asu_rec_data.chip_addr = SXRD241_I2C_ADDRESS;
		if(para0_len == 1)
			asu_rec_data.reg_addr = CharToHex(buf[3]);
		if(para0_len == 2)
			asu_rec_data.reg_addr = CharToHex(buf[3])*16 + CharToHex(buf[4]);
		if(para0_len == 3)
			asu_rec_data.reg_addr = CharToHex(buf[3])*16*16 + CharToHex(buf[4])*16 + CharToHex(buf[5]);
		if(para0_len == 4)
			asu_rec_data.reg_addr = CharToHex(buf[3])*16*16*16 + CharToHex(buf[4])*16*16 + CharToHex(buf[5])*16 + CharToHex(buf[6]);
		asu_rec_data.w_r = 3 ;
	}
	if((buf[0]=='h') && (buf[1]=='r'))
	{
		asu_rec_data.chip_addr = CXD3554_I2C_ADDRESS;
		if(para0_len == 1)
			asu_rec_data.reg_addr = CharToHex(buf[3]);
		if(para0_len == 2)
			asu_rec_data.reg_addr = CharToHex(buf[3])*16 + CharToHex(buf[4]);
		if(para0_len == 3)
			asu_rec_data.reg_addr = CharToHex(buf[3])*16*16 + CharToHex(buf[4])*16 + CharToHex(buf[5]);
		if(para0_len == 4)
			asu_rec_data.reg_addr = CharToHex(buf[3])*16*16*16 + CharToHex(buf[4])*16*16 + CharToHex(buf[5])*16 + CharToHex(buf[6]);
		asu_rec_data.w_r = 4 ;
	}
	if(para1_len!=0)
	{
		if((buf[0]=='h') && (buf[1]=='w'))
			asu_rec_data.chip_addr = CXD3554_I2C_ADDRESS;
		else
			asu_rec_data.chip_addr = SXRD241_I2C_ADDRESS;

		if(para0_len == 1)
			asu_rec_data.reg_addr = CharToHex(buf[3]);
		if(para0_len == 2)
			asu_rec_data.reg_addr = CharToHex(buf[3])*16 + CharToHex(buf[4]);
		if(para0_len == 3)
			asu_rec_data.reg_addr = CharToHex(buf[3])*16*16 + CharToHex(buf[4])*16 + CharToHex(buf[5]);
		if(para0_len == 4)
			asu_rec_data.reg_addr = CharToHex(buf[3])*16*16*16 + CharToHex(buf[4])*16*16 + CharToHex(buf[5])*16 + CharToHex(buf[6]);

		if(para1_len == 1)
			asu_rec_data.reg_value = CharToHex(buf[3 + para0_len +1]);
		if(para1_len == 2)
			asu_rec_data.reg_value = CharToHex(buf[3 + para0_len +1])*16 + CharToHex(buf[3 + para0_len +2]);
		if(para1_len == 3)
			asu_rec_data.reg_value = CharToHex(buf[3 + para0_len +1])*16*16 + CharToHex(buf[3 + para0_len +2])*16 + CharToHex(buf[3 + para0_len +3]);
		if(para1_len == 4)
			asu_rec_data.reg_value = CharToHex(buf[3 + para0_len +1])*16*16*16 + CharToHex(buf[3 + para0_len +2])*16*16 + CharToHex(buf[3 + para0_len +3])*16 + CharToHex(buf[3 + para0_len +4]) ;
		asu_rec_data.w_r = 5 ;
	}
}

void show_data(struct asu_date *data)
{
	//if(data->reg_value >=0x0 && data->reg_value <=0xf)
	if(data->reg_value <=0xf)
		data_len = 1;
	if(data->reg_value >=0x10 && data->reg_value <=0xff)
		data_len = 2;
	if(data->reg_value >=0x100 && data->reg_value <=0xfff)
		data_len = 3;
	if(data->reg_value >=0x1000 && data->reg_value <=0xffff)
		data_len = 4;
}


/* flash ------------------------------------------------------------------*/
/**
  * @brief  Gets the page of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The page of a given address
  */
static uint32_t GetPage(uint32_t Addr)
{
  return (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;;
}

void GetParameter(void)
{
	uint32_t *Projector_Config;
	uint32_t i;
	uint32_t Address;	
	
	/* Check the correctness of written data */
	Projector_Config = (uint32_t *)&g_projector_para;
	Address = FLASH_USER_START_ADDR;
	for(i=0 ;i<sizeof(struct Projector_parameter)/4 ;i++)
	{
		Projector_Config[i] = *(__IO uint32_t*) Address	;
		Address += 4;
	}
#if 0	
	printf("current:0x%x \r\n",g_projector_para.current.valid);
	printf("rgb:0x%x 0x%x 0x%x \r\n",g_projector_para.current.r, g_projector_para.current.g, g_projector_para.current.b);
	printf("kst:0x%x \r\n",g_projector_para.kst.valid);	
	printf("flip:0x%x \r\n",g_projector_para.flip.valid);
	printf("gama:0x%x \r\n",g_projector_para.gama.valid);
	printf("wp:0x%x \r\n",g_projector_para.wp.valid);	
#endif
	return;
}

void SetParameter(struct Projector_parameter  *g_projector_parameter )
{
	uint32_t FirstPage = 0, NbOfPages = 0;
	uint32_t Address = 0, PageError = 0;
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint64_t *projector_Config;
	uint32_t i;

	projector_Config =  (uint64_t *)g_projector_parameter;

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();
  
  /* Erase the user Flash area
    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

  /* Get the 1st page to erase */
  FirstPage = GetPage(FLASH_USER_START_ADDR);

  /* Get the number of pages to erase from 1st page */
  NbOfPages = GetPage(FLASH_USER_END_ADDR) - FirstPage + 1;

  /* Fill EraseInit structure*/
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Page        = FirstPage;
  EraseInitStruct.NbPages     = NbOfPages;	
	
	printf("SetParameter FirstPage[%d] NbOfPages[%d]\r\n",FirstPage, NbOfPages);
 
  /* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
     you have to make sure that these data are rewritten before they are accessed during code
     execution. If this cannot be done safely, it is recommended to flush the caches by setting the
     DCRST and ICRST bits in the FLASH_CR register. */
  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
  {
    /*
      Error occurred while page erase.
      User can add here some code to deal with this error.
      PageError will contain the faulty page and then to know the code error on this page,
      user can call function 'HAL_FLASH_GetError()'
    */
		printf("HAL_FLASHEx_Erase errcode[%d]\r\n",HAL_FLASH_GetError());
  }
  /* Program the user Flash area word by word
    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

  Address = FLASH_USER_START_ADDR;
	for(i=0 ;i<sizeof(struct Projector_parameter)/8 ;i++)
	{
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, *projector_Config) == HAL_OK)
		{
			Address = Address + 8;
			projector_Config = projector_Config + 1;
		} else {
		/* Error occurred while writing data in Flash memory.
			 User can add here some code to deal with this error */
			printf("HAL_FLASH_Program errcode[%d]\r\n",HAL_FLASH_GetError());				
		}
	}

  /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();
	
	return;
}


HAL_StatusTypeDef SetBootPinMode(void)
{
	FLASH_OBProgramInitTypeDef pOBInit;
	HAL_StatusTypeDef ret = HAL_ERROR;
	
	memset(&pOBInit,0,sizeof(FLASH_OBProgramInitTypeDef));
	pOBInit.WRPArea = OB_WRPAREA_ZONE_A;
	HAL_FLASHEx_OBGetConfig(&pOBInit);
	printf("get---RDPLevel:0x%x OPTIONTYPE:0x%x USERConfig:0x%x\r\n", pOBInit.RDPLevel, pOBInit.OptionType, pOBInit.USERConfig);
	if(GET_BIT(pOBInit.USERConfig, FLASH_OPTR_nBOOT_SEL_Pos) == 0) {
		return HAL_OK;
	}
	
	pOBInit.OptionType = OPTIONBYTE_USER;
	pOBInit.USERType = OB_USER_nBOOT_SEL;
	pOBInit.USERConfig = OB_BOOT0_FROM_PIN;
	ret = HAL_FLASH_Unlock();
	if(HAL_OK != ret)
	{
			printf("Flash Unlock fail \r\n");
			return ret;
	}	
	ret = HAL_FLASH_OB_Unlock();
	if(HAL_OK != ret)
	{
			printf("ob Unlock fail \r\n");
			return ret;
	}
	
	ret = HAL_FLASHEx_OBProgram(&pOBInit);
	if(HAL_OK != ret)
	{
			printf("HAL_FLASHEx_OBProgram fail:%d \r\n",ret);
			return ret;
	}

	ret = HAL_FLASH_OB_Lock();
	if(HAL_OK != ret)
	{
			printf("ob Unlock fail \r\n");
			return ret;
	}
	ret = HAL_FLASH_Lock();
	if(HAL_OK != ret)
	{
			printf("flash Unlock fail:%d \r\n",ret);
			return ret;
	}
	
	return HAL_OK;
}

/* usart ------------------------------------------------------------------*/
#ifdef __GUNC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);	
	return ch;
}

unsigned short checksum(unsigned char* pdata, unsigned short size)
{
	unsigned short sum = 0;

	for (int i = 0; i < size; i++)
		sum += *pdata++;

	return sum;
}


void Uart_Cmd_WriteReg(uint16_t cmd, uint16_t reg, uint8_t* val_buf, uint8_t count)
{
		uint8_t ret;
	
		if(count == 1)
		{
			if(cmd == CMD_WRITE_SXMB241_REG)			
				ret = I2cWriteSxmb241(SXRD241_I2C_ADDRESS, reg, val_buf[0]);
			else
				ret = I2cWriteCxd3554(CXD3554_I2C_ADDRESS, reg, val_buf[0]);
		}
		else
			
		{
			if(cmd == CMD_WRITE_SXMB241_REG)
			{				
				for(uint8_t i = 0; i < count; i++)
				{
					ret = I2cWriteSxmb241(SXRD241_I2C_ADDRESS, reg+i, val_buf[i]);
				}
			}
			else
				ret = I2cWriteCxd3554Burst(CXD3554_I2C_ADDRESS, reg, val_buf, count);
		}
		
		if(ret != HAL_OK)
			Uart_Send_Response(CMD_ERROR, NULL, 0);	
		else		
			Uart_Send_Response(cmd, NULL, 0);		
}

void Uart_Cmd_ReadReg(uint16_t cmd, uint16_t reg, uint8_t* val_buf, uint8_t count)
{
		uint8_t ret;
		uint8_t *pBuf = val_buf;
#if 0	
		if(count == 1)
		{
			if(cmd == CMD_READ_SXMB241_REG)
				ret = I2cReadSxmb241(SXRD241_I2C_ADDRESS, reg, &val_buf[0]);
			else
				ret = I2cReadCxd3554(CXD3554_I2C_ADDRESS, reg, &val_buf[0]);
		}
		else
#endif			
		{
			if(cmd == CMD_READ_SXMB241_REG)
			{
				for(uint8_t i = 0; i < count; i++)
				{
					ret = I2cReadSxmb241(SXRD241_I2C_ADDRESS, reg+i, &val_buf[i]);
				}	
			}
			else
			{
				for(uint16_t i = 0; i < count; i++)
				{
					ret = I2cReadCxd3554Ex(CXD3554_I2C_ADDRESS, reg+i, &val_buf[i]);
				}
			}
		}
		
		if(ret != HAL_OK)
			Uart_Send_Response(CMD_ERROR, NULL, 0);	
		else
			Uart_Send_Response(cmd, pBuf, count);	
}	

uint8_t Uart_Set_RGB_Current(uint16_t current_r, uint16_t current_g, uint16_t current_b)
{
		uint8_t ret = SetRedCurrent(current_r);
		if(ret != HAL_OK)
		{				
			return ret;
		}
		
		ret = SetGreenCurrent(current_g);
		if(ret != HAL_OK)
		{				
			return ret;
		}
		
		ret = SetBlueCurrent(current_b);
		if(ret != HAL_OK)
		{				
			return ret;
		}
		
		return HAL_OK;
}

void Uart_Save_Parameter(uint8_t* pData)
{
		uint8_t data_len = pData[PACKAGE_DATA_BASE - 3];
		uint8_t type = pData[PACKAGE_DATA_BASE];
	
		switch(type)
		{
			case PARA_CURRENT:
			{
				if(data_len == 1)//user parameter
				{
					g_projector_para.current.valid = PARAMETER_VALID;
					g_projector_para.current.index = CURRENT_USER;
					g_projector_para.current.r[CURRENT_USER] = g_RGBCurrent[0];
					g_projector_para.current.g[CURRENT_USER] = g_RGBCurrent[1];
					g_projector_para.current.b[CURRENT_USER] = g_RGBCurrent[2];	
				}	else if(data_len == 10){//tunning parameter
					g_projector_para.current.valid = PARAMETER_VALID;
					g_projector_para.current.index = CURRENT_NORMAL;
					
					for (uint8_t i=0; i<3; i++)
					{
						g_projector_para.current.r[i] = pData[PACKAGE_DATA_BASE + i*3 + 1];
						g_projector_para.current.g[i] = pData[PACKAGE_DATA_BASE + i*3 + 2];
						g_projector_para.current.b[i] = pData[PACKAGE_DATA_BASE + i*3 + 3];	
					}	
				}
				
				Uart_Set_RGB_Current(g_projector_para.current.r[g_projector_para.current.index], 
														g_projector_para.current.g[g_projector_para.current.index],g_projector_para.current.b[g_projector_para.current.index]);				
				break;
			}
			
			case PARA_FLIP:
			{
				g_projector_para.flip.valid = PARAMETER_VALID;
				I2cReadCxd3554Ex(CXD3554_I2C_ADDRESS, 0x0090, &g_projector_para.flip.h);
				I2cReadCxd3554Ex(CXD3554_I2C_ADDRESS, 0x0091, &g_projector_para.flip.v);
				break;
			}

			case PARA_KST:
			{
				g_projector_para.kst.valid = PARAMETER_VALID;
				for(uint16_t i = 0; i < KST_REG_NUM; i++) {
					I2cReadCxd3554Ex(CXD3554_I2C_ADDRESS, 0x1A49+i, &g_projector_para.kst.val[i]);
				}
#if 0
				printf("set kst:");
				for(uint8_t i = 1; i < KST_REG_NUM+1; i++)
				{
					printf("0x%x ",g_projector_para.kst.val[i]);
				}	
				printf("\r\n");
#endif
				break;
			}	
			
			case PARA_GAMA:
			{
				g_projector_para.gama.valid = PARAMETER_VALID;
				for(uint8_t i = 0; i < GAMA_REG_NUM; i++) {
					I2cReadSxmb241(SXRD241_I2C_ADDRESS, 0x58 + i, &g_projector_para.gama.val[i]);
				}
				break;
			}		
			
			case PARA_WP:
			{
				g_projector_para.wp.valid = PARAMETER_VALID;
				for(uint16_t i = 0; i < WP_REG_NUM; i++) {
					I2cReadCxd3554Ex(CXD3554_I2C_ADDRESS, 0x3500+i, &g_projector_para.wp.val[i]);
				}
				
				break;
			}		

			
		default:
			break;
		}
		
		SetParameter(&g_projector_para);	
}

void ToolUartCmdHandler(uint8_t *pRx,uint8_t length)
{
	uint8_t ret = HAL_OK;
	uint8_t buf[UART_BUFFER_MAX_SIZE];
  uint16_t reg, reg_count;
	uint16_t check;
/*-------------------------------------------------------------------------------------------*/
	struct PACKAGE_HEAD *	head = ( struct PACKAGE_HEAD *)(pRx);	
	memset(buf, 0, UART_BUFFER_MAX_SIZE);
	if (head->flag != COMM_FLAG)
	{
		Uart_Send_Response(CMD_ERROR, NULL, 0);	
		printf(" flag error!: 0x%x\n\r", head->flag);		
		return;
	}
	
	check = checksum((uint8_t *)&pRx[PACKAGE_DATA_BASE], head->size);
	if (head->check != check)
	{
		Uart_Send_Response(CMD_ERROR, NULL, 0);	
		printf(" checksum error!: 0x%x cal_sum:0x%x \n\r", head->check, check);		
		return;
	}
/*-------------------------------------------------------------------------------------------*/	
	switch(head->command)
	{	
		case CMD_ECHO:
		{
			Uart_Send_Response(head->command, &pRx[PACKAGE_DATA_BASE], head->size);
			break;
		}
		
		case CMD_WRITE_CXD3554_REG:		
		case CMD_WRITE_SXMB241_REG:
		{
			reg = (pRx[PACKAGE_DATA_BASE+1] << 8) + pRx[PACKAGE_DATA_BASE];
			reg_count = head->size - 2;
			Uart_Cmd_WriteReg(head->command, reg, (uint8_t *)&pRx[PACKAGE_DATA_BASE + 2], reg_count);
			break;
		}
		
		case CMD_READ_CXD3554_REG:
		case CMD_READ_SXMB241_REG:
		{
			reg = (pRx[PACKAGE_DATA_BASE+1] << 8) + pRx[PACKAGE_DATA_BASE];			
			reg_count = pRx[PACKAGE_DATA_BASE+2];
			Uart_Cmd_ReadReg(head->command, reg, buf, reg_count);
			break;
		}	

		case CMD_SET_COLOR_TEMP:
		{
			if(pRx[PACKAGE_DATA_BASE] < CURRENT_MAX)
			{
				g_projector_para.current.index = pRx[PACKAGE_DATA_BASE];
				ret = Uart_Set_RGB_Current(g_projector_para.current.r[g_projector_para.current.index], 
							g_projector_para.current.g[g_projector_para.current.index],g_projector_para.current.b[g_projector_para.current.index]);				
			}
			
			if(ret != HAL_OK || pRx[PACKAGE_DATA_BASE] >= CURRENT_MAX)
			{				
				Uart_Send_Response(CMD_ERROR, NULL, 0);
				break;
			}
			
			Uart_Send_Response(head->command, NULL, 0);
			
			SetParameter(&g_projector_para);
			break;			
		}	
		
		case CMD_SET_CURRENTS:
		{
			uint8_t ret = Uart_Set_RGB_Current(pRx[PACKAGE_DATA_BASE], pRx[PACKAGE_DATA_BASE + 1], pRx[PACKAGE_DATA_BASE + 2]);
			if(ret != HAL_OK)
			{				
				Uart_Send_Response(CMD_ERROR, NULL, 0);
				break;
			}
			
			Uart_Send_Response(head->command, NULL, 0);
			break;			
		}

		case CMD_SET_FANS:
		{
			g_FanMode = pRx[PACKAGE_DATA_BASE + 3];	
			if(g_FanMode != 0)
			{
				SetFan12Speed(pRx[PACKAGE_DATA_BASE]);
				SetFan34Speed(pRx[PACKAGE_DATA_BASE + 1]);
				SetFan5Speed(pRx[PACKAGE_DATA_BASE + 2]);	
			}
			Uart_Send_Response(head->command, NULL, 0);
			
			break;			
		}
		
		case CMD_SET_FOCUSMOTOR:
		{
			uint16_t steps = (pRx[PACKAGE_DATA_BASE + 2] << 8) + pRx[PACKAGE_DATA_BASE + 1];
			Motor_start(pRx[PACKAGE_DATA_BASE], steps);
			break;			
		}	
				
		case CMD_SAVE_PARAMRTER:
		{
			Uart_Save_Parameter(pRx);
			Uart_Send_Response(head->command, NULL, 0);
			break;			
		}
		case CMD_GET_VERSION:
		{
			buf[0] = VERSION0;
			buf[1] = VERSION1;
			buf[2] = VERSION2;			
			Uart_Send_Response(head->command, buf, 3);
			break;			
		}

		case CMD_GET_TEMPS:
		{
			uint16_t ld_adc = 0, lcos_adc = 0;
			uint16_t adc_val[3];

			ld_adc = adc_GetAdcVal(adc_val);
			lcos_adc = adc_val[2];
			
			buf[0] = GetLd_RT_Temp(ld_adc); //ld temp
			buf[1] = GetLcos_RT_Temp(lcos_adc); //lcos temp
			Uart_Send_Response(head->command, buf, 2);	
			break;			
		}

		case CMD_GET_FANS:
		{	
			buf[0] =  g_fan12_speed;
			buf[1] =  g_fan34_speed;
			buf[2] =  g_fan5_speed;
			buf[3] = g_FanMode;
			GetFan1Speed();
			GetFan2Speed();
			GetFan3Speed();
			GetFan4Speed();
			GetFan5Speed();			
			//GetThreePMotorSpeed();
			Uart_Send_Response(head->command, buf, 4);			
			break;			
		}

		case CMD_GET_CURRENTS:
		{
			buf[0] =  GetRGBCurrent(0);
			buf[1] =  GetRGBCurrent(1);
			buf[2] =  GetRGBCurrent(2);
			Uart_Send_Response(head->command, buf, 3);
			break;			
		}

		case CMD_GET_COLOR_TEMP:
		{
			buf[0] =  g_projector_para.current.index;
			Uart_Send_Response(head->command, buf, 1);
			break;			
		}
		
		case CMD_ENTER_MAT:
		{
			Flag_MatMode = 1;
			Uart_Send_Response(head->command, NULL, 0);
			break;				
		}	
		
		default:
			break;
	}	
}

//#define PRINT_RECV_LOG
//#define PRINT_SEND_LOG
void Uart_Send_Response(uint16_t command, uint8_t* data, uint8_t size )
{
	struct PACKAGE_HEAD head;

	head.flag = COMM_FLAG;
	head.command = command;
	head.size = size;
	head.check = 0;
	head.check = checksum(data, size);		
#ifdef PRINT_SEND_LOG
	printf("\r\n send head:");	
	uint8_t* p = (uint8_t* )&head;
	for(uint8_t i=0;i<sizeof(struct PACKAGE_HEAD);i++)
	{
		printf("0x%x ",p[i]);
	}
	printf("\r\n");
#endif	
	
	HAL_UART_Transmit(&huart1, (uint8_t*)&head,  sizeof(struct PACKAGE_HEAD), 100);
	HAL_UART_Transmit(&huart1, data,  size, 100);
	
#ifdef PRINT_SEND_LOG
	printf("\r\n send data:");	
	for(uint8_t i=0;i<size;i++)
	{
		printf("0x%02x ",data[i]);
	}
	printf("\r\n");
#endif
}

void UartCmdHandler(uint8_t *pRx,uint8_t length)
{
	I2C_HandleTypeDef hi2c;
  uint16_t i;
	uint16_t frame_head = *(uint16_t*)pRx;

#ifdef PRINT_RECV_LOG
	printf("\n\r UartCmdHandler %d Bytes:",length);
	for(uint16_t i = 0; i < length; i++)
	{
		printf(" 0x%02X", pRx[i]);
	}
	printf("\n\r");
#endif
	
	if(frame_head == COMM_FLAG)
	{
		ToolUartCmdHandler(pRx, length);
		return;
	} else 	
/*-------------------------------------------------------------------------------------------*/	
	{
		int para0_len,para1_len,read_len;
		unsigned char temp;
		
		memset(&receive_buffer,0,40);
		para0_len = 0;
		para1_len = 0;
		memcpy(receive_buffer,pRx,UartReceiveLength);
		read_len = read_evm(receive_buffer,UartReceiveLength,&para0_len,&para1_len);
		para_Analysis((char*)(&receive_buffer),para0_len,para1_len);

		if(asu_rec_data.w_r == 1)
		{
			if(asu_rec_data.chip_addr == CXD3554_I2C_ADDRESS)
			{
/*				
						uint16_t temp_value;
						temp_value = (asu_rec_data.reg_value+1)*4-1;
						if(asu_rec_data.reg_addr==0x3216)
						{
							SetRedCurrent(temp_value);
							g_red_value	= asu_rec_data.reg_value;
						}
						else if(asu_rec_data.reg_addr==0x3217)
						{
							SetGreenCurrent(temp_value);
							g_green_value	= asu_rec_data.reg_value;
						}
						else if(asu_rec_data.reg_addr==0x3218)
						{
							SetBlueCurrent(temp_value);
							g_blue_value	= asu_rec_data.reg_value;
						}
						else
*/
						{
							I2cWriteCxd3554(CXD3554_I2C_ADDRESS, asu_rec_data.reg_addr, asu_rec_data.reg_value);
						}
			}
			else if (asu_rec_data.chip_addr == SXRD241_I2C_ADDRESS)
			{
						I2cWriteSxmb241(SXRD241_I2C_ADDRESS, asu_rec_data.reg_addr, asu_rec_data.reg_value);
			}
			memset(&asu_rec_data,0x00,sizeof(struct asu_date));
		}

		if(asu_rec_data.w_r == 2)
		{
			if(asu_rec_data.chip_addr == CXD3554_I2C_ADDRESS)
			{
				I2cReadCxd3554Ex(CXD3554_I2C_ADDRESS, asu_rec_data.reg_addr, (uint8_t *)&asu_rec_data.reg_value);
			}
			else
				I2cReadSxmb241(SXRD241_I2C_ADDRESS, asu_rec_data.reg_addr, (uint8_t *)&asu_rec_data.reg_value);
		}

		else if(asu_rec_data.w_r == 5)
		{
			if(asu_rec_data.chip_addr == CXD3554_I2C_ADDRESS)
			{
/*				
					uint16_t temp_value;
					temp_value = (asu_rec_data.reg_value+1)*4-1;
					if(asu_rec_data.reg_addr==0x3216)
					{
						SetRedCurrent(temp_value);
						g_red_value	= asu_rec_data.reg_value;
					}
					else if(asu_rec_data.reg_addr==0x3217)
					{
						SetGreenCurrent(temp_value);
						g_green_value	= asu_rec_data.reg_value;
					}
					else if(asu_rec_data.reg_addr==0x3218)
					{
						SetBlueCurrent(temp_value);
						g_blue_value = asu_rec_data.reg_value;
					}
					else
*/
					{
						I2cWriteCxd3554(CXD3554_I2C_ADDRESS, asu_rec_data.reg_addr, asu_rec_data.reg_value);			
					}
			}
			else if (asu_rec_data.chip_addr == SXRD241_I2C_ADDRESS)
			{
			      I2cWriteSxmb241(SXRD241_I2C_ADDRESS, asu_rec_data.reg_addr, asu_rec_data.reg_value);
			}
			memset(&asu_rec_data,0x00,sizeof(struct asu_date));
		}
		if((asu_rec_data.w_r == 3) || (asu_rec_data.w_r == 4))
		{

			if(asu_rec_data.chip_addr == CXD3554_I2C_ADDRESS)
			{
/*				
						if(asu_rec_data.reg_addr==0x3216)
						{
							asu_rec_data.reg_value = g_red_value;
						}
						else if(asu_rec_data.reg_addr==0x3217)
						{
							asu_rec_data.reg_value = g_green_value;
						}
						else if(asu_rec_data.reg_addr==0x3218)
						{
							asu_rec_data.reg_value = g_blue_value;
						}
						else
*/				
						{
							I2cReadCxd3554Ex(CXD3554_I2C_ADDRESS, asu_rec_data.reg_addr, (uint8_t *)&asu_rec_data.reg_value);	
						}
			}
			else if (asu_rec_data.chip_addr == SXRD241_I2C_ADDRESS)
			{
							I2cReadSxmb241(SXRD241_I2C_ADDRESS, asu_rec_data.reg_addr, (uint8_t *)&asu_rec_data.reg_value);
			}
		}
		show_data(&asu_rec_data);
		if((receive_buffer[0]=='R') ||(receive_buffer[0]=='r'))
		{
			if (receive_buffer[2] =='0')
			{
				HAL_UART_Transmit(&huart1, (uint8_t *)(&R_0), 15, 100);
			}
			else if(receive_buffer[2] =='1')
			{
				HAL_UART_Transmit(&huart1, (uint8_t *)(&R_1), 13, 100);
			}
			else if(receive_buffer[2] =='2')
			{
				HAL_UART_Transmit(&huart1, (uint8_t *)(&R_2), 15, 100);
			}
			else if (((receive_buffer[2] == '9') && (receive_buffer[3] == '2'))|| ((receive_buffer[2] == '8') && (receive_buffer[3] == 'c')) )
			{

				for(i = 0; i < 8 ;i++)
				{
					receive_buffer[read_len-2+i] = OK_0[i];
				}

				receive_buffer[read_len+4] = 32;
				if(data_len == 1)
				{
					temp = asu_rec_data.reg_value;
					receive_buffer[read_len+5] = HexToChar(temp);
				}

				if(data_len == 2)
				{
					temp = asu_rec_data.reg_value;
					receive_buffer[read_len+5] = HexToChar(temp/16);
					receive_buffer[read_len+6] = HexToChar(temp%16);
				}

				if(data_len == 4)
				{
					temp = asu_rec_data.reg_value >>8;
					receive_buffer[read_len+5] = HexToChar(temp/16);
					receive_buffer[read_len+6] = HexToChar(temp%16);
					temp = asu_rec_data.reg_value & 0xff;
					receive_buffer[read_len+7] = HexToChar(temp/16);
					receive_buffer[read_len+8] = HexToChar(temp%16);
				}
				receive_buffer[read_len -4 +8 +1 + data_len+1] = 13;
				receive_buffer[read_len -4 +8 +1 + data_len+2] = 10;
				HAL_UART_Transmit(&huart1, (uint8_t *)(&receive_buffer), read_len -2 + 8 +1 + data_len+2, 100);
				memset(&asu_rec_data,0x00,sizeof(struct asu_date));
			}
			else if ((receive_buffer[2] == '8') && ((receive_buffer[3] == '7') ||(receive_buffer[3] == 'd')) && (flag_0 != 0))
			{
				for(i=0;i<10;i++)
				{
					if(flag_0 == '0')
						receive_buffer[read_len -2 + i] = OK_1[i];
					if(flag_0 == '2')
						receive_buffer[read_len -2 + i] = OK_3[i];
					if(flag_0 == '6')
						receive_buffer[read_len -2 + i] = OK_7[i];
				}
				flag_0 = 0 ;
				HAL_UART_Transmit(&huart1, (uint8_t *)(&receive_buffer), read_len+8, 100);
			}
			else
			{
				for(i = 0; i < 10 ;i++)
					receive_buffer[read_len -2 + i] = OK_1[i];
				HAL_UART_Transmit(&huart1, (uint8_t *)(&receive_buffer), read_len+8, 100);
			}
		}
		else if (receive_buffer[0] == 'W' || receive_buffer[0] == 'w')
		{
			if((receive_buffer[2] == '8') && ((receive_buffer[3] == '7') || (receive_buffer[3] == 'd')) && (receive_buffer[5] == '1'))
			{
				flag_0 = '0' ;
			}
			else if((receive_buffer[2] == '8') && ((receive_buffer[3] == '7') || (receive_buffer[3] == 'd')) && (receive_buffer[5] == '7'))
			{
				flag_0 = '6' ;
			}
			else if((receive_buffer[2] == '8') && ((receive_buffer[3] == '7') || (receive_buffer[3] == 'd')) && (receive_buffer[5] == '3'))
			{
				flag_0 = '2' ;
			}
			for(i = 0; i < 8 ;i++)
				receive_buffer[read_len-2+i] = OK_0[i];
				HAL_UART_Transmit(&huart1, (uint8_t *)(&receive_buffer), read_len+6, 100);
		}

		else if(((receive_buffer[0]=='p') && (receive_buffer[1]=='r')) || ((receive_buffer[0]=='h') && (receive_buffer[1]=='r')) || ((receive_buffer[0]=='m') && (receive_buffer[1]=='r')) )
		{
			for(i = 0; i < 8 ;i++)
				receive_buffer[read_len-2+i] = OK_0[i];

			receive_buffer[read_len+4] = 32;
			if(data_len == 1)
			{
				temp = asu_rec_data.reg_value;
				receive_buffer[read_len+5] = HexToChar(temp);
			}
			if(data_len == 2)
			{
				temp = asu_rec_data.reg_value;
				receive_buffer[read_len+5] = HexToChar(temp/16);
				receive_buffer[read_len+6] = HexToChar(temp%16);
			}
			if(data_len == 4)
			{
				temp = asu_rec_data.reg_value >>8;
				receive_buffer[read_len+5] = HexToChar(temp/16);
				receive_buffer[read_len+6] = HexToChar(temp%16);
				temp = asu_rec_data.reg_value & 0xff;
				receive_buffer[read_len+7] = HexToChar(temp/16);
				receive_buffer[read_len+8] = HexToChar(temp%16);
			}
			receive_buffer[read_len -4 +8 +1 + data_len+1] = 13;
			receive_buffer[read_len -4 +8 +1 + data_len+2] = 10;
			HAL_UART_Transmit(&huart1, (uint8_t *)(&receive_buffer), read_len -2 + 8 +1 + data_len+2, 100);
			memset(&asu_rec_data,0x00,sizeof(struct asu_date));
		}

		else if(((receive_buffer[0]=='p') && (receive_buffer[1]=='w')) || ((receive_buffer[0]=='h') && (receive_buffer[1]=='w')) || ((receive_buffer[0]=='m') && (receive_buffer[1]=='w')) )
		{
			for(i = 0; i < 8 ;i++)
				receive_buffer[read_len-2+i] = OK_0[i];
			HAL_UART_Transmit(&huart1, (uint8_t *)(&receive_buffer), read_len+6, 100);
			memset(&asu_rec_data,0x00,sizeof(struct asu_date));
		}
	}
}

void ReceiveUart1Data(void)
{
	HAL_UART_Receive_DMA(&huart1, (uint8_t *)UartReceiveRxBuffer, UART_BUFFER_MAX_SIZE);	
}

void UartCommandParser(void)
{
	int i = 0;
	
	if(UartTempLength > 0) {
#if 0		
		for (i=0;i<UartTempLength;i++)
		{
				if(UartTempBuffer[i] == 0xFE) break;
		}
		if(i == UartTempLength) 
		{
			Uart_Send_Response(CMD_ERROR, NULL, 0);	
			return; //no 0xFE return 
		}
#endif
		UartCommandLength = UartTempLength - i;
		memset((void*)UartCommandBuffer,0x00,UART_BUFFER_MAX_SIZE);	
		memcpy((void*)UartCommandBuffer, (void*)&UartTempBuffer[i], UartCommandLength);
		
		UartTempLength = 0;
		UartCmdHandler((uint8_t *)UartCommandBuffer,UartCommandLength);
	}
}

void HAL_UART_AbortReceiveCpltCallback (UART_HandleTypeDef *huart)
{

	memset((void*)UartTempBuffer,0x00,UART_BUFFER_MAX_SIZE);	
	memcpy((void*)UartTempBuffer, (void*)UartReceiveRxBuffer, UartReceiveLength);
	UartTempLength = UartReceiveLength;
	
	//printf("HAL_UART_AbortReceiveCpltCallback len=%d\r\n",UartReceiveLength);
	//Re-start receiving
	ReceiveUart1Data();
	/* NOTE : This function should not be modified, when the callback is needed,
	the HAL_UART_AbortTransmitCpltCallback can be implemented in the user file.
	*/
}	

/* lcos ------------------------------------------------------------------*/

void LcosSetGain(void)
{
		if((g_projector_para.gain.valid) == PARAMETER_VALID)
		{
			if(g_projector_para.gain.r < 155)
				g_projector_para.gain.r = 255;
			if(g_projector_para.gain.g < 155)
				g_projector_para.gain.g = 255;
			if(g_projector_para.gain.b < 155)
				g_projector_para.gain.b = 255;

			I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x3216, g_projector_para.gain.r);
			I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x3217, g_projector_para.gain.g);
			I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x3218, g_projector_para.gain.b);
		}

}

void LcosSetGamma(void)
{
		uint16_t i;
	
		if((g_projector_para.gama.valid) == PARAMETER_VALID)
		{
			for(i = 0; i < GAMA_REG_NUM; i++) {
				I2cWriteSxmb241(SXRD241_I2C_ADDRESS, 0x58 + i, g_projector_para.gama.val[i]);
				printf("reg[%d]=0x%x\r\n",i+58, g_projector_para.gama.val[i]);
			}
		}
		
#if 0		
		uint8_t reg_data;
		for(i=0; i<GAMA_REG_NUM; i++) {
			I2cReadSxmb241(SXRD241_I2C_ADDRESS, 0x58+i, &reg_data);
			printf("reg[%d]=%d\r\n",i+58, reg_data);
		}	
#endif			
}

void LcosSetFlip(void)
{
		if(g_projector_para.flip.valid == PARAMETER_VALID)
		{
			I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x0090, (H_FLIP_Mode)(g_projector_para.flip.h));
			I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x0091, (V_FLIP_Mode)(g_projector_para.flip.v));
		} else {
			I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x0090, 0x01); //default val
			I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x0091, 0x01);		
		}
}

void LcosSetKst(void)
{
		if(g_projector_para.kst.valid == PARAMETER_VALID)
		{
			I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A40, 0x6d);
			I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A41, 0x68);
#if 0			
			printf("get kst:");
			for(uint8_t i = 0; i < KST_REG_NUM; i++)
			{
				printf("0x%x ",g_projector_para.kst.val[i]);
			}	
			printf("\r\n");
#endif			
			I2cWriteCxd3554Burst(CXD3554_I2C_ADDRESS, 0x1A49, g_projector_para.kst.val, KST_REG_NUM);
		}
}

void LcosSetWP(void)
{
		I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A40,0x6d);
		I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A42,0x30);
	
		if(g_projector_para.wp.valid != PARAMETER_VALID)
		{
			memset(g_projector_para.wp.val, 0, WP_REG_NUM);
		}

		I2cWriteCxd3554Burst(CXD3554_I2C_ADDRESS, 0x3500, g_projector_para.wp.val, WP_REG_NUM);
}
/* tim ------------------------------------------------------------------*/
uint8_t FanSpeedConvert(uint32_t speed)
{
	 return (39*speed)/100;
}
uint8_t SetFan12Speed(uint32_t speed)
{
  if(speed > FAN_SPEED_FULL)	speed = FAN_SPEED_FULL;
	if(g_fan12_speed == (uint16_t)speed) return 0;
	g_fan12_speed = (uint16_t)speed;
	
  TIM_OC_InitTypeDef sConfigOC={0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = FanSpeedConvert(speed);
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	
  return 0;
}

uint8_t SetFan34Speed(uint32_t speed)
{
  if(speed > FAN_SPEED_FULL)	speed = FAN_SPEED_FULL;
	if(g_fan34_speed == (uint16_t)speed) return 0;
	g_fan34_speed = (uint16_t)speed;
	
  TIM_OC_InitTypeDef sConfigOC={0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = FanSpeedConvert(speed);
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  return 0;
}
uint8_t SetFan5Speed(uint32_t speed)
{
  if(speed > FAN_SPEED_FULL)	speed = FAN_SPEED_FULL;
	if(g_fan5_speed == (uint16_t)speed) return 0;
	g_fan5_speed = (uint16_t)speed;
	
  TIM_OC_InitTypeDef sConfigOC={0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = FanSpeedConvert(speed);
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  return 0;
}

#if 1
static uint8_t fan1_capCnt, fan2_capCnt, fan3_capCnt, fan4_capCnt, fan5_capCnt, ThreePMotor_capCnt;
static uint16_t fan1_capBuf[3], fan2_capBuf[3],fan3_capBuf[3], fan4_capBuf[3],  fan5_capBuf[3], ThreePMotor_capBuf[3];

#define TESTSPEED_BASE   30000000

uint8_t GetThreePMotorSpeed(void)
{
	uint16_t count = 1000, speed = 0, T_time;
	
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim14, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
		HAL_TIM_IC_Start_IT(&htim14, TIM_CHANNEL_1);		
		ThreePMotor_capCnt = 1;
	
	/* waiting convert complete */
	while(count && ThreePMotor_capCnt != 4 ) {
		HAL_Delay(1);
		count--;
	}
	
	if(count != 0)
	{
		T_time =  ThreePMotor_capBuf[2] - ThreePMotor_capBuf[0];
		speed = TESTSPEED_BASE/T_time;
	}
	
	printf("GetThreePMotorSpeed  speed:%d T_time:%d count:%d \r\n", speed, T_time, count);
	return speed;
}

uint8_t GetFan1Speed(void) //PA8 timer1_ch1
{
	uint16_t count = 1000, speed = 0, T_time;
	
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
		HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);		
		fan1_capCnt = 1;
	
	/* waiting convert complete */
	while(count && fan1_capCnt != 4 ) {
		HAL_Delay(1);
		count--;
	}
	
	if(count != 0)
	{
		T_time =  fan1_capBuf[2] - fan1_capBuf[0];
		speed = TESTSPEED_BASE/T_time;
	}

	printf("GetFan1Speed  speed:%d T_time:%d count:%d \r\n", speed, T_time, count);
	return speed;
}

uint8_t GetFan2Speed(void) //PB3 timer1_ch2
{
	uint16_t count = 1000, speed = 0, T_time;
	
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
		HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);		
		fan2_capCnt = 1;
	
	/* waiting convert complete */
	while(count && fan2_capCnt != 4 ) {
		HAL_Delay(1);
		count--;
	}
	
	if(count != 0)
	{
		T_time =  fan2_capBuf[2] - fan2_capBuf[0];
		speed = TESTSPEED_BASE/T_time;
	}

	printf("GetFan2Speed  speed:%d T_time:%d count:%d \r\n", speed, T_time, count);	
	return speed;
}
uint8_t GetFan3Speed(void) //PC10 timer1_ch3
{
	uint16_t count = 1000, speed = 0, T_time;
	
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
		HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);		
		fan3_capCnt = 1;
	
	/* waiting convert complete */
	while(count && fan3_capCnt != 4 ) {
		HAL_Delay(1);
		count--;
	}
	
	if(count != 0)
	{
		T_time =  fan3_capBuf[2] - fan3_capBuf[0];
		speed = TESTSPEED_BASE/T_time;
	}

	printf("GetFan3Speed  speed:%d T_time:%d count:%d \r\n", speed, T_time, count);	
	return speed;
}

uint8_t GetFan4Speed(void) //PC11 timer1_ch4
{
	uint16_t count = 1000, speed = 0, T_time;
	
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
		HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);		
		fan4_capCnt = 1;
	
	/* waiting convert complete */
	while(count && fan4_capCnt != 4 ) {
		HAL_Delay(1);
		count--;
	}
	
	if(count != 0)
	{
		T_time =  fan4_capBuf[2] - fan4_capBuf[0];
		speed = TESTSPEED_BASE/T_time;
	}
	
	printf("GetFan4Speed  speed:%d T_time:%d count:%d \r\n", speed, T_time, count);	
	return speed;
}

uint8_t GetFan5Speed(void) //PC1 timer15_ch1
{
	uint16_t count = 1000, speed = 0, T_time;
	
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim15, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
		HAL_TIM_IC_Start_IT(&htim15, TIM_CHANNEL_1);		
		fan5_capCnt = 1;
	
	/* waiting convert complete */
	while(count && fan5_capCnt != 4 ) {
		HAL_Delay(1);
		count--;
	}
	
	if(count != 0)
	{
		T_time =  fan5_capBuf[2] - fan5_capBuf[0];
		speed = TESTSPEED_BASE/T_time;
	}
	
	printf("GetFan5Speed  speed:%d T_time:%d count:%d \r\n", speed, T_time, count);	
	return speed;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(TIM14 == htim->Instance)
	{
		//printf("HAL_TIM_IC_CaptureCallback  ThreePMotor_capCnt:%d !!! \r\n", ThreePMotor_capCnt);
		switch(ThreePMotor_capCnt){
			case 1:
				ThreePMotor_capBuf[0] = HAL_TIM_ReadCapturedValue(&htim14,TIM_CHANNEL_1);
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim14,TIM_CHANNEL_1,TIM_ICPOLARITY_FALLING);
				ThreePMotor_capCnt++;
				break;
			
			case 2:
				ThreePMotor_capBuf[1] = HAL_TIM_ReadCapturedValue(&htim14,TIM_CHANNEL_1);
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim14, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
				ThreePMotor_capCnt++;  
				break;		
			
			case 3:
				ThreePMotor_capBuf[2] = HAL_TIM_ReadCapturedValue(&htim14,TIM_CHANNEL_1);
				HAL_TIM_IC_Stop_IT(&htim14,TIM_CHANNEL_1);
				ThreePMotor_capCnt++;  
				break;
		}			
	}
	
	if(TIM1 == htim->Instance)
	{
		//printf("HAL_TIM_IC_CaptureCallback  fan1_capCnt:%d fan2_capCnt:%d fan3_capCnt:%d fan4_capCnt:%d !!! \r\n", fan1_capCnt,fan2_capCnt,fan3_capCnt,fan4_capCnt);
		switch(fan1_capCnt){
			case 1:
				fan1_capBuf[0] = HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_1);
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim1,TIM_CHANNEL_1,TIM_ICPOLARITY_FALLING);
				fan1_capCnt++;
				break;
			
			case 2:
				fan1_capBuf[1] = HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_1);
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
				fan1_capCnt++;  
				break;		
			
			case 3:
				fan1_capBuf[2] = HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_1);
				HAL_TIM_IC_Stop_IT(&htim1,TIM_CHANNEL_1);
				fan1_capCnt++;  
				break;
		}	
		
		switch(fan2_capCnt){
			case 1:
				fan2_capBuf[0] = HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_2);
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim1,TIM_CHANNEL_2,TIM_ICPOLARITY_FALLING);
				fan2_capCnt++;
				break;
			
			case 2:
				fan2_capBuf[1] = HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_2);
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
				fan2_capCnt++;  
				break;		
			
			case 3:
				fan2_capBuf[2] = HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_2);
				HAL_TIM_IC_Stop_IT(&htim1,TIM_CHANNEL_2);
				fan2_capCnt++;  
				break;
		}	

		switch(fan3_capCnt){
			case 1:
				fan3_capBuf[0] = HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_3);
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim1,TIM_CHANNEL_3,TIM_ICPOLARITY_FALLING);
				fan3_capCnt++;
				break;
			
			case 2:
				fan3_capBuf[1] = HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_3);
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
				fan3_capCnt++;  
				break;		
			
			case 3:
				fan3_capBuf[2] = HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_3);
				HAL_TIM_IC_Stop_IT(&htim1,TIM_CHANNEL_3);
				fan3_capCnt++;  
				break;
		}	
		
		switch(fan4_capCnt){
			case 1:
				fan4_capBuf[0] = HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_4);
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim1,TIM_CHANNEL_4,TIM_ICPOLARITY_FALLING);
				fan4_capCnt++;
				break;
			
			case 2:
				fan4_capBuf[1] = HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_4);
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
				fan4_capCnt++;  
				break;		
			
			case 3:
				fan4_capBuf[2] = HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_4);
				HAL_TIM_IC_Stop_IT(&htim1,TIM_CHANNEL_4);
				fan4_capCnt++;  
				break;
		}	
				
	}
		
	if(TIM15 == htim->Instance)
	{
		//printf("HAL_TIM_IC_CaptureCallback  fan5_capCnt:%d !!! \r\n", fan5_capCnt);
		switch(fan5_capCnt){
			case 1:
				fan5_capBuf[0] = HAL_TIM_ReadCapturedValue(&htim15,TIM_CHANNEL_1);
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim15,TIM_CHANNEL_1,TIM_ICPOLARITY_FALLING);
				fan5_capCnt++;
				break;
			
			case 2:
				fan5_capBuf[1] = HAL_TIM_ReadCapturedValue(&htim15,TIM_CHANNEL_1);
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim15, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
				fan5_capCnt++;  
				break;		
			
			case 3:
				fan5_capBuf[2] = HAL_TIM_ReadCapturedValue(&htim15,TIM_CHANNEL_1);
				HAL_TIM_IC_Stop_IT(&htim15,TIM_CHANNEL_1);
				fan5_capCnt++;  
				break;			
		}
	}
}
#endif
/* adc ------------------------------------------------------------------*/
#define ADC_CONVERSION_CNT  10
#define ADC_CHANNEL_CNT  3
#define ADC_CONVERSION_NUM 	ADC_CHANNEL_CNT * ADC_CONVERSION_CNT

/* Variable to report status of DMA transfer of ADC group regular conversions */
/*  0: DMA transfer is not completed                                          */
/*  1: DMA transfer is completed                                              */
__IO   uint8_t ubDmaTransferStatus = 0; /* Variable set into DMA interruption callback */

/**
  * @brief  Conversion complete callback in non blocking mode 
  * @param  hadc: ADC handle
  * @note   This example shows a simple way to report end of conversion
  *         and get conversion result. You can add your own implementation.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  /* Update status variable of DMA transfer */
  ubDmaTransferStatus = 1;  

}

uint16_t get_ADC_DmaValue(uint16_t ch, uint16_t* val)
{
	uint16_t aResultDMA[ADC_CONVERSION_NUM], ret, i, j;
	uint16_t count = 1000;
	static uint8_t flag_cal = 1;
	uint32_t val_sum[ADC_CHANNEL_CNT] = {0};
	
	memset(aResultDMA, 0, sizeof(aResultDMA));
	
	if(flag_cal) {
		if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) {
			printf("HAL_ADCEx_Calibration_Start faild. ret=%d \r\n", ret);
		}
		flag_cal = 0;
	}
	
	/* ###- Start conversion in DMA mode ################################# */
	ret = HAL_ADC_Start_DMA(&hadc1, (uint32_t*)aResultDMA, ADC_CONVERSION_NUM);
	if (ret != HAL_OK)
	{
		printf("HAL_ADC_Start_DMA faild. ret=%d \r\n", ret);
	}

	/* waiting convert complete */
	while(count && !ubDmaTransferStatus) {
		count--;
	}
#if 0
	printf("count=%d \r\n", count);
	if(ubDmaTransferStatus == 1) {
		for (i=0; i<ADC_CONVERSION_NUM; i++) {
			printf("aResultDMA[%d]=%d \r\n",i, aResultDMA[i]);
		}
	}
#endif
	/* Update status variable of DMA transfer */
	ubDmaTransferStatus = 0;
	HAL_ADC_Stop(&hadc1);	
	
	for (j=0; j<ADC_CHANNEL_CNT; j++)	
	{
		for (i=0; i<ADC_CONVERSION_CNT; i++)
		{
			val_sum[j] += aResultDMA[i * ADC_CHANNEL_CNT + j];
		}		
	}

	if(val != NULL) 
	{
		for (j=0; j<ADC_CHANNEL_CNT;j++)
		{
			val[j] = val_sum[j]/ADC_CONVERSION_CNT;
			//printf("ADC_DMA_VAL[%d]=%d \r\n",j, val[j]);
		}
	}
	
	return (val_sum[ch]/ADC_CONVERSION_CNT);
}

uint16_t adc_GetTsOut(void)
{
	return get_ADC_DmaValue(2, NULL);
}

uint16_t adc_GetPsOut(void)
{
	return get_ADC_DmaValue(1, NULL);
}

uint16_t adc_GetLDTemp(void)
{
	return get_ADC_DmaValue(0, NULL);
}

uint16_t adc_GetAdcVal(uint16_t *val)
{
	return get_ADC_DmaValue(0, val);	
}

extern void StepperVar_Init(void);
void Variables_Init(void)
{
	memset(&asu_rec_data,0x00,sizeof(struct asu_date));
	StepperVar_Init();
	
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
