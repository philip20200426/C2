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
/* Private variables ---------------------------------------------------------*/
uint16_t g_fan12_speed,g_fan34_speed,g_fan5_speed;
struct Projector_parameter  g_projector_para;
struct asu_date asu_rec_data;
volatile uint8_t UartReceiveRxBuffer[UART_BUFFER_MAX_SIZE] = {0};
volatile uint8_t UartReceiveLength = 0;
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

uint8_t wc_temp[438] = {0};
struct Projector_WC wc_data_temp[4];
uint8_t g_red_value,g_green_value, g_blue_value;
/*just for debug in bringup*/
uint8_t a[20] = {0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55};
uint8_t b[20] = {0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66};
uint8_t c[20] = {0xee,0xee,0xee,0xee,0xee,0xee,0xee,0xee,0xee,0xee};
#ifdef CONFIG_KST_INDEX
#include "kst.h"
#endif
extern uint32_t g_fan_value;
extern uint8_t gpiopin;
extern uint16_t g_RedCurrent;
extern uint16_t g_GreenCurrent;
extern uint16_t g_BlueCurrent;
extern volatile _Bool Flag_FanTest;
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

void show_A100_data(struct asu_date *data)
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

void A100_GetParameter(void)
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
	
	return;
}

void A100_setparameter(struct Projector_parameter  *g_projector_parameter )
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
	
	printf("A100_setparameter FirstPage[%d] NbOfPages[%d]\r\n",FirstPage, NbOfPages);
 
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


HAL_StatusTypeDef A100_SetBootPinMode(void)
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

void A100_UartCmdHandler(uint8_t *pRx,uint8_t length)
{
	uint8_t I2cBuf;
	I2C_HandleTypeDef hi2c;
  uint16_t i;
	uint16_t  checksum;
	uint16_t  MemAddSize;
	uint16_t  value0;

	struct Projector_date *recevie_data;
	struct Projector_CCT  *recevie_cct_data;
	struct Projector_Gama *recevie_gamma_data;
	struct Projector_wb_date *recevie_wb_data;
#if 0
	printf("\n\r A100_UartCmdHandler %d Bytes:",length);
	for(uint16_t i = 0; i < length; i++)
	{
		printf(" 0x%02X", pRx[i]);
	}
	printf("\n\r");
#endif	
	if(pRx[0] == A100_SET_TUNING_PARAMETER)
	{
		recevie_cct_data = (struct Projector_CCT *)(pRx);
		memcpy(&(g_projector_para.projector_tuning),recevie_cct_data,sizeof(struct Projector_CCT));
		HAL_UART_Transmit(&huart1, b,  20, 100);
		return;
	}
	else if(pRx[0] == A100_SET_WC_PARAMETER)
	{
			int i;
			recevie_wb_data = ( struct Projector_wb_date *)(pRx);
			if((recevie_wb_data->direction)<4)
			{
				wc_data_temp[recevie_wb_data->direction].power_num = recevie_wb_data->power_num;
				wc_data_temp[recevie_wb_data->direction].gain = recevie_wb_data->gain;
				wc_data_temp[recevie_wb_data->direction].offset = recevie_wb_data->offset;
				wc_data_temp[recevie_wb_data->direction].l = recevie_wb_data->l;
			}
			if((recevie_wb_data->direction)==0)
			{
				for(i=0;i<132;i++)
				{
						wc_temp[i+0xA0] = recevie_wb_data->reg[i];
				}
			}
			else if ((recevie_wb_data->direction)==1)
			{
				for(i=0;i<78;i++)
				{
						wc_temp[i] = recevie_wb_data->reg[i];
				}
			}
			else if ((recevie_wb_data->direction)==2)
			{
				for(i=0;i<132;i++)
				{
						wc_temp[i+0x130] = recevie_wb_data->reg[i];
				}
			}
			else if ((recevie_wb_data->direction)==3)
			{
				for(i=0;i<78;i++)
				{
						wc_temp[i+0x50] = recevie_wb_data->reg[i];
				}
			}
			
			A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A40,0x6c);
			A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A42,0x70);		
			for(i=0; i<438; i++)
			{
					A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS,0x3500 +i , wc_temp[i]);
			}
			A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A40,0x6d);
			A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A42,0x30);
			HAL_UART_Transmit(&huart1, c,  20, 100);
			return;
	}	
	else if(pRx[0] == A100_SET_GAMA_PARAMETER)
	{
		recevie_gamma_data = (struct Projector_Gama *)(pRx);
		memcpy(&(g_projector_para.gamma_data),recevie_gamma_data,sizeof(struct Projector_Gama));
		HAL_UART_Transmit(&huart1, a,  20, 100);
		return;
	}
	else if(pRx[0] == A100_UART_COMMAND_HEAD)
	{
	//cal checksum here
		recevie_data = (struct Projector_date *)(pRx);
		checksum = (recevie_data->command + recevie_data->data0 + recevie_data->data1 + recevie_data->data2 + recevie_data->data3 + recevie_data->data4+ recevie_data->data5) % 0x1000;
	//check STX & ETX
		if((A100_UART_COMMAND_HEAD != recevie_data->start_head) || (A100_UART_COMMAND_END != recevie_data->end_tail))
		{
			recevie_data->data5 = 101;
			HAL_UART_Transmit(&huart1, (uint8_t *)recevie_data,  sizeof(struct Projector_date), 100);
		}

		//checksum error
		if(checksum != recevie_data->checksum)
		{
			if(recevie_data->checksum != 0xffff) //universual checksum
			{
				recevie_data->data4 = checksum;
				recevie_data->data5 = 100;
				HAL_UART_Transmit(&huart1, (uint8_t *)recevie_data,  sizeof(struct Projector_date), 100);
				return;
			}
		}	
			switch(recevie_data->command)
			{		
			/*		
					case A100_SET_LED:
						switch(recevie_data->data0)
						{
							case A100_SET_LED_RRGGBBGG:
								A100_Set_LED_RRGGBBGG();
							break;

							case A100_SET_LED_RRGGRRBB:
								A100_Set_LED_RRGGRRBB();
							break;

							case A100_SET_LED_RRGGBBWW:
								A100_Set_LED_RRBBGGBB();
							break;

							case A100_SET_LED_RRBBGGBB:
								A100_Set_LED_RRBBGGBB();
							break;
						}
					break;
			*/
					case A100_GET_OE_FAN_SPEED:
						recevie_data->data0 = g_fan12_speed;	
						recevie_data->data1 = g_fan34_speed;
						recevie_data->data2 = g_fan5_speed;					
					break;
						
					case A100_SET_OE_FAN_SPEED:
						if(recevie_data->data0 == 0)
						{
							A100_SetFan12Speed(recevie_data->data1);
							A100_SetFan34Speed(recevie_data->data1);
						}
						else if(recevie_data->data0 == 1)	
							A100_SetFan5Speed(recevie_data->data1);	
						else	
						{
							A100_SetFan12Speed(recevie_data->data1);
							A100_SetFan34Speed(recevie_data->data1);
							A100_SetFan5Speed(recevie_data->data1);
						}
						
						Flag_FanTest = 1;
					break;

					case A100_SET_CURRENT_MODE:
						if(recevie_data->data0<3 && g_projector_para.projector_tuning.tuning_valid == 1 )
						{
								uint16_t Red_Current = 0x0000;
								uint16_t Green_Current = 0x0000;
								uint16_t Blue_Current = 0x0000;
							
								g_projector_para.projector_tuning.briness_index = recevie_data->data0;
								Red_Current 	= g_projector_para.projector_tuning.cct[g_projector_para.projector_tuning.cct_index].briness[g_projector_para.projector_tuning.briness_index].red_current ;
								Green_Current = g_projector_para.projector_tuning.cct[g_projector_para.projector_tuning.cct_index].briness[g_projector_para.projector_tuning.briness_index].green_current ;
								Blue_Current 	= g_projector_para.projector_tuning.cct[g_projector_para.projector_tuning.cct_index].briness[g_projector_para.projector_tuning.briness_index].blue_current ;
								A100_SetRedCurrent(Red_Current);
								A100_SetGreenCurrent(Green_Current);
								A100_SetBlueCurrent(Blue_Current);

								recevie_data->data0 = g_projector_para.projector_tuning.cct_index;
								recevie_data->data1 = g_projector_para.projector_tuning.briness_index;
								recevie_data->data3 = Red_Current;
								recevie_data->data4 = Green_Current;
								recevie_data->data5 = Blue_Current;
						}
						else
						{
								recevie_data->data0 = 257;
						}
					break;

					case A100_SET_CCT_MODE:
					{
							if(recevie_data->data0<3  && g_projector_para.projector_tuning.tuning_valid == 1 )
							{
								uint16_t Red_Current = 0x0000;
								uint16_t Green_Current = 0x0000;
								uint16_t Blue_Current = 0x0000;
								g_projector_para.projector_tuning.cct_index = recevie_data->data0;
								Red_Current 	= g_projector_para.projector_tuning.cct[g_projector_para.projector_tuning.cct_index].briness[g_projector_para.projector_tuning.briness_index].red_current ;
								Green_Current = g_projector_para.projector_tuning.cct[g_projector_para.projector_tuning.cct_index].briness[g_projector_para.projector_tuning.briness_index].green_current ;
								Blue_Current 	= g_projector_para.projector_tuning.cct[g_projector_para.projector_tuning.cct_index].briness[g_projector_para.projector_tuning.briness_index].blue_current ;
								A100_SetRedCurrent(Red_Current);
								A100_SetGreenCurrent(Green_Current);
								A100_SetBlueCurrent(Blue_Current);
								for(i=0; i<88; i++) {//write gamma
									A100_I2cWriteSxmb241(SXRD241_I2C_ADDRESS, 0x58+i, g_projector_para.gamma_data.gamma_reg[recevie_data->data0 * 88 + i]);
								}
								recevie_data->data0 = g_projector_para.projector_tuning.cct_index;
								recevie_data->data1 = g_projector_para.projector_tuning.briness_index;
								recevie_data->data3 = Red_Current;
								recevie_data->data4 = Green_Current;
								recevie_data->data5 = Blue_Current;
							}
							else
							{
								recevie_data->data0 = 256;
							}
					}
					break;

					case A100_SET_HAPTIC_START:

					break;

					case A100_I2C_WRITE:
					{
							if(recevie_data->data0 == 1)
							{
								hi2c = hi2c1;
							}
							else if(recevie_data->data0 == 2)
							{
								hi2c = hi2c2;
							}
							else
							{
								HAL_UART_Transmit(&huart1, (uint8_t *)c,  16, 100);
							}
							if(recevie_data->data1 == CXD3554_I2C_ADDRESS)
							{
								MemAddSize =	I2C_MEMADD_SIZE_16BIT;
							}
							else
							{
								MemAddSize = I2C_MEMADD_SIZE_8BIT;
							}
							HAL_I2C_Mem_Write(&hi2c,
																recevie_data->data1,
																recevie_data->data2,
																MemAddSize,
																(uint8_t *)(&recevie_data->data3),
																1,
																300);
					}
					break;
					case A100_I2C_READ:
					{
							if(recevie_data->data0 == 1)
							{
								hi2c = hi2c1;
							}
							else if(recevie_data->data0 == 2)
							{
								hi2c = hi2c2;
							}
							else
							{
								HAL_UART_Transmit(&huart1, (uint8_t *)c,  16, 100);
							}
							if(recevie_data->data1 == CXD3554_I2C_ADDRESS)
								MemAddSize =	I2C_MEMADD_SIZE_16BIT;
							else
								MemAddSize = I2C_MEMADD_SIZE_8BIT;

							HAL_I2C_Mem_Read(&hi2c,
																recevie_data->data1,
																recevie_data->data2,
																MemAddSize,
																&I2cBuf,
																1,
																1000);
							HAL_I2C_Mem_Read(&hi2c,
																recevie_data->data1,
																recevie_data->data2,
																MemAddSize,
																&I2cBuf,
																1,
																1000);
							recevie_data->data3 = I2cBuf;
					}
				case A100_SET_INT_PATTERN_TYPE:
					switch(recevie_data->data0)
					{
								case INTERNAL_PATTERN_RASTER:
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1860, 0xF3);
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1864, INTERNAL_PATTERN_RASTER);
								break;

								case INTERNAL_PATTERN_WINDOW:
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1860, 0xF3);
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1864, INTERNAL_PATTERN_WINDOW);
								break;

								case INTERNAL_PATTERN_VERTICAL_stripe:
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1860, 0xF3);
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1864, INTERNAL_PATTERN_VERTICAL_stripe);
								break;

								case INTERNAL_PATTERN_HORIZONTAL_STRIPE:
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1860, 0xF3);
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1864, INTERNAL_PATTERN_HORIZONTAL_STRIPE);
								break;

								case INTERNAL_PATTERN_CROSSHATCH:
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1860, 0xF3);
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1864, INTERNAL_PATTERN_CROSSHATCH);
								break;

								case INTERNAL_PATTERN_DOT:
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1860, 0xF3);
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1864, INTERNAL_PATTERN_DOT);
								break;

								case INTERNAL_PATTERN_CROSSHATCH_DOT:
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1860, 0xF3);
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1864, INTERNAL_PATTERN_CROSSHATCH_DOT);
								break;

								case INTERNAL_PATTERN_HRAMP:
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1860, 0xF3);
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1864, INTERNAL_PATTERN_HRAMP);
								 break;

								case INTERNAL_PATTERN_VRAMP:
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1860, 0xF3);
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1864, INTERNAL_PATTERN_VRAMP);
								 break;

								case INTERNAL_PATTERN_FRAME:
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1860, 0xF3);
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1864, INTERNAL_PATTERN_FRAME);
								 break;

								case INTERNAL_PATTERN_CHECKBOX:
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1860, 0xF3);
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1864, INTERNAL_PATTERN_CHECKBOX);
								 break;

								case INTERNAL_PATTERN_COLORBAR:
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1860, 0x01);
									A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1480, 0x71);
								 break;
							}
					break;

					case A100_SET_SOURCE_INPUT:
					switch(recevie_data->data0)
					{
						case SOURCE_TEST_PATTERN1:
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1860, 0x00);
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1480, 0x76);
						break;

						case SOURCE_TEST_PATTERN2:
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1860, 0x00);
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1480, 0x71);//colorbar
						break;

						case SOURCE_CUSTOMER_INPUT:								
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1860, 0x00);
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1480, (0x70 + recevie_data->data1));
						break;
						
						case SOURCE_EXTERNAL_INPUT:
						default:
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1860, 0x00);
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1480, 0x00);
						break;
					}
					
					break;

					case A100_GET_VERSION:
						recevie_data->data0 = VERSION0;
						recevie_data->data1 = VERSION1;
						recevie_data->data2 = VERSION2;
					break;
#ifdef CONFIG_KST_INDEX
					case A100_SET_KST:
						if(recevie_data->data0 < KST_DEGREE_NUM)
						{
							uint16_t index;
							g_projector_para.keystone = recevie_data->data0;
							index = g_projector_para.keystone * KST_DEGREE_NUM + g_projector_para.side_keystone;
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A40, 0x6d);
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A41, 0x68);
							for(i = 0; i<KST_REG_NUM ;i++)
							{
								A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A49 + i, g_kst[index][i]);
							}
							recevie_data->data3 = index;
						}
						else
						{
							recevie_data->data3 = 101;
						}

						g_projector_para.keystone_valid = 1;

					break;

					case A100_SET_SIDE_KST:
						if(recevie_data->data0 < KST_DEGREE_NUM)
						{
							uint16_t index;
							g_projector_para.side_keystone = recevie_data->data0;
							index = g_projector_para.keystone * KST_DEGREE_NUM + g_projector_para.side_keystone;
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A40, 0x6d);
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A41, 0x68);
							for(i = 0; i<KST_REG_NUM ;i++)
							{
								A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A49 + i, g_kst[index][i]);
							}
							recevie_data->data3 = index;
						}
						else
						{
							recevie_data->data3 = 132;
						}
						g_projector_para.side_keystone_valid = 1;
					break;

					case A100_GET_KST:
						recevie_data->data0 = g_projector_para.keystone;
					break;

					case A100_GET_SIDE_KST:
						recevie_data->data0 = g_projector_para.side_keystone;
					break;
					
#else
					case A100_SET_KST:
						printf("data0[%d] \r\n",recevie_data->data0);
						if(recevie_data->data0 == 0) {
							g_projector_para.kst_val[0] = (uint8_t)(recevie_data->data1 & 0xff);
							g_projector_para.kst_val[1] = (uint8_t)(recevie_data->data1 >> 8);
							g_projector_para.kst_val[2] = (uint8_t)(recevie_data->data2 & 0xff);
							g_projector_para.kst_val[3] = (uint8_t)(recevie_data->data2 >> 8);
							g_projector_para.kst_val[4] = (uint8_t)(recevie_data->data3 & 0xff);
							g_projector_para.kst_val[5] = (uint8_t)(recevie_data->data3 >> 8);
							g_projector_para.kst_val[6] = (uint8_t)(recevie_data->data4 & 0xff);
							g_projector_para.kst_val[7] = (uint8_t)(recevie_data->data4 >> 8);							
						}
						else if(recevie_data->data0 == 1) {
							g_projector_para.kst_val[8] = (uint8_t)(recevie_data->data1 & 0xff);
							g_projector_para.kst_val[9] = (uint8_t)(recevie_data->data1 >> 8);
							g_projector_para.kst_val[10] = (uint8_t)(recevie_data->data2 & 0xff);
							g_projector_para.kst_val[11] = (uint8_t)(recevie_data->data2 >> 8);
							g_projector_para.kst_val[12] = (uint8_t)(recevie_data->data3 & 0xff);
							g_projector_para.kst_val[13] = (uint8_t)(recevie_data->data3 >> 8);
							g_projector_para.kst_val[14] = (uint8_t)(recevie_data->data4 & 0xff);
							g_projector_para.kst_val[15] = (uint8_t)(recevie_data->data4 >> 8);

							g_projector_para.kst_index = recevie_data->data5;
							g_projector_para.kst_valid = 1;				
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A40, 0x6d);
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A41, 0x68);							
							for(i = 0; i < 16 ;i++)
							{
								A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A49 + i, g_projector_para.kst_val[i]);
								//printf("kst_val[%d]: 0x%x \r\n",i, g_projector_para.kst_val[i]);
							}
						}
					break;
					
					case A100_GET_KST:
						recevie_data->data0 = g_projector_para.kst_index;
					break;
#endif
					case A100_GET_CURRENT_MODE:
						recevie_data->data0 = g_projector_para.projector_tuning.briness_index;
					break;

					case A100_GET_CCT_MODE:
						recevie_data->data0 = g_projector_para.projector_tuning.cct_index;
					break;

					case A100_SET_HORIZONTAL:
						A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x0090, (H_FLIP_Mode)recevie_data->data0);
						g_projector_para.hflip = (H_FLIP_Mode)recevie_data->data0;
						g_projector_para.hflip_valid = 0x01;
					break;

					case A100_SET_VERTICAL:
						A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x0091, (V_FLIP_Mode)recevie_data->data0);
						g_projector_para.vflip = (V_FLIP_Mode)recevie_data->data0;
						g_projector_para.vflip_valid = 0x01;
					break;

					case A100_GET_HORIZONTAL:
						recevie_data->data0 = g_projector_para.hflip;
					break;

					case A100_GET_VERTICAL:
						recevie_data->data0 = g_projector_para.vflip;
					break;

					case A100_SET_LED_INPUT:
					switch((Led_type)recevie_data->data0)
					{
								case RED_LED_ON:
									A100_SetRedCurrent(recevie_data->data1);
									A100_SetGreenCurrent(0);
									A100_SetBlueCurrent(0);
									recevie_data->data3 = 102;
								break;			

								case RED_LED1_ON:

									recevie_data->data3 = 103;
								break;

								case RED_LED2_ON:

									recevie_data->data3 = 104;
								break;

								case GREEN_LED_ON:
									A100_SetRedCurrent(0);
									A100_SetGreenCurrent(recevie_data->data1);
									A100_SetBlueCurrent(0);
									recevie_data->data3 = 105;
								break;

								case GREEN_LED1_ON:

									recevie_data->data3 = 106;
								break;

								case GREEN_LED2_ON:

									recevie_data->data3 = 107;
								break;

								case BLUE_LED1_ON:

								break;

								case BLUE_LED_ON:
									A100_SetRedCurrent(0);
									A100_SetGreenCurrent(0);
									A100_SetBlueCurrent(recevie_data->data1);
									recevie_data->data3 = 108;
								break;

								case LED_ON:
									g_projector_para.red_current = recevie_data->data1;
									g_projector_para.green_current = recevie_data->data2;
									g_projector_para.blue_current = recevie_data->data3;
									g_projector_para.brightness = 0x00;
									g_projector_para.current_valid = 0x01;
									g_projector_para.projector_tuning.tuning_valid = 0;
									
									printf("set red:%d green:%d blue:%d \r\n",recevie_data->data1,recevie_data->data2,recevie_data->data3);
									A100_SetRedCurrent(recevie_data->data1);
									A100_SetGreenCurrent(recevie_data->data2);
									A100_SetBlueCurrent(recevie_data->data3);
								
									recevie_data->data0 =  TPL1401_ReadI2C_Byte(0x90, 0x21) >> 4;//g_RedCurrent;
									recevie_data->data1 =  TPL1401_ReadI2C_Byte(0x92, 0x21) >> 4;//g_GreenCurrent;
									recevie_data->data2 =  TPL1401_ReadI2C_Byte(0x94, 0x21) >> 4;//g_BlueCurrent;																	
									recevie_data->data5 = 109;
								break;

								case ALL_LED_ON:
								default:
									A100_SetRGBCurrent();
								break;
					 }
					break;

					case A100_GET_ADC_TEMP:
					{
							uint16_t ld_adc = 0, lcos_adc = 0;
							uint16_t adc_val[3];
							uint16_t ld_temp, lcos_temp;
	
							ld_adc = adc_GetAdcVal(adc_val);
							lcos_adc = adc_val[2];
							ld_temp = GetLd_RT_Temp(ld_adc);
						  lcos_temp = GetLcos_RT_Temp(lcos_adc);
						
							recevie_data->data0 =  ld_adc;
							recevie_data->data1 =  lcos_adc;
							recevie_data->data2 =  ld_temp;
							recevie_data->data3 =  lcos_temp;
							recevie_data->data4 =  (lcos_adc*3300)/4096;
							recevie_data->data5 =  114;
					}
					break;

					case A100_GET_CURRENT:
							recevie_data->data0 =  96 - (TPL1401_ReadI2C_Byte(0x90, 0x21) >> 4);//g_RedCurrent;
							recevie_data->data1 =  96 - (TPL1401_ReadI2C_Byte(0x92, 0x21) >> 4);//g_GreenCurrent;
							recevie_data->data2 =  96 - (TPL1401_ReadI2C_Byte(0x94, 0x21) >> 4);//g_BlueCurrent;
							recevie_data->data3 = 115;
					break;

					case A100_SET_DISPLAY_ON:
							A100_display_on(recevie_data->data0);
							recevie_data->data3 = 116;
					break;

					case A100_SET_MOTOR_START:
						Motor_start(recevie_data->data0, recevie_data->data1);
						break;					

					case A100_GET_BRITNRSS_NUM:
						recevie_data->data0 = A100_BRITNRSS_NUM;
					break;
#ifdef CONFIG_KST_INDEX	
					case A100_GET_KST_NUM:
						recevie_data->data0 = A100_KST_NUM;
					break;

					case A100_GET_SIDE_KST_NUM:
						recevie_data->data0 = A100_SIDE_KST_NUM;
					break;
#endif
					case A100_GET_DISPLAY_SIZE_NUM:
						recevie_data->data0 = A100_DISPLAY_SIZE_NUM;
					break;

					case A100_GET_HORIZONTAL_NUM:
						recevie_data->data0 = A100_HORIZONTAL_NUM;
					break;

					case A100_GET_VERTICAL_NUM:
						recevie_data->data0 = A100_VERTICAL_NUM;
					break;
					
					case A100_GET_CCT_NUM:
						recevie_data->data0 = A100_CCT_NUM;
					break;

					case A100_GET_LED_ON_OFF:
						recevie_data->data0 = gpiopin;
					break;		
					
					case A100_GET_ADC_PS:
							value0 = adc_GetPsOut();
							recevie_data->data0 =  value0;
							recevie_data->data2 = 124;
					break;		

					case A100_GET_ADC_TS:
							value0 = adc_GetTsOut();
							recevie_data->data0 =  value0;
							recevie_data->data1 =  g_fan_value;
					break;		

					case A100_GET_TUNING_PARAMETER:
						HAL_UART_Transmit(&huart1, (uint8_t *)(&(g_projector_para.projector_tuning)), sizeof(struct Projector_CCT), 1000);
					break;

					case A100_SET_R_GAIN:
					{
						uint8_t val;
						val = recevie_data->data0;
						if(val<=100)
						{
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x3216, val+155);
							g_projector_para.r_gain = val+155 ;
						}
						else
						{
							recevie_data->data5 = 0xEE;
						}
						recevie_data->data3 = 125;
						break;
					}
					case A100_SET_G_GAIN:
					{
						uint8_t val;
						val = recevie_data->data0;
						if(val<=100)
						{
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x3217, recevie_data->data0+155);
							g_projector_para.g_gain = val+155 ;
						}
						else
						{
							recevie_data->data5 = 0xEE;
						}
						recevie_data->data3 = 126;
						break;
					}
					case A100_SET_B_GAIN:
					{
						uint8_t val;
						val = recevie_data->data0;
						if(val<=100)
						{
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x3218, recevie_data->data0+155);
							g_projector_para.b_gain = val+155 ;
						}
						else
						{
							recevie_data->data5 = 0xEE;
						}
						recevie_data->data3 = 127;
						break;
					}
					case A100_GET_R_GAIN:
					{
						uint8_t val;
						A100_I2cReadCxd3554(CXD3554_I2C_ADDRESS, 0x3216, &val);

						recevie_data->data0 = val-155;
						recevie_data->data1 = (unsigned short)g_projector_para.r_gain;
						recevie_data->data2 = (char)g_projector_para.r_gain;
						recevie_data->data3 = (unsigned short)g_projector_para.r_gain;
						recevie_data->data4 = sizeof(struct Projector_parameter);
						break;
					}
					case A100_GET_G_GAIN:
					{
						uint8_t val;
						A100_I2cReadCxd3554(CXD3554_I2C_ADDRESS, 0x3217, &val);
						recevie_data->data0 = val-155 ;
						recevie_data->data3 = 129;
						break;
					}
					case A100_GET_B_GAIN:
					{
						uint8_t val;
						A100_I2cReadCxd3554(CXD3554_I2C_ADDRESS, 0x3218, &val);
						recevie_data->data0 = val-155;
						recevie_data->data3 = 130;
						break;
					}	
					case A100_READ_GAMA:
					{
						uint8_t reg_val;
						uint8_t reg_addr;
						
						reg_addr = 0x58 + recevie_data->data0;
						A100_I2cReadSxmb241(SXRD241_I2C_ADDRESS, reg_addr, &reg_val);
						recevie_data->data0 = reg_val;
						A100_I2cReadSxmb241(SXRD241_I2C_ADDRESS, reg_addr + 1, &reg_val);
						recevie_data->data1 = reg_val;
						A100_I2cReadSxmb241(SXRD241_I2C_ADDRESS, reg_addr + 2, &reg_val);
						recevie_data->data2 = reg_val;
						A100_I2cReadSxmb241(SXRD241_I2C_ADDRESS, reg_addr + 3, &reg_val);
						recevie_data->data3 = reg_val;
						break;
					}
					case A100_WRITE_GAMA:
					{
						uint8_t reg_val;
						uint8_t reg_addr;
						reg_addr = 0x58 + recevie_data->data0;
						reg_val = recevie_data->data1;
						A100_I2cWriteSxmb241(SXRD241_I2C_ADDRESS, reg_addr, reg_val);
						reg_addr = reg_addr + 4;
						reg_val = recevie_data->data2;
						A100_I2cWriteSxmb241(SXRD241_I2C_ADDRESS, reg_addr, reg_val);
						break;
					}
					case A100_GET_GAMA:
					{
						uint8_t reg_val;
						uint8_t reg_addr;
						
						reg_addr = 0x58 + 8 * recevie_data->data0;
						A100_I2cReadSxmb241(SXRD241_I2C_ADDRESS, reg_addr, &reg_val);
						recevie_data->data0 = reg_val;
						A100_I2cReadSxmb241(SXRD241_I2C_ADDRESS, reg_addr + 1, &reg_val);
						recevie_data->data1 = reg_val;
						A100_I2cReadSxmb241(SXRD241_I2C_ADDRESS, reg_addr + 2, &reg_val);
						recevie_data->data2 = reg_val;
						A100_I2cReadSxmb241(SXRD241_I2C_ADDRESS, reg_addr + 3, &reg_val);
						recevie_data->data3 = reg_val;
						
						break;
					}
					case A100_SET_GAMA:
					{
						uint8_t reg_val;
						uint8_t reg_addr;

						reg_addr = 0x58 + 8 * recevie_data->data0;
						reg_val = recevie_data->data1;
						A100_I2cWriteSxmb241(SXRD241_I2C_ADDRESS, reg_addr, reg_val);
						reg_val = recevie_data->data2;
						A100_I2cWriteSxmb241(SXRD241_I2C_ADDRESS, reg_addr + 1, reg_val);
						reg_val = recevie_data->data3;
						A100_I2cWriteSxmb241(SXRD241_I2C_ADDRESS, reg_addr + 2, reg_val);
						//reg_val = recevie_data->data4;
						//A100_I2cWriteSxmb241(SXRD241_I2C_ADDRESS, reg_addr + 3, reg_val);
						reg_val = 255 - recevie_data->data1;
						A100_I2cWriteSxmb241(SXRD241_I2C_ADDRESS, reg_addr + 4, reg_val);
						reg_val = 255 - recevie_data->data2;
						A100_I2cWriteSxmb241(SXRD241_I2C_ADDRESS, reg_addr + 5, reg_val);
						reg_val = 255 - recevie_data->data3;
						A100_I2cWriteSxmb241(SXRD241_I2C_ADDRESS, reg_addr + 6, reg_val);
						//reg_val = 255 - recevie_data->data4;
						//A100_I2cWriteSxmb241(SXRD241_I2C_ADDRESS, reg_addr + 7, reg_val);
						break;
					}
					
					case A100_SAVE_GAMA:
					{
						uint16_t cct_index = recevie_data->data0;
						
						g_projector_para.gamma_data.gamma_valid = 0xFDDF;
						for(i=0; i<88; i++) {
							A100_I2cReadSxmb241(SXRD241_I2C_ADDRESS, 0x58+i, &g_projector_para.gamma_data.gamma_reg[cct_index*88 + i]);
						}
						break;
					}
					
					case A100_READ_CXD:
					{
						uint8_t  reg_val;
						uint16_t reg_addr;
						reg_addr = recevie_data->data0;
						A100_I2cReadCxd3554(CXD3554_I2C_ADDRESS, reg_addr, &reg_val);
						recevie_data->data0 = reg_val;
						break;
					}
					case A100_WRITE_CXD:
					{
						uint8_t  reg_val;
						uint16_t reg_addr;
						reg_addr = recevie_data->data0;
						reg_val  = recevie_data->data1;
						A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, reg_addr, reg_val);
						break;
					}
					case A100_SET_IPG:
					{
						A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1860, 0xf2);
						A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1480, 0x00);
						A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1864, 0x01);
						break;
					}
					case A100_GET_WC:
					{
						if((recevie_data->data0)<4)
						{
							recevie_data->data1 = wc_data_temp[recevie_data->data0].power_num;
							recevie_data->data2 = wc_data_temp[recevie_data->data0].gain;
							recevie_data->data3 = wc_data_temp[recevie_data->data0].offset;
							recevie_data->data4 = wc_data_temp[recevie_data->data0].l;
						}
						else
						{
							recevie_data->data1 = 0xff;
							recevie_data->data2 = 0xff;
							recevie_data->data3 = 0xff;
							recevie_data->data4 = 0xff;
						}
						break;
					}
					case A100_SET_WC_SAVE:
					{
							memcpy(g_projector_para.wc,wc_temp,438);
							memcpy(g_projector_para.wc_data,wc_data_temp,sizeof(struct Projector_WC)*4);
							g_projector_para.wc_valid = 0x02;
						break;
					}
					case A100_GET_GAMA_PARAMETER:
					{
							//for(i=0;i<88;i++) printf("reg[%d]=0x%x\r\n",i+58, g_projector_para.gamma_data.gamma_reg[i]);
							HAL_UART_Transmit(&huart1, (uint8_t *)&g_projector_para.gamma_data,  sizeof(struct Projector_Gama), 100);
							return;
					}
					case A100_SET_PARAMRTER:
					{
							A100_setparameter(&g_projector_para);
							break;
					}
					case A100_READ_CXD3554:
					{
							uint8_t reg_data;
						
							A100_I2cReadCxd3554(CXD3554_I2C_ADDRESS, recevie_data->data0, &reg_data);
							recevie_data->data0 = reg_data;
							//A100_Dump_Cxd3554(recevie_data->data0, recevie_data->data1);
							recevie_data->data4 = 3554;
							break;
					}
					
					case A100_WRITE_CXD3554:
					{						
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, recevie_data->data0, recevie_data->data1);
							recevie_data->data4 = 3555;
							break;
					}
					
					case A100_TEST_IDC:
					{
							uint8_t reg_h, reg_l;
						
							//A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x0061, 0xff);
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1840, recevie_data->data2);
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1841, (recevie_data->data0 >> 8) & 0x0f);
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1842, recevie_data->data0 & 0xff);
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1843, (recevie_data->data1 >> 8) & 0x07);
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1844, recevie_data->data1 & 0xff);
						
						  A100_I2cReadCxd3554(CXD3554_I2C_ADDRESS, 0x1845, &reg_h);
						  A100_I2cReadCxd3554(CXD3554_I2C_ADDRESS, 0x1846, &reg_l);
							recevie_data->data0 = ((reg_h & 0x03) << 8) + reg_l;
						
							recevie_data->data4 = 777;
							break;
					}

					case A100_TEST_DC:
					{
							uint8_t reg_data;
						
							//A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x0065, 0xff);
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x21E0, recevie_data->data2 << 4);
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x21E1, (recevie_data->data0 >> 8) & 0x0f);
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x21E2, recevie_data->data0 & 0xff);
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x21E3, (recevie_data->data1 >> 8) & 0x07);
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x21E4, recevie_data->data1 & 0xff);
						  A100_I2cReadCxd3554(CXD3554_I2C_ADDRESS, 0x21E6, &reg_data);

							recevie_data->data0 = reg_data;
							recevie_data->data4 = 778;
							break;
					}

					case A100_TEST_ODC:
					{
							uint8_t reg_data;
						
							//A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x0067, 0xff);
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x30E0, recevie_data->data2 << 4);
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x30E1, recevie_data->data3);//sub frame
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x30E2, (recevie_data->data0 >> 8) & 0x0f);
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x30E3, recevie_data->data0 & 0xff);
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x30E4, (recevie_data->data1 >> 8) & 0x07);
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x30E5, recevie_data->data1 & 0xff);
						  A100_I2cReadCxd3554(CXD3554_I2C_ADDRESS, 0x30E6, &reg_data);
							recevie_data->data0 = reg_data;						
						  A100_I2cReadCxd3554(CXD3554_I2C_ADDRESS, 0x30E7, &reg_data);
							recevie_data->data1 = reg_data;						
						  A100_I2cReadCxd3554(CXD3554_I2C_ADDRESS, 0x30E8, &reg_data);
							recevie_data->data2 = reg_data;
						
						  recevie_data->data4 = 779;
							break;
					}					
					default:
					break;
				}
				//recevie_data->checksum = UartReceiveLength;
				recevie_data->checksum = (recevie_data->command + recevie_data->data0 + recevie_data->data1 + recevie_data->data2 + recevie_data->data3 + recevie_data->data4+ recevie_data->data5) % 0x1000;
				HAL_UART_Transmit(&huart1, (uint8_t *)recevie_data,  sizeof(struct Projector_date), 100);
		}
	else
	{
		int para0_len,para1_len,read_len;
		unsigned char temp;
		
		memset(&receive_buffer,0,40);
		para0_len = 0;
		para1_len = 0;
		memcpy(receive_buffer,pRx,UartReceiveLength);
		read_len = read_evm(receive_buffer,UartReceiveLength,&para0_len,&para1_len);
		para_Analysis((char*)(&receive_buffer),para0_len,para1_len);
#if 1
		printf("rw:%d chip_addr:0x%x  0x%x->0x%x \r\n",asu_rec_data.w_r, asu_rec_data.chip_addr, asu_rec_data.reg_addr, asu_rec_data.reg_value);
#endif

		if(asu_rec_data.w_r == 1)
		{
			if(asu_rec_data.chip_addr == CXD3554_I2C_ADDRESS)
			{
/*				
						uint16_t temp_value;
						temp_value = (asu_rec_data.reg_value+1)*4-1;
						if(asu_rec_data.reg_addr==0x3216)
						{
							A100_SetRedCurrent(temp_value);
							g_red_value	= asu_rec_data.reg_value;
						}
						else if(asu_rec_data.reg_addr==0x3217)
						{
							A100_SetGreenCurrent(temp_value);
							g_green_value	= asu_rec_data.reg_value;
						}
						else if(asu_rec_data.reg_addr==0x3218)
						{
							A100_SetBlueCurrent(temp_value);
							g_blue_value	= asu_rec_data.reg_value;
						}
						else
*/
						{
							A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, asu_rec_data.reg_addr, asu_rec_data.reg_value);
						}
			}
			else if (asu_rec_data.chip_addr == SXRD241_I2C_ADDRESS)
			{
						A100_I2cWriteSxmb241(SXRD241_I2C_ADDRESS, asu_rec_data.reg_addr, asu_rec_data.reg_value);
			}
			memset(&asu_rec_data,0x00,sizeof(struct asu_date));
		}

		if(asu_rec_data.w_r == 2)
		{
			if(asu_rec_data.chip_addr == CXD3554_I2C_ADDRESS)
			{
				A100_I2cReadCxd3554(CXD3554_I2C_ADDRESS, asu_rec_data.reg_addr, (uint8_t *)&asu_rec_data.reg_value);
			}
			else
				A100_I2cReadSxmb241(SXRD241_I2C_ADDRESS, asu_rec_data.reg_addr, (uint8_t *)&asu_rec_data.reg_value);
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
						A100_SetRedCurrent(temp_value);
						g_red_value	= asu_rec_data.reg_value;
					}
					else if(asu_rec_data.reg_addr==0x3217)
					{
						A100_SetGreenCurrent(temp_value);
						g_green_value	= asu_rec_data.reg_value;
					}
					else if(asu_rec_data.reg_addr==0x3218)
					{
						A100_SetBlueCurrent(temp_value);
						g_blue_value = asu_rec_data.reg_value;
					}
					else
*/
					{
						A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, asu_rec_data.reg_addr, asu_rec_data.reg_value);			
					}
			}
			else if (asu_rec_data.chip_addr == SXRD241_I2C_ADDRESS)
			{
			      A100_I2cWriteSxmb241(SXRD241_I2C_ADDRESS, asu_rec_data.reg_addr, asu_rec_data.reg_value);
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
							A100_I2cReadCxd3554(CXD3554_I2C_ADDRESS, asu_rec_data.reg_addr, (uint8_t *)&asu_rec_data.reg_value);
						}
			}
			else if (asu_rec_data.chip_addr == SXRD241_I2C_ADDRESS)
			{
							A100_I2cReadSxmb241(SXRD241_I2C_ADDRESS, asu_rec_data.reg_addr, (uint8_t *)&asu_rec_data.reg_value);
			}
		}
		show_A100_data(&asu_rec_data);
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

void A100_ReceiveUart1Data(void)
{
	HAL_UART_Receive_DMA(&huart1, (uint8_t *)UartReceiveRxBuffer, UART_BUFFER_MAX_SIZE);	
}

void HAL_UART_AbortReceiveCpltCallback (UART_HandleTypeDef *huart)
{

	A100_UartCmdHandler((uint8_t *)UartReceiveRxBuffer,UartReceiveLength);
	UartReceiveLength = 0;

	//Re-start receiving
	A100_ReceiveUart1Data();
	/* NOTE : This function should not be modified, when the callback is needed,
	the HAL_UART_AbortTransmitCpltCallback can be implemented in the user file.
	*/
}	
/* lcos ------------------------------------------------------------------*/

void A100_LcosSetGain(void)
{
		if((unsigned char)g_projector_para.r_gain<155)
			g_projector_para.r_gain = 255;
		if((unsigned char)g_projector_para.g_gain<155)
			g_projector_para.g_gain = 255;
		if((unsigned char)g_projector_para.b_gain<155)
			g_projector_para.b_gain = 255;

		A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x3216, (unsigned char)g_projector_para.r_gain);
		A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x3217, (unsigned char)g_projector_para.g_gain);
		A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x3218, (unsigned char)g_projector_para.b_gain);
}

void A100_LcosSetGamma(void)
{
	  int i, cct_index;

		printf("gamma_valid=0x%x\r\n",g_projector_para.gamma_data.gamma_valid);
		if((g_projector_para.gamma_data.gamma_valid) == 0xFDDF)
		{
			cct_index = 0;//g_projector_para.projector_tuning.cct_index;
			for(i=0; i<88; i++) {
				A100_I2cWriteSxmb241(SXRD241_I2C_ADDRESS, 0x58+i, g_projector_para.gamma_data.gamma_reg[cct_index*88 + i]);
				printf("reg[%d]=0x%x\r\n",i+58, g_projector_para.gamma_data.gamma_reg[cct_index*88 + i]);
			}
		}
		
#if 0		
		uint8_t reg_data;
		for(i=0; i<88; i++) {
			A100_I2cReadSxmb241(SXRD241_I2C_ADDRESS, 0x58+i, &reg_data);
			printf("reg[%d]=%d\r\n",i+58, reg_data);
		}	
#endif			
}

void A100_LcosSetFlip(void)
{
	A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x0090, 0x01);
	A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x0091, 0x01);

	if(g_projector_para.vflip_valid == 0x01)
	{
		A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x0091, (V_FLIP_Mode)(g_projector_para.vflip));
	}
	if(g_projector_para.hflip_valid == 0x01)
	{
		A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x0090, (H_FLIP_Mode)(g_projector_para.hflip));	
	}
}

#ifdef CONFIG_KST_INDEX
void A100_LcosSetKst(void)
{
		uint32_t i;
		uint16_t index;

		if((g_projector_para.keystone < KST_DEGREE_NUM)&&(g_projector_para.side_keystone<KST_DEGREE_NUM))
		{
			;
		}
		else
		{
			g_projector_para.keystone = 5;
			g_projector_para.side_keystone =5;
		}		
		index = g_projector_para.keystone * KST_DEGREE_NUM + g_projector_para.side_keystone;
		
		A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A40, 0x6d);
		A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A41, 0x68);
		for(i = 0; i<KST_REG_NUM ;i++)
		{
			A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A49 + i, g_kst[index][i]);
		}
}
#else
void A100_LcosSetKst(void)
{
		uint32_t i;

		if(g_projector_para.kst_valid == 1)
		{
			A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A40, 0x6d);
			A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A41, 0x68);
			for(i = 0; i < 16 ;i++)
				A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A49 + i, g_projector_para.kst_val[i]);
		}
}
#endif
void A100_LcosSetWC(void)
{
	  int i;

		if((g_projector_para.wc_valid) == 0x02)
		{
			A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A40,0x6d);
			A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A42,0x30);
			for(i=0; i<438; i++)
			{ 
					A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS,0x3500 +i , g_projector_para.wc[i]);
			}
		}
		else
		{
			A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A40,0x6d);
			A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS, 0x1A42,0x30);
			for(i=0; i<438; i++)
			{ 
				  g_projector_para.wc[i] = 0x00;
					A100_I2cWriteCxd3554(CXD3554_I2C_ADDRESS,0x3500 +i , g_projector_para.wc[i]);
			}
			memset(g_projector_para.wc_data,0x00,sizeof(struct Projector_WC)*4);
		}
		memcpy(wc_temp,g_projector_para.wc,438);
		memcpy(wc_data_temp,g_projector_para.wc_data,sizeof(struct Projector_WC)*4);	
}
/* tim ------------------------------------------------------------------*/
uint8_t A100_SetFan12Speed(uint32_t speed)
{
  if(speed > FAN_SPEED_FULL)	speed = FAN_SPEED_FULL;
	if(g_fan12_speed == (uint16_t)speed) return 0;
	g_fan12_speed = (uint16_t)speed;
	
  TIM_OC_InitTypeDef sConfigOC={0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = speed;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	
  return 0;
}

uint8_t A100_SetFan34Speed(uint32_t speed)
{
  if(speed > FAN_SPEED_FULL)	speed = FAN_SPEED_FULL;
	if(g_fan34_speed == (uint16_t)speed) return 0;
	g_fan34_speed = (uint16_t)speed;
	
  TIM_OC_InitTypeDef sConfigOC={0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = speed;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  return 0;
}
uint8_t A100_SetFan5Speed(uint32_t speed)
{
  if(speed > FAN_SPEED_FULL)	speed = FAN_SPEED_FULL;
	if(g_fan5_speed == (uint16_t)speed) return 0;
	g_fan5_speed = (uint16_t)speed;
	
  TIM_OC_InitTypeDef sConfigOC={0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = speed;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);

  return 0;
}

#if 1
static uint8_t capture_Cnt = 0;
static uint16_t capture_Buf[3] = {0};

void SetTIM1ICPolarity(uint32_t Polarity)
{
  TIM_IC_InitTypeDef sConfigIC = {0};
	
  sConfigIC.ICPolarity = Polarity;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }	
}

uint8_t A100_GetFanSpeed(void)
{
	uint16_t high_time, lower_time, T_time;
	uint16_t count = 0xffff;
	
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
		//SetTIM1ICPolarity(TIM_INPUTCHANNELPOLARITY_RISING);
		HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);		
		capture_Cnt = 1;
	
	/* waiting convert complete */
	while(count && capture_Cnt != 4 ) {
		count--;
	}
	
	if(count != 0)
	{
		high_time = capture_Buf[1]- capture_Buf[0]; 
		lower_time = capture_Buf[2]- capture_Buf[1];
		T_time =  capture_Buf[2] - capture_Buf[0];
		
		printf("A100_GetFanSpeed  capture1:%d capture2:%d capture3:%d \r\n", capture_Buf[0], capture_Buf[1], capture_Buf[2]);
		printf("A100_GetFanSpeed  high_time:%d lower_time:%d T_time:%d count:%d \r\n", high_time, lower_time, T_time, count);
	}
	else printf("A100_GetFanSpeed  timeout!!! \r\n");

	return 0;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	printf("HAL_TIM_IC_CaptureCallback  capture_Cnt:%d !!! \r\n", capture_Cnt);
	if(TIM1 == htim->Instance)
	{
		switch(capture_Cnt){
			case 1:
				capture_Buf[0] = HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_1);
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim1,TIM_CHANNEL_1,TIM_ICPOLARITY_FALLING);
				//SetTIM1ICPolarity(TIM_ICPOLARITY_FALLING);
				capture_Cnt++;
				break;
			
			case 2:
				capture_Buf[1] = HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_1);
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
				//SetTIM1ICPolarity(TIM_INPUTCHANNELPOLARITY_RISING);
				capture_Cnt++;  
				break;		
			
			case 3:
				capture_Buf[2] = HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_1);
				HAL_TIM_IC_Stop_IT(&htim1,TIM_CHANNEL_1);
				capture_Cnt++;  
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
			printf("ADC_DMA_VAL[%d]=%d \r\n",j, val[j]);
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

void A100_Variables_Init(void)
{
	memset(&asu_rec_data,0x00,sizeof(struct asu_date));	
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
