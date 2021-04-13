/******************************************************************************
  * @project: A70
  * @file: a5931.c
  * @author: zyz
  * @company: ASU
  * @date: 2020.10.23
	*****************************************************************************/
#include "i2c.h"
#include "stdio.h"

#define A5931_I2C_ADDR 0xAA
#define RETRY_CNT 5
static uint16_t A5931_ReadI2C_Byte(uint8_t RegAddr)
{
	uint8_t d[2];
	uint8_t ret,retry_cnt=RETRY_CNT;
		
	while(retry_cnt --) {	
		ret = HAL_I2C_Mem_Read(&hi2c2, (uint16_t)A5931_I2C_ADDR, (uint16_t)RegAddr, I2C_MEMADD_SIZE_8BIT, (uint8_t *)d, 2, 300);
		if(ret == HAL_OK) break;
	}
	if(ret != HAL_OK) printf("A5931_ReadI2C_Byte 0x%x faild. ret=%d\r\n", RegAddr, ret);
	
	return (d[0] << 8) + d[1];
}

static uint8_t A5931_WriteI2C_reg(uint8_t RegAddr, uint16_t data)
{
		uint8_t ret, retry_cnt = RETRY_CNT;
		uint8_t d[2];
	
		d[0] = data  >> 8;
		d[1] = data;
	
		while(retry_cnt --) {
			ret = HAL_I2C_Mem_Write(&hi2c2, (uint16_t)A5931_I2C_ADDR, (uint16_t)RegAddr, I2C_MEMADD_SIZE_8BIT, (uint8_t *)d, 2, 300);
			if(ret == HAL_OK) break;
		}
		if(ret != HAL_OK) printf("A5931_WriteI2C_Byte 0x%x->0x%x  faild. ret=%d\r\n",RegAddr, data, ret);
	
		return ret;
}

static uint8_t A5931_WriteI2C_Byte(uint8_t RegAddr, uint16_t data)
{
		uint8_t ret;
	
		//Erase the word
		ret = A5931_WriteI2C_reg(162, (uint16_t)RegAddr);
		if(ret != HAL_OK) {
			printf("A5931_WriteI2C_Byte 162->0x%x  faild. ret=%d\r\n",RegAddr ,ret);	
			return ret;
		}
		ret = A5931_WriteI2C_reg(163, 0x0000);
		if(ret != HAL_OK) {
			printf("A5931_WriteI2C_Byte 163->0x00  faild. ret=%d\r\n",ret);
			return ret;
		}		
		ret = A5931_WriteI2C_reg(161, 0x0003);
		if(ret != HAL_OK) {
			printf("A5931_WriteI2C_Byte 161->0x03  faild. ret=%d\r\n",ret);
			return ret;
		}
		HAL_Delay(15);
		ret = A5931_WriteI2C_reg(161, 0x0000);
		if(ret != HAL_OK) {
			printf("A5931_WriteI2C_Byte 161->0x00  faild. ret=%d\r\n",ret);	
			return ret;
		}		
		
		//Write the word
		ret = A5931_WriteI2C_reg(162, (uint16_t)RegAddr);
		if(ret != HAL_OK) {
			printf("A5931_WriteI2C_Byte [162->0x%x]  faild. ret=%d\r\n",RegAddr ,ret);	
			return ret;
		}	
		ret = A5931_WriteI2C_reg(163, data);
		if(ret != HAL_OK) {
			printf("A5931_WriteI2C_Byte [163->0x%x]  faild. ret=%d\r\n",data ,ret);
			return ret;
		}
		ret = A5931_WriteI2C_reg(161, 0x0005);
		if(ret != HAL_OK) {
			printf("A5931_WriteI2C_Byte [161->0x05]  faild. ret=%d\r\n",ret);
			return ret;
		}
		HAL_Delay(15);
		ret = A5931_WriteI2C_reg(161, 0x0000);
		if(ret != HAL_OK) {
			printf("A5931_WriteI2C_Byte 161->0x00  faild. ret=%d\r\n",ret);	
			return ret;
		}				

		return ret;
}

#define ASU_FLAG 0x4153
uint16_t A5931_init_data[] = {
	ASU_FLAG,
	0x4000,
	0x04E4,
	0x2633,
	0x0000,
	0xFF19,
	0xC706,
	0x4116,
	0x1D0B,
	0x4566,
	0x5F77,
	0x320C,
	0x0000,
	0x0000,
	0xF1A4,
	0x6419,
	0x51B1,
	0x1562,
	0x0000,
	0x07D0,
	0x0000,
};	

void ThreePhaseMotorDriver_init(void)
{
	uint16_t i;
	
	if(A5931_ReadI2C_Byte((uint8_t)0) != ASU_FLAG) {
		for(i=0;i<sizeof(A5931_init_data)/sizeof(uint16_t);i++) {
			A5931_WriteI2C_Byte(i, A5931_init_data[i]);	
			printf("A5931_init write reg %d->0x%x\r\n",i, A5931_init_data[i]);			
		}
	}
	
#if 0	
	for(i=0;i<24;i++) {
		uint16_t data = A5931_ReadI2C_Byte((uint8_t)i);
		printf("A5931_init read reg %d->0x%x\r\n",i, data);
	}
#endif	
	A5931_WriteI2C_reg(165, 511);

} 


