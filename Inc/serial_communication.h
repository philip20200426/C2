#ifndef __SERIAL_COMMUNICATION_H__
#define __SERIAL_COMMUNICATION_H__

#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "i2c.h"
#include "tim.h"
#include "adc.h"
#include "usart.h"
#include "stm32g0xx_hal_flash.h"
#include "lcos.h"
#include "LaserDrv.h"

/* Private define ------------------------------------------------------------*/
#define FLASH_USER_START_ADDR   (FLASH_BASE + (63 * FLASH_PAGE_SIZE))   /*0~63 Start @ of user Flash area */
#define FLASH_COLORTEMP_START_ADDR	(FLASH_BASE + (62 * FLASH_PAGE_SIZE))
#define FLASH_USER_END_ADDR     (FLASH_BASE + FLASH_SIZE - 1)   /* End @ of user Flash area */

#define	GET_BIT(x, bit)	((x & (1 << bit)) >> bit)

#define PARAMETER_VALID			0x3A
#define PARAMETER_INVALID		0xFF

#define UART_BUFFER_MAX_SIZE 			256
#define VERSION0 	0
#define VERSION1 	0
#define VERSION2 	4
/* -----------------------------------------------------------*/
#define COMM_FLAG								0xFEFE
#define CMD_ERROR								0
#define CMD_ECHO								10
#define CMD_WRITE_CXD3554_REG		11
#define CMD_READ_CXD3554_REG		12
#define CMD_WRITE_SXMB241_REG		13
#define CMD_READ_SXMB241_REG		14

#define CMD_GET_CURRENTS				20
#define CMD_GET_FANS						21
#define CMD_GET_VERSION					22
#define CMD_GET_TEMPS						23



#define CMD_SET_CURRENTS				30
#define CMD_SET_FANS						31
#define CMD_SET_FOCUSMOTOR			32
#define CMD_SET_SONY_TOOL				33
#define CMD_SET_LT9211_TEST			34

#define CMD_SAVE_PARAMRTER			40
#define CMD_CLR_PARAMRTER				41

#define CMD_ENTER_MAT						50

#define PACKAGE_DATA_BASE 			6
/* Private typedef -----------------------------------------------------------*/
typedef enum
{
	H_FLIP = 0,
  H_NORMAL = 1,

}H_FLIP_Mode;

typedef enum
{
	V_FLIP = 0,
  V_NORMAL = 1,

}V_FLIP_Mode;

typedef enum
{
  FAN_SPEED_FULL   		= 100,
  FAN_SPEED_DEFAULT		= 40, //15
  FAN_SPEED_SLOW   		= 27, //10
  FAN_PEED_STOP    		= 0

}FAN_Speed;

typedef enum
{
	MOTOR_CENTER   			= 0,
	MOTOR_RIGHT		 			= 1,
	MOTOR_LEFT   				= 2,
	
}MOTOR_position;

typedef enum
{
	LIMIT_RIGHT		 			= 0,
	LIMIT_LEFT   				= 1,
	LIMIT_UNKNOWN		 		= 10,	
}LIMIT_position;


typedef enum
{
	PARA_CURRENT = 0,
	PARA_FLIP,
	PARA_KST,
	PARA_COLOR_TEMP,
	PARA_WP,
	PARA_241,
	PARA_LED,
	PARA_FRC,
	PARA_MISC,
	PARA_MAX	
}PARA_Type;

typedef enum
{
	FLASH_MISC = 0,
	FLASH_COLOR_TEMP,
	FLASH_MAX	
}FLASH_Type;

typedef enum
{
	CURRENT_NORMAL = 0,
	CURRENT_COOL,
	CURRENT_WARM,
	CURRENT_USER,
	CURRENT_MAX	
}CURRENT_Type;
/* Private variables ---------------------------------------------------------*/


struct Parameter_Gain
{
    uint8_t  valid;
    uint8_t  r;
    uint8_t  g;
    uint8_t  b;	
};

struct Parameter_Flip
{
    uint8_t  valid;
    uint8_t  h;	
    uint8_t  v;	
};

struct Parameter_Kst
{
    uint8_t  valid;
    uint8_t  val[KST_REG_NUM];		
};


struct Parameter_Wp
{
    uint8_t  valid;
    uint8_t  val[WP_REG_NUM];		
};

struct Projector_parameter{
		struct Parameter_Gain gain;
		struct Parameter_Flip flip;
		struct Parameter_Kst kst;
		struct Parameter_Wp wp;	
		uint8_t  Reserved[8];
};


struct Parameter_Sxrd241
{
    uint8_t  valid;
    uint8_t  val[SXRD241_REG_NUM];		
};

struct Parameter_Current
{
    uint8_t  valid;
    uint16_t  r;
    uint16_t  g;
    uint16_t  b;	
};

struct Parameter_Led
{
    uint8_t  valid;
    uint8_t  val[LED_REG_NUM];		
};

struct Parameter_Frc
{
    uint8_t  valid;
    uint8_t  val[FRC_REG_NUM];		
};

struct Parameter_Misc
{
    uint8_t  valid;
    uint8_t  val[MISC_REG_NUM];		
};

struct Projector_Color_Temp{
		struct Parameter_Current current;
		struct Parameter_Sxrd241 reg_241;
		struct Parameter_Led reg_led;
		struct Parameter_Frc reg_frc;
		struct Parameter_Frc reg_misc;
		uint8_t  Reserved[8];
};

struct PACKAGE_HEAD
{
	unsigned short flag;
	unsigned char command;
	unsigned char size;
	unsigned short check;
};

struct asu_date{
    unsigned char  chip_addr;
    unsigned char  w_r; // 0 write ,1 read, other fails
    unsigned short reg_addr;
    unsigned short reg_value;
};

/* Exported functions prototypes ---------------------------------------------*/
extern void UartCmdHandler(uint8_t *pRx,uint8_t length);
extern uint8_t SetFan12Speed(uint32_t speed);
extern uint8_t SetFan34Speed(uint32_t speed);
extern uint8_t SetFan5Speed(uint32_t speed);
extern uint8_t I2cWriteCxd3554(uint8_t I2cAddr,uint16_t I2cReg, uint8_t I2cData);
extern uint8_t I2cReadCxd3554(uint8_t I2cAddr,uint16_t I2cReg, uint8_t *I2cData);
extern uint8_t I2cReadCxd3554Ex(uint8_t I2cAddr,uint16_t I2cReg, uint8_t *I2cData);
extern uint8_t I2cWriteSxmb241(uint8_t I2cAddr,uint8_t I2cReg,uint8_t I2cData);
extern uint8_t I2cReadSxmb241(uint8_t I2cAddr, uint8_t I2cReg, uint8_t *I2cData);
extern uint8_t I2cReadCxd3554Burst(uint8_t I2cAddr,uint16_t I2cReg, uint8_t *I2cData, uint16_t size);
extern uint8_t I2cWriteCxd3554Burst(uint8_t I2cAddr,uint16_t I2cReg, uint8_t *I2cData, uint16_t size);
uint16_t adc_GetTsOut(void);
uint16_t adc_GetPsOut(void);
extern uint16_t adc_GetLDTemp(void);
extern uint16_t adc_GetAdcVal(uint16_t *val);
extern void display_on(uint16_t on);
extern uint8_t Motor_start(uint8_t dir, uint16_t steps);
extern void SetRGBCurrent(void);
extern uint8_t SetRedCurrent(uint16_t RedCurrent);
extern uint8_t SetGreenCurrent(uint16_t RedCurrent);
extern uint8_t SetBlueCurrent(uint16_t RedCurrent);
extern uint16_t TPL1401_ReadI2C_Byte(uint8_t DevAddr, uint8_t RegAddr);
/* Exported variables --------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif
