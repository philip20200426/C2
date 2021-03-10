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
#define A100_UART_COMMAND_HEAD 		101
#define A100_UART_COMMAND_END  		102

#define A100_I2C_WRITE        		  3
#define A100_I2C_READ       	 	    4

#define A100_GET_OE_FAN_SPEED 			7
#define A100_SET_OE_FAN_SPEED 			8
#define A100_SET_HAPTIC_START 		  9
#define A100_SET_CURRENT_MODE  			16
#define A100_SET_INT_PATTERN_TYPE 	18
#define A100_SET_SOURCE_INPUT 		  22
#define A100_SET_LED				   		  42
#define A100_GET_VERSION				    43
#define A100_SET_KST				        44
#define A100_GET_KST				        45
#define A100_GET_CURRENT_MODE  			46
#define A100_SET_HORIZONTAL 		  	48
#define A100_GET_HORIZONTAL 		  	49
#define A100_SET_VERTICAL 		  		50
#define A100_GET_VERTICAL 		  		51
#define A100_SET_LED_INPUT 		  		52
#define A100_DUMP_CXD3554 					54
#define A100_GET_ADC_TEMP 		  		55	//ld temperature sensor
#define A100_SET_DISPLAY_SIZE 		 	56
#define A100_SET_DISPLAY_ON					57
#define A100_SET_MOTOR_START				58

#define A100_GET_DISPLAY_SIZE     	63
#define A100_GET_BRITNRSS_NUM				64
#define A100_GET_KST_NUM						65
#define A100_GET_DISPLAY_SIZE_NUM  	66
#define A100_GET_HORIZONTAL_NUM			67
#define A100_GET_VERTICAL_NUM     	68
#define A100_GET_LED_ON_OFF      		69
#define A100_GET_ADC_PS      				70  //ps out(photo sensor)
#define A100_GET_ADC_TS      				71	//ts out(temperature sensor)
#define A100_GET_TUNING_PARAMETER  	74
#define A100_SET_CCT_MODE  					75
#define A100_GET_CCT_MODE  					76
#define A100_GET_CCT_NUM						77
#define A100_SET_R_GAIN           	80
#define A100_GET_R_GAIN           	81
#define A100_SET_G_GAIN           	82
#define A100_GET_G_GAIN           	83
#define A100_SET_B_GAIN           	84
#define A100_GET_B_GAIN           	85
#define A100_SET_SIDE_KST				  	86
#define A100_GET_SIDE_KST				  	87
#define A100_GET_SIDE_KST_NUM     	89
#define A100_SET_WC_EN      				90
#define A100_READ_GAMA      				91
#define A100_WRITE_GAMA      				92
#define A100_WRITE_SXRD      				93
#define A100_SET_IPG      					94
#define A100_READ_CXD      					95
#define A100_WRITE_CXD      				96
#define A100_SET_WC_EN1      				97
#define A100_SET_WC_EN2      				98
#define A100_GET_WC      						99
#define A100_SET_WC_SAVE      			100
//101 don't use
#define A100_SET_PARAMRTER     			102
#define A100_GET_CURRENT     				111
#define A100_GET_GAMA      					112
#define A100_SET_GAMA      					113
#define A100_SAVE_GAMA      				114

#define A100_SET_TUNING_PARAMETER  	201
#define A100_SET_WC_PARAMETER  			202
#define A100_SET_GAMA_PARAMETER  		203
#define A100_GET_GAMA_PARAMETER  		204

#define A100_BRITNRSS_NUM						2
#define A100_HORIZONTAL_NUM					1
#define A100_VERTICAL_NUM      			1		
#define A100_CCT_NUM      					2
#define A100_DISPLAY_SIZE_NUM  			5
#define A100_SIDE_KST_NUM						10
#define A100_KST_NUM								10

#ifndef UART_BUFFER_MAX_SIZE
  #define UART_BUFFER_MAX_SIZE 			256
#endif
#define VERSION0 	0
#define VERSION1 	0
#define VERSION2 	1

#define FLASH_USER_START_ADDR   (FLASH_BASE + (62 * FLASH_PAGE_SIZE))   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     (FLASH_BASE + FLASH_SIZE - 1)   /* End @ of user Flash area */

#define	GET_BIT(x, bit)	((x & (1 << bit)) >> bit)
/* Private typedef -----------------------------------------------------------*/
typedef enum
{
	ALL_LED_ON   		= 0,
	RED_LED1_ON   	= 1,
	RED_LED2_ON		 	= 2,
	GREEN_LED1_ON		= 3,
	GREEN_LED2_ON   = 4,
	BLUE_LED1_ON    = 5,
	LED_ON   				= 6,
	RED_LED_ON   		= 7,
	GREEN_LED_ON		= 8,
	BLUE_LED_ON    	= 9
}Led_type;


typedef enum
{
	SOURCE_TEST_PATTERN1   		= 0,
	SOURCE_TEST_PATTERN2   		= 1,
	SOURCE_EXTERNAL_INPUT		 	= 2,
	SOURCE_CUSTOMER_INPUT			= 3,
}Source_type;


typedef enum
{
  FAN_SPEED_FULL   = 40 - 1,
  FAN_SPEED_FAST   = 35,
  FAN_SPEED_MIDDLE = 30,
  FAN_SPEED_DEFAULT= 20,
  FAN_SPEED_SLOW   = 15,
  FAN_PEED_STOP    = 0

}FAN_Speed;

/*
typedef enum
{
  A100_SET_LED_RRGGBBGG = 0,
  A100_SET_LED_RRGGRRBB = 1,
  A100_SET_LED_RRBBGGBB = 2,
  A100_SET_LED_RRGGBBWW = 3,
}LED_Mode;

typedef enum
{
  Display_Size_100 = 0,
	Display_Size_97 = 1,
	Display_Size_95 = 2,
  Display_Size_90 = 3,
  Display_Size_85 = 4,
	Display_Size_80 = 5,
  Display_Size_60 = 6,

}Display_Size;
*/
typedef enum
{
	H_FLIP = 0,
  H_NORMAL = 1,

}H_FLIP_Mode;

typedef enum
{
	Off = 0,
  On = 1,
}MOTER_Mode;

typedef enum
{
	V_FLIP = 0,
  V_NORMAL = 1,

}V_FLIP_Mode;

/* Private variables ---------------------------------------------------------*/
struct Projector_current{
		unsigned short  red_current ;
		unsigned short  green_current;
		unsigned short  blue_current;
};

struct Projector_brightness{
	  struct Projector_current briness[10]; //100 ,85 ,60 ,50 ,37
};

struct Projector_CCT{
		unsigned short start_head ;
		unsigned short tuning_valid;
		unsigned short briness_index;
		unsigned short cct_index;
		struct Projector_brightness cct[3]; // 7500, 9000
		unsigned short end_tail;
};


struct reg_set{
		unsigned short reg_addr ;
		unsigned short reg_value;
};

struct projector_gamma_date{
		unsigned short pannel_len ;
		unsigned short cxd3354_len;
		struct reg_set *pannel_reg;
		struct reg_set *cxd3354_reg;
};


struct Projector_WC{
		unsigned short power_num ;
		unsigned short gain;
		unsigned short offset;
		unsigned short l;
};


struct Projector_Gama{
		unsigned short  start_head ;
		unsigned short  gamma_valid;
		unsigned char   gamma_reg[88*3];
};

struct Projector_Csc{
		unsigned short  start_head ;
		short  flag ;
		struct reg_set  csc_reg[42];
};

struct Projector_parameter{
    unsigned int  valid ;
    unsigned int  red_current ;
    unsigned int  blue_current;
    unsigned int  green_current;
		unsigned int	current_valid;
    unsigned int  contrast;
    unsigned int  brightness;
	  unsigned int  brightness_valid;
		unsigned int  cct;
		unsigned int  cct_valid;
    unsigned int  hflip;
		unsigned int	hflip_valid;
	  unsigned int  vflip;
		unsigned int	vflip_valid;
		unsigned int  display_size;
		unsigned int	display_size_valid;
	  unsigned int  keystone;
		unsigned int  keystone_valid;
		unsigned int  side_keystone;
		unsigned int  side_keystone_valid;
		unsigned int  throwratio;
		unsigned int  rgbmode;
		unsigned int  modeseclection;
	  unsigned int  r_gain;
		unsigned int  g_gain;
		unsigned int  b_gain;
		unsigned int  fan_speed;
		unsigned int  motor_status;
		struct Projector_WC wc_data[4];
		unsigned int  wc_valid;
		unsigned char wc[438];
		struct Projector_Gama gamma_data;
		struct Projector_CCT projector_tuning;
		unsigned char lt89121_valid;
		unsigned char reserved[3];//don't modify alignment 8
};

struct Projector_date{
    unsigned short start_head ;
    unsigned short command;
    unsigned short data0;
    unsigned short data1;
    unsigned short data2;
		unsigned short data3;
		unsigned short data4;
		unsigned short data5;
    unsigned short checksum;
    unsigned short end_tail;
};

struct Projector_wb_date{
		unsigned short start_head ;
		unsigned short direction ;
		unsigned short power_num ;
		unsigned short gain ;
		unsigned short offset ;
		unsigned short l ;
		unsigned char reg[132];
};

struct asu_date{
    unsigned char  chip_addr;
    unsigned char  w_r; // 0 write ,1 read, other fails
    unsigned short reg_addr;
    unsigned short reg_value;
};

/* Exported functions prototypes ---------------------------------------------*/
extern void A100_UartCmdHandler(uint8_t *pRx,uint8_t length);
extern uint8_t A100_SetFan12Speed(uint32_t speed);
extern uint8_t A100_SetFan34Speed(uint32_t speed);
extern uint8_t A100_SetFan5Speed(uint32_t speed);
extern void A100_I2cWriteCxd3554(uint8_t I2cAddr,uint16_t I2cReg, uint8_t I2cData);
extern void A100_I2cReadCxd3554(uint8_t I2cAddr,uint16_t I2cReg, uint8_t *I2cData);
extern void A100_I2cWriteSxmb241(uint8_t I2cAddr,uint8_t I2cReg,uint8_t I2cData);
extern void A100_I2cReadSxmb241(uint8_t I2cAddr, uint8_t I2cReg, uint8_t *I2cData);
uint16_t adc_GetTsOut(void);
uint16_t adc_GetPsOut(void);
extern uint16_t adc_GetLDTemp(void);
extern uint16_t adc_GetAdcVal(uint16_t *val);
extern void A100_display_on(uint16_t on);
extern uint8_t Motor_start(uint8_t dir, uint16_t steps);
extern void A100_SetRGBCurrent(void);
extern uint8_t A100_SetRedCurrent(uint16_t RedCurrent);
extern uint8_t A100_SetGreenCurrent(uint16_t RedCurrent);
extern uint8_t A100_SetBlueCurrent(uint16_t RedCurrent);
extern void A100_Dump_Cxd3554(uint16_t begin, uint16_t end);
extern uint16_t TPL1401_ReadI2C_Byte(uint8_t DevAddr, uint8_t RegAddr);
/* Exported variables --------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif
