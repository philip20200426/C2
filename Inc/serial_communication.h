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
#define UART_COMMAND_HEAD 		101
#define UART_COMMAND_END  		102

#define I2C_WRITE        		  3
#define I2C_READ       	 	    4

#define SET_GPIOC 						5
#define SET_WEC 							6
#define GET_OE_FAN_SPEED 			7
#define SET_OE_FAN_SPEED 			8
#define SET_HAPTIC_START 		  9
#define SET_CURRENT_MODE  			16
#define SET_INT_PATTERN_TYPE 	18
#define SET_SOURCE_INPUT 		  22
#define SET_LED				   		  42
#define GET_VERSION				    43
#define SET_KST				        44
#define GET_KST				        45
#define GET_CURRENT_MODE  			46
#define SET_HORIZONTAL 		  	48
#define GET_HORIZONTAL 		  	49
#define SET_VERTICAL 		  		50
#define GET_VERTICAL 		  		51
#define SET_LED_INPUT 		  		52
#define WRITE_CXD3554					53
#define READ_CXD3554 					54
#define GET_ADC_TEMP 		  		55	//ld temperature sensor
#define SET_DISPLAY_SIZE 		 	56
#define SET_DISPLAY_ON					57
#define SET_MOTOR_START				58

#define GET_DISPLAY_SIZE     	63
#define GET_BRITNRSS_NUM				64
//#define GET_KST_NUM						65
#define GET_DISPLAY_SIZE_NUM  	66
#define GET_HORIZONTAL_NUM			67
#define GET_VERTICAL_NUM     	68
#define GET_LED_ON_OFF      		69
#define GET_ADC_PS      				70  //ps out(photo sensor)
#define GET_ADC_TS      				71	//ts out(temperature sensor)
#define GET_TUNING_PARAMETER  	74
#define SET_CCT_MODE  					75
#define GET_CCT_MODE  					76
#define GET_CCT_NUM						77
#define SET_R_GAIN           	80
#define GET_R_GAIN           	81
#define SET_G_GAIN           	82
#define GET_G_GAIN           	83
#define SET_B_GAIN           	84
#define GET_B_GAIN           	85
#define SET_SIDE_KST				  	86
#define GET_SIDE_KST				  	87
#define GET_SIDE_KST_NUM     	89
#define SET_WC_EN      				90
#define READ_GAMA      				91
#define WRITE_GAMA      				92
#define WRITE_SXRD      				93
#define SET_IPG      					94
#define READ_CXD      					95
#define WRITE_CXD      				96
#define SET_WC_EN1      				97
#define SET_WC_EN2      				98
#define GET_WC      						99
#define SET_WC_SAVE      			100
//101 don't use
#define SET_PARAMRTER     			102
#define GET_CURRENT     				111
#define GET_GAMA      					112
#define SET_GAMA      					113
#define SAVE_GAMA      				114

#define TEST_IDC      					120
#define TEST_DC      					121
#define TEST_ODC      					122

#define SET_TUNING_PARAMETER  	201
#define SET_WC_PARAMETER  			202
#define SET_GAMA_PARAMETER  		203
#define GET_GAMA_PARAMETER  		204

#define BRITNRSS_NUM						2
#define HORIZONTAL_NUM					1
#define VERTICAL_NUM      			1		
#define CCT_NUM      						2
#define DISPLAY_SIZE_NUM  			5


#ifndef UART_BUFFER_MAX_SIZE
  #define UART_BUFFER_MAX_SIZE 			256
#endif
#define VERSION0 	0
#define VERSION1 	0
#define VERSION2 	1

#define FLASH_USER_START_ADDR   (FLASH_BASE + (62 * FLASH_PAGE_SIZE))   /*0~63 Start @ of user Flash area */
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
  FAN_SPEED_DEFAULT= 15,
  FAN_SPEED_SLOW   = 10,
  FAN_PEED_STOP    = 0

}FAN_Speed;

/*
typedef enum
{
  SET_LED_RRGGBBGG = 0,
  SET_LED_RRGGRRBB = 1,
  SET_LED_RRBBGGBB = 2,
  SET_LED_RRGGBBWW = 3,
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
		unsigned char kst_val[16];
		unsigned short  kst_valid;
		unsigned short  kst_index;
#ifdef CONFIG_KST_INDEX	
	  unsigned int 	keystone;
		unsigned int  keystone_valid;
		unsigned int  side_keystone;
		unsigned int  side_keystone_valid;
#endif
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
		unsigned char reserved[7];//don't modify alignment 8
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
extern void UartCmdHandler(uint8_t *pRx,uint8_t length);
extern uint8_t SetFan12Speed(uint32_t speed);
extern uint8_t SetFan34Speed(uint32_t speed);
extern uint8_t SetFan5Speed(uint32_t speed);
extern void I2cWriteCxd3554(uint8_t I2cAddr,uint16_t I2cReg, uint8_t I2cData);
extern void I2cReadCxd3554(uint8_t I2cAddr,uint16_t I2cReg, uint8_t *I2cData);
extern void I2cWriteSxmb241(uint8_t I2cAddr,uint8_t I2cReg,uint8_t I2cData);
extern void I2cReadSxmb241(uint8_t I2cAddr, uint8_t I2cReg, uint8_t *I2cData);
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
extern void Dump_Cxd3554(uint16_t begin, uint16_t end);
extern uint16_t TPL1401_ReadI2C_Byte(uint8_t DevAddr, uint8_t RegAddr);
/* Exported variables --------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif
