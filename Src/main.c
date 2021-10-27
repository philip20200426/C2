/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "serial_communication.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIME_BASE_TIMER         (50)
#define TIME_BASE_50US_1MS    	(1000  / TIME_BASE_TIMER)
#define TIME_BASE_50US_4MS    	(4000  / TIME_BASE_TIMER)
#define TIME_BASE_50US_8MS    	(8000  / TIME_BASE_TIMER)
#define TIME_BASE_50US_16MS     (16000  / TIME_BASE_TIMER)
#define TIME_BASE_50US_24MS     (24000  / TIME_BASE_TIMER)
#define TIME_BASE_50US_32MS     (32000  / TIME_BASE_TIMER)
#define TIME_BASE_50US_64MS     (64000  / TIME_BASE_TIMER)
#define TIME_BASE_50US_256MS    (256000  / TIME_BASE_TIMER)
#define TIME_BASE_50US_512MS    (512000  / TIME_BASE_TIMER)
#define TIME_BASE_50US_1S       (1000000 / TIME_BASE_TIMER)
#define TIME_BASE_50US_5S       (5384000 / TIME_BASE_TIMER)
#define TIME_BASE_50US_10S      (10000000/ TIME_BASE_TIMER)
#define TIME_BASE_50US_20S      (20000000/ TIME_BASE_TIMER)
#define TIME_BASE_50US_30S      (30000000/ TIME_BASE_TIMER)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t CounterBaseStep;
volatile uint32_t Counter1ms = 0;
volatile uint32_t Counter4ms = 0;
volatile uint32_t Counter8ms = 0;
volatile uint32_t Counter16ms = 0;
volatile uint32_t Counter24ms = 0;
volatile uint32_t Counter32ms = 0;
volatile uint32_t Counter64ms = 0;
volatile uint32_t Counter256ms = 0;
volatile uint32_t Counter512ms = 0;
volatile uint32_t Counter1s = 0;
volatile uint32_t Counter5s = 0;
volatile uint32_t Counter20s = 0;
volatile uint32_t CounterBaseStep = 0;
volatile uint32_t LedCounter = 0;

volatile _Bool Flag1ms = 0;
volatile _Bool Flag4ms = 0;
volatile _Bool Flag8ms = 0;
volatile _Bool Flag16ms = 0;
volatile _Bool Flag24ms = 0;
volatile _Bool Flag32ms = 0;
volatile _Bool Flag64ms = 0;
volatile _Bool Flag256ms = 0;
volatile _Bool Flag512ms = 0;
volatile _Bool Flag1s = 0;
volatile _Bool Flag5s = 0;
volatile _Bool Flag20s = 0;

volatile _Bool Flag_Sxrd241_Alarm = 0;
volatile _Bool Flag_Projector_On = 1;
volatile _Bool Flag_Lvds_Clk_Stb = 1;
volatile _Bool Flag_LT9211_Int = 0;
volatile _Bool g_FanNOFlag = 1;
volatile _Bool g_FanMode = 0;
volatile _Bool g_UserDisplayOff = 0;
volatile _Bool Flag_MatMode = 0;
volatile _Bool FlagIwdg = 1;
uint16_t g_overtemp_cnt = 0;


void Fan_Auto_Control(void);
void SysTaskDispatch(void);
extern void Variables_Init(void);
extern void ThreePhaseMotorDriver_init(void);
extern void LcosSetKst(void);
extern void LcosSetWP(void);
extern void LcosSetIntBCHS(void);
extern void LcosSetGain(void);
extern void LcosSetFlip(void);
extern void LcosInitWec(void);
extern void LcosInitCsco(void);
extern void GetParameter(void);
extern void GetColorTempParameter(void);
extern void ReceiveUart1Data(void);
extern HAL_StatusTypeDef SetBootPinMode(void);
extern void Lcos_CheckRegError(uint8_t local);
extern 	uint8_t GetFanSpeed(void);
extern void MotorLimit_DealWith(uint8_t lr);
extern uint8_t Motor_start(uint8_t dir, uint16_t steps);
extern void UartCommandParser(void);
extern void LcosSetColorTempBlock(void);
extern void LcosSetBchs(void);
extern void LcosSetCe1d(void);
extern void LcosSetCebc(void);
extern void LcosSetCeacc(void);
extern uint8_t getIwdgFlag(void);
#ifdef USE_LT9211_LVDS2MIPI
extern void	LT9211_Init(void);
extern void LT9211_Pattern_Init(void);
extern int lt9211_get_lvds_clkstb(uint8_t* count);
extern void LT9211_Video_Reset(void);
extern uint8_t get_LT9211_Mode(void);
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void FanInit(void)
{
	SetFan12Speed(30);
	SetFan34Speed(30);
	SetFan5Speed(30);
}

void SetRGB_Enable(GPIO_PinState value)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, value); //LD_EN enable LD DC-DC EN
}

void 	display_on(uint16_t on)
{
	if(on==1)
	{
		SetRGB_Enable(GPIO_PIN_SET);
		FanInit();
	}
	else
	{
		SetRGB_Enable(GPIO_PIN_RESET);		
	}
}	

void GpioConfig(void)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);    //FAN_OFF enable Fan
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);    //CX3554_3V3_EN 
	LcosVideoMute(VIDEO_UNMUTE);
}

void LT89121_Reset(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_Delay(20);		
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET); //LT89121 reset	
}

#define CONFIG_IWDG

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM6_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM15_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM16_Init();
  //MX_IWDG_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */

  GetParameter();
	GetColorTempParameter();

	Variables_Init();
  GpioConfig();
	FanInit();
	ThreePhaseMotorDriver_init();	
  HAL_TIM_Base_Start_IT(&htim6);	
	
  SetRGBCurrent();
  LcosInitSequence();
  LcosSetPatternSize();
  LcosSetIntPattern();
	LcosSetIntBCHS();

	LcosSetColorTempBlock();
	LcosSetFlip();
	LcosSetKst();
	LcosSetWP();
	LcosSetBchs();
	LcosSetCe1d();
	LcosSetCebc();
#ifdef CONFIG_CEACC
	LcosSetCeacc();
#endif	
	LcosInitCsco();
	LcosInitWec();
#ifdef USE_LT9211_LVDS2MIPI
	LT9211_Init();
	//LT9211_Pattern_Init();
#else
	LT89121_Reset();
#endif
	SetRGB_Enable(GPIO_PIN_SET); //LD_EN

	SetBootPinMode();
	ReceiveUart1Data();
#ifdef CONFIG_IWDG	
	if(getIwdgFlag() != 0)
	{
		MX_IWDG_Init();
	}
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    SysTaskDispatch();

#ifdef CONFIG_IWDG
		if(getIwdgFlag() != 0)
		{
			/* Refresh the IWDG 4S overflow*/
			HAL_IWDG_Refresh(&hiwdg);
		}
#endif
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks 
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
	printf("HAL_GPIO_EXTI_Rising_Callback: 0x%x \r\n", GPIO_Pin);

	if((GPIO_PIN_3 == GPIO_Pin)) /* LIMIT_L interrupt */
	{
			MotorLimit_DealWith(LIMIT_LEFT);
	}

	if((GPIO_PIN_4 == GPIO_Pin)) /*  LIMIT_R interrupt */
	{
			MotorLimit_DealWith(LIMIT_RIGHT);
	}
		
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
	uint8_t val;

	printf("HAL_GPIO_EXTI_Falling_Callback: 0x%x \r\n", GPIO_Pin);
	if((GPIO_PIN_15 == GPIO_Pin) && (0 == Flag_Sxrd241_Alarm)) /* SXRD241 SHORT DET interrupt */
	{
		val  = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15);
		printf("HAL_GPIO_EXTI_Callback: PB15 val:0x%x \r\n", val);
		Flag_Sxrd241_Alarm = 1;
	}	
	
#if 0	
	if((GPIO_PIN_8 == GPIO_Pin)) /* PA8 LT9211 interrupt */
	{
		val = HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_9);
		printf("HAL_GPIO_EXTI_Callback: PD9 val:0x%x \r\n", val);
		Flag_LT9211_Int = 1;
	}
#endif
	
#ifdef CONFIG_MAT	
	if((GPIO_PIN_1 == GPIO_Pin)) /* PD1 MAT MODE		*/
	{
		val = HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_1);
		printf("HAL_GPIO_EXTI_Callback: PD1 val:0x%x \r\n", val);
		Flag_MatMode = 1;
	}	
#endif	
	
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == htim6.Instance)
  {
			if(CounterBaseStep < 0xFFFFFFFF)
			{
				CounterBaseStep++;
			}
			else
			{
				CounterBaseStep = 0;
			}
			if(LedCounter > 0)
			{
				LedCounter--;
			}
			else
			{
				LedCounter = 0xFFFFFFFF;
			}
	
			if(TIME_BASE_50US_1MS == ++Counter1ms)
  	  {
				Flag1ms = 1;
				Counter1ms = 0;
  	  }	
			
			if(TIME_BASE_50US_4MS == ++Counter4ms)
  	  {
				Flag4ms = 1;
				Counter4ms = 0;
  	  }	
			
			if(TIME_BASE_50US_8MS == ++Counter8ms)
  	  {
				Flag8ms = 1;
				Counter8ms = 0;
  	  }	
			
			if(TIME_BASE_50US_16MS == ++Counter16ms)
  	  {
				Flag16ms = 1;
				Counter16ms = 0;
  	  }
			
			if(TIME_BASE_50US_24MS == ++Counter24ms)
  	  {
				Flag24ms = 1;
				Counter24ms = 0;
  	  }
			if(TIME_BASE_50US_32MS == ++Counter32ms)
  	  {
				Flag32ms = 1;
				Counter32ms = 0;
  	  }
			if(TIME_BASE_50US_64MS == ++Counter64ms)
  	  {
				Flag64ms = 1;
				Counter64ms = 0;
  	  }
					
  	  if(TIME_BASE_50US_256MS == ++Counter256ms)
  	  {
				Flag256ms = 1;
				Counter256ms = 0;
  	  }
  	  if(TIME_BASE_50US_512MS == ++Counter512ms)
  	  {
				Flag512ms = 1;
				Counter512ms = 0;
  	  }
  	  if(TIME_BASE_50US_1S == ++Counter1s)
  	  {
				Flag1s = 1;
				Counter1s = 0;
  	  }
			if(TIME_BASE_50US_5S == ++Counter5s)
  	  {
				Flag5s = 1;
				Counter5s = 0;
  	  }
			if(TIME_BASE_50US_20S == ++Counter20s)
  	  {
				Flag20s = 1;
				Counter20s = 0;
  	  }
  }
}

/* 50us/step, º∆ ±ŒÛ≤Ó «50us*/
void Delay50Us(uint32_t timer)
{
  CounterBaseStep = 1;
  while(CounterBaseStep <= timer);
  return;
}

void SysTask0ms(void)
{

}

void SysTask1ms(void)
{

}

void SysTask4ms(void)
{
	UartCommandParser();
}
void SysTask8ms(void)
{

}

void SysTask16ms(void)
{

}

void SysTask256ms(void)
{
#ifdef USE_LT9211_LVDS2MIPI
	if(Flag_Lvds_Clk_Stb == 0)
#else
	if(Flag_Projector_On == 0)
#endif
	{
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_9);
	}
}

void SysTask512ms(void)
{
#ifdef USE_LT9211_LVDS2MIPI
	uint8_t count = 30;
	
	if(!get_LT9211_Mode())
	{
		if(!lt9211_get_lvds_clkstb(&count))
		{
			if(Flag_Lvds_Clk_Stb)
			{
				printf("\r\n lvds clk not stable \r\n");
				Flag_Lvds_Clk_Stb = 0;
			}
			
			LT9211_Video_Reset();
		}
	}
#endif
}

#ifdef CONFIG_MAT
int cnt_1s = 0;
_Bool Flag_Limit_OK = 1;
extern uint8_t LimitPosition;
#endif

void SysTask1s(void)
{
#ifdef USE_LT9211_LVDS2MIPI
	if(Flag_Lvds_Clk_Stb == 1)
#else
	if(Flag_Projector_On == 1) 
#endif
	{
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_9);
	}
/*************************MAT TEST *************************************/	
#ifdef CONFIG_MAT
  static int flag_enter = 1;

	if(Flag_MatMode != 0 && flag_enter)
	{
#ifdef USE_LT9211_LVDS2MIPI
		LT9211_Pattern_Init();
#endif
		flag_enter = 0;
	}
	
	//motor test
	if(Flag_MatMode != 0 && Flag_Limit_OK)
	{
		if(cnt_1s == 0)
		{ 
			if(LimitPosition == LIMIT_LEFT)
				Motor_start(0, 10000);
			else
				Flag_Limit_OK = 0;
		}
		else if(cnt_1s == 5)
		{
			if(LimitPosition == LIMIT_RIGHT)
				Motor_start(1, 10000);
			else
				Flag_Limit_OK = 0;			
		}
			
		cnt_1s++;
		if(cnt_1s == 10) cnt_1s = 0;
	}
#endif		
}

void SysTask5s(void)
{

}
void SysTask20s(void)
{
	Fan_Auto_Control();
}
void SysTaskDispatch(void)
{
    SysTask0ms();  	
		if(Flag1ms)
		{
		   Flag1ms = 0;
		   SysTask1ms();
		}
		if(Flag4ms)
		{
		   Flag4ms = 0;
		   SysTask4ms();
		}
		if(Flag8ms)
		{
		   Flag8ms = 0;
		   SysTask8ms();
		}
		
		if(Flag16ms)
		{
		   Flag16ms = 0;
		   SysTask16ms();
		}
		if(Flag24ms)
		{
		   Flag24ms = 0;
		}		
		if(Flag32ms)
		{
		   Flag32ms = 0;
		}			
		if(Flag64ms)
		{
		   Flag64ms = 0;
		}	
		if(Flag256ms)
		{
		   Flag256ms = 0;
			 SysTask256ms();
		}
		if(Flag512ms)
		{
		   Flag512ms = 0;
		   SysTask512ms();
		}
		if(Flag1s)
		{
		   Flag1s = 0;
		   SysTask1s();
		}
		if(Flag5s)
		{
		   Flag5s = 0;
		   SysTask5s();
		}
		if(Flag20s)
		{
		   Flag20s = 0;
		   SysTask20s();
		}
}
/*-------------------------------------------------------------------------------------------*/	
const uint16_t LD_RT_TABLE[] =
{/* 0~55   45*/
	3318,  //0 degree
	3151,	 
	2993,	
	2844,	
	2703,	
	2570,	
	2444,	
	2325,	
	2213,	
	2107,	
	2007,	
	1912,	
	1822,	
	1736,	
	1656,	
	1579,	
	1507,
	1438,
	1373,
	1311,
	1252,
	1196,
	1144,
	1093,
	1045,
	1000,		//25
	957, 
	916, 
	877, 
	839, 
	804, 
	770, 
	738, 
	708, 
	679, 
	651, 
	624, 
	599, 
	575, 
	552, 
	530, 
	509, 
	489, 
	470, 
	452, 
	434, 
	417, 
	401, 
	386, 
	372, 
	358, 
	344, 
	332, 
	319, 
	308, 
	296,  //55 degree
	286,
	275,
	265,
	256,
	247,
	238,
	230,
	222,
	214,
	207,
	199,
	193,
	186,
	180,
	174,
	168,
	162,
	157,
	152,
	147,
	142,
	137,
	133,
	129,
	125,
	121,
	117,
	113,
	110,
	106,
	103,
	100,
	97,
	94,
	91   //90 degree
};

uint16_t GetLd_RT_Temp(uint16_t adc_val)
{
	uint16_t ohm, i;
	
	ohm = (4096000/adc_val - 1000);
	if(ohm >  LD_RT_TABLE[0]) 
	{
		printf("LD OHM:%d \r\n",ohm);	
		if (ohm > 18744) return 50; //no ld RT register
		return 0;
	}
	
	for (i = 0; i < sizeof(LD_RT_TABLE)/sizeof(uint16_t); i++)
	{		
		if(LD_RT_TABLE[i] < ohm ) break;
	}
	
	return i;
}

/* -10 ~ +60  Recommended temperature is +45 to +55 */
uint16_t GetLcos_RT_Temp(uint16_t adc_val)
{//temp = (1839 - vol)/11
	uint16_t vol;
	
	vol = (adc_val*3300)/4096;
	if(vol > 1839) 
	{
		//printf("LCOS VOL:%d \r\n",vol);	
		return 0;
	}
	
	return (1839 - vol)/11;
}
/*-------------------------------------------------------------------------------------------*/	
//new fan parameter
const uint8_t LD_CTL_TABLE12[][2] =
{/* 0~55   45*/	
	{41, 21},//8
	{42, 21},
	{43, 21},		
	{44, 21},//		
	{45, 21},	
	{46, 21},	
	{47, 21},	
	{48, 21},	
	{49, 25},//9	
	{50, 25},	
	{51, 25},	
	{52, 42},//16	
	{53, 51},//20	
	{54, 62},//24
	{55, 72} //28
};

const uint8_t LD_CTL_TABLE34[][2] =
{/* 0~55   45*/	
	{41, 31},//12
	{42, 31},		
	{43, 31},		
	{44, 31},//	
	{45, 31},	
	{46, 31},	
	{47, 31},	
	{48, 31},	
	{49, 35},//13	
	{50, 35},	
	{51, 35},	
	{52, 51},//20	
	{53, 62},//24	
	{54, 72},//28
	{55, 85} //33
};

uint8_t get_ld_fan12pwm(uint8_t temp)
{
	uint8_t i;
	static uint8_t s_pwm = 0, s_temp = 0;
	
	if(temp < LD_CTL_TABLE12[0][0]) return LD_CTL_TABLE12[0][1];
	
	if(s_temp > 52){
		if(temp == s_temp - 1 || temp == s_temp - 2 || temp == s_temp - 3)
		{
			return s_pwm;
		}
	} else {
		if(temp == s_temp - 1 || temp == s_temp - 2)
		{
			return s_pwm;
		}
	}
	for (i = 0; i < sizeof(LD_CTL_TABLE12)/sizeof(uint8_t)/2; i++)
	{		
		if(LD_CTL_TABLE12[i][0] == temp ) 
		{
			s_temp = temp;
			s_pwm = LD_CTL_TABLE12[i][1];
			return s_pwm;
		}
	}
	
	return 100;
}

uint8_t get_ld_fan34pwm(uint8_t temp)
{
	uint8_t i;
	static uint8_t s_pwm = 0, s_temp = 0;
	
	if(temp < LD_CTL_TABLE34[0][0]) return LD_CTL_TABLE34[0][1];
	
	if(s_temp > 52){
		if(temp == s_temp - 1 || temp == s_temp - 2 || temp == s_temp - 3)
		{
			return s_pwm;
		}
	} else {
		if(temp == s_temp - 1 || temp == s_temp - 2)
		{
			return s_pwm;
		}
	}
	
	for (i = 0; i < sizeof(LD_CTL_TABLE34)/sizeof(uint8_t)/2; i++)
	{		
		if(LD_CTL_TABLE34[i][0] == temp )
		{
			s_temp = temp;
			s_pwm = LD_CTL_TABLE34[i][1];
			return s_pwm;
		}
	}
	
	return 100;
}

const uint8_t LCOS_CTL_TABLE[][2] =
{/*-10~60    +45 to +55 */
	{44, 0},
	{45, 30},
	{46, 30},
	{47, 30},
	{48, 30},
	{49, 40},
	{50, 40},
	{51, 40},
	{52, 40},
	{53, 50},
	{54, 50},
	{55, 50},	
	{56, 60},	
	{57, 70},	
	{58, 80},	
	{59, 90},	
	{60, 100}
};

uint8_t get_lcos_fanpwm(uint8_t temp)
{
	uint8_t i;
	
	if(temp < LCOS_CTL_TABLE[0][0]) return LCOS_CTL_TABLE[0][1];
		
	for (i = 0; i < sizeof(LCOS_CTL_TABLE)/sizeof(uint8_t)/2; i++)
	{		
		if(LCOS_CTL_TABLE[i][0] == temp ) return LCOS_CTL_TABLE[i][1];
	}

	return 100;
}
#define TEMP_OVER_CNT   8   //3min
void Fan_Auto_Control(void)
{
	uint16_t ld_adc = 0, lcos_adc = 0;
	uint16_t adc_val[3];
	uint16_t ld_temp, lcos_temp;
	uint8_t fan12pwm, fan34pwm, fan5pwm;
	
	ld_adc = adc_GetAdcVal(adc_val);
	lcos_adc = adc_val[2];
	ld_temp = GetLd_RT_Temp(ld_adc);
	lcos_temp = GetLcos_RT_Temp(lcos_adc);
	//printf("LD_TEMP:%d LCOS_TEMP:%d  LD ADC:%d  LCOS ADC:%d \r\n",ld_temp, lcos_temp, ld_adc, lcos_adc);
	//GetFanSpeed();
#if 1
	if(!g_FanMode)
	{
		if(Flag_Projector_On == 1)
		{
			fan12pwm = get_ld_fan12pwm(ld_temp);
			fan34pwm = get_ld_fan34pwm(ld_temp);
			fan5pwm = get_lcos_fanpwm(lcos_temp);

			if(g_UserDisplayOff && (fan5pwm == 0))
			{//Low LCOS temperature
				fan12pwm = 0;
				fan34pwm = 0;
			}
			
			SetFan12Speed(fan12pwm);
			SetFan34Speed(fan34pwm);		
			SetFan5Speed(fan5pwm);
		}
		
		if(ld_temp > 55 || lcos_temp > 60)
		{  	
			g_overtemp_cnt++;
			if(g_overtemp_cnt > TEMP_OVER_CNT)
			{
				if(Flag_Projector_On != 0)
				{
					Flag_Projector_On = 0;
					SetRGB_Enable((GPIO_PinState)Flag_Projector_On);
					SetFan12Speed(FAN_SPEED_FULL);
					SetFan34Speed(FAN_SPEED_FULL);
					SetFan5Speed(FAN_SPEED_FULL);	
				}	
			}
		}
		else 
		{
			g_overtemp_cnt = 0;
			if(ld_temp < 45 && lcos_temp < 50)
			{
				if(Flag_Projector_On != 1)
				{
					Flag_Projector_On = 1;
					SetRGB_Enable((GPIO_PinState)Flag_Projector_On);
				}	
			}
		}
	}
#endif	
}
/*-------------------------------------------------------------------------------------------*/	

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	printf("Error: %s  line:%d \r\n",__FILE__, __LINE__);
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	printf("Assert:file:%s line:%d \r\n",file, line);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
