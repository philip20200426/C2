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
#define TIME_BASE_50US_16MS     (16000  / TIME_BASE_TIMER)
#define TIME_BASE_50US_24MS     (24000  / TIME_BASE_TIMER)
#define TIME_BASE_50US_32MS     (32000  / TIME_BASE_TIMER)
#define TIME_BASE_50US_64MS     (64000  / TIME_BASE_TIMER)
#define TIME_BASE_50US_256MS    (256000  / TIME_BASE_TIMER)
#define TIME_BASE_50US_512MS    (512000  / TIME_BASE_TIMER)
#define TIME_BASE_50US_1S       (1000000 / TIME_BASE_TIMER)
#define TIME_BASE_50US_5S       (5384000 / TIME_BASE_TIMER)
#define TIME_BASE_50US_10S      (10000000/ TIME_BASE_TIMER)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t CounterBaseStep;
volatile uint32_t Counter16ms = 0;
volatile uint32_t Counter24ms = 0;
volatile uint32_t Counter32ms = 0;
volatile uint32_t Counter64ms = 0;
volatile uint32_t Counter256ms = 0;
volatile uint32_t Counter512ms = 0;
volatile uint32_t Counter1s = 0;
volatile uint32_t Counter5s = 0;
volatile uint32_t CounterBaseStep = 0;
volatile uint32_t LedCounter = 0;

volatile _Bool Flag16ms = 0;
volatile _Bool Flag24ms = 0;
volatile _Bool Flag32ms = 0;
volatile _Bool Flag64ms = 0;
volatile _Bool Flag256ms = 0;
volatile _Bool Flag512ms = 0;
volatile _Bool Flag1s = 0;
volatile _Bool Flag5s = 0;
volatile _Bool Flag_Sxrd241_Alarm = 0;
volatile _Bool Flag_Projector_On = 0;
volatile _Bool Flag_LIMIT_L = 0;
volatile _Bool Flag_LIMIT_R = 0;
volatile _Bool Flag_FanLock = 0;
volatile _Bool Flag_LT9211_Int = 0;

uint32_t g_fan_value;
uint8_t gpiopin = 0;

void SysTaskDispatch(void);
extern void A100_Variables_Init(void);
extern void A5931_init(void);
extern void A100_LcosSetKst(void);
extern void A100_LcosSetWC(void);
extern void A100_LcosSetGamma(void);
extern void A100_LcosSetGain(void);
extern void A100_LcosSetFlip(void);
extern void A100_GetParameter(void);
extern void A100_ReceiveUart1Data(void);
extern HAL_StatusTypeDef A100_SetBootPinMode(void);
extern void A100_Lcos_CheckRegError(uint8_t local);
#ifdef CONFIG_LT89121
extern uint8_t LT89121_Update_Fw(void);
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void A100_SetRGB_Enable(GPIO_PinState value)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, value); //LD_EN enable LD DC-DC EN
}

void 	A100_display_on(uint16_t on)
{
	if(on==1)
	{
		A100_SetRGB_Enable(GPIO_PIN_SET);
	}
	else
	{
		A100_SetRGB_Enable(GPIO_PIN_RESET);		
	}
}	

void A100_GpioConfig(void)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);    //FAN_OFF enable Fan
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);    //CX3554_3V3_EN 
	A100_LcosVideoMute(VIDEO_UNMUTE);
	A100_SetRGB_Enable(GPIO_PIN_SET); //LD_EN	
}

void A100_LT89121_Reset(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_Delay(20);		
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET); //LT89121 reset	
}

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
  /* USER CODE BEGIN 2 */
	printf("main() Projector_parameter size=%d \r\n",sizeof(struct Projector_parameter));

	A100_Variables_Init();
  A100_GetParameter();
  A100_GpioConfig();	
  A100_SetFan12Speed(FAN_SPEED_DEFAULT);
  A100_SetFan34Speed(FAN_SPEED_DEFAULT);
  A100_SetFan5Speed(FAN_SPEED_DEFAULT);
	A5931_init();	
  HAL_TIM_Base_Start_IT(&htim6);	
	
#ifdef CONFIG_LT89121
	LT89121_Update_Fw();
#endif

  A100_SetRGBCurrent();
  A100_LcosInitSequence();
  A100_LcosSetPatternSize();
  A100_LcosSetIntPattern();
	A100_LcosSetRRGGBBGGMode();
	A100_LcosInitGamma();
	
	A100_LcosSetFlip();
	A100_LcosSetKst();
	A100_LcosSetWC();
	A100_LcosSetGamma();
	A100_LcosSetGain();

	A100_LT89121_Reset();
	A100_SetBootPinMode();
	A100_ReceiveUart1Data();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    SysTaskDispatch();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
	uint8_t val;
	
	printf("HAL_GPIO_EXTI_Rising_Callback: 0x%x \r\n", GPIO_Pin);

	if((GPIO_PIN_3 == GPIO_Pin)) /* Projector On  LIMIT_L interrupt */
	{
		val  = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3);
		if(Flag_LIMIT_L != val) {
			Flag_LIMIT_L = val;
			printf("HAL_GPIO_EXTI_Rising_Callback: PC3 val:0x%x \r\n", Flag_LIMIT_L);
		}
	}

	if((GPIO_PIN_4 == GPIO_Pin)) /*  LIMIT_R interrupt */
	{
		val = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_4);
		if(Flag_LIMIT_R != val) {
			Flag_LIMIT_R = val;
			printf("HAL_GPIO_EXTI_Rising_Callback: PC4 val:0x%x \r\n", Flag_LIMIT_R);
		}
	}
	
	if((GPIO_PIN_5 == GPIO_Pin)) /*  FanLock interrupt */
	{
		val = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_5);
		if(Flag_FanLock != val) {
			Flag_FanLock = val;
			printf("HAL_GPIO_EXTI_Callback: PC5 val:0x%x \r\n", Flag_FanLock);
		}
	}		
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
	uint8_t val;

	printf("HAL_GPIO_EXTI_Falling_Callback: 0x%x \r\n", GPIO_Pin);

	if((GPIO_PIN_3 == GPIO_Pin)) /* Projector On  LIMIT_L interrupt */
	{
		val  = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3);
		if(Flag_LIMIT_L != val) {
			Flag_LIMIT_L = val;
			printf("HAL_GPIO_EXTI_Falling_Callback: PC3 val:0x%x \r\n", Flag_LIMIT_L);
		}
	}

	if((GPIO_PIN_4 == GPIO_Pin)) /*  LIMIT_R interrupt */
	{
		val = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_4);
		if(Flag_LIMIT_R != val) {
			Flag_LIMIT_R = val;
			printf("HAL_GPIO_EXTI_Falling_Callback: PC4 val:0x%x \r\n", Flag_LIMIT_R);
		}
	}
	
	if((GPIO_PIN_15 == GPIO_Pin) && (0 == Flag_Sxrd241_Alarm)) /* SXRD241 SHORT DET interrupt */
	{
		val  = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15);
		printf("HAL_GPIO_EXTI_Callback: PB15 val:0x%x \r\n", val);
		Flag_Sxrd241_Alarm = 1;
	}	
	
	if((GPIO_PIN_9 == GPIO_Pin)) /*  LT9211 interrupt */
	{
		val = HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_9);
		printf("HAL_GPIO_EXTI_Callback: PD9 val:0x%x \r\n", val);
		Flag_LT9211_Int = 1;
	}			
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

void SysTask16ms(void)
{

}

void SysTask256ms(void)
{

}

void SysTask512ms(void)
{

}

void SysTask1s(void)
{
  HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_9);
  //HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
}

void SysTask5s(void)
{
	uint32_t ld_temp = 0;
	
	ld_temp = adc_GetLDTemp();
	printf("LD TEMP:%d \r\n",ld_temp);

#if 0	
	if(ld_temp > 2700)
	{   if(ld_temp > 2800)
			{
				if(Flag_Projector_On != 0)
				{
					Flag_Projector_On = 0;
					A100_SetRGB_Enable((GPIO_PinState)Flag_Projector_On);
				}	
			}	
			A100_SetFan12Speed(FAN_SPEED_FULL);
			A100_SetFan34Speed(FAN_SPEED_FULL);
			A100_SetFan5Speed(FAN_SPEED_FULL);
	}
	else if(ld_temp < 2300)
	{
			if(Flag_Projector_On != 1)
			{
				Flag_Projector_On = 1;
				A100_SetRGB_Enable((GPIO_PinState)Flag_Projector_On);
			}	
			A100_SetFan12Speed(FAN_SPEED_SLOW);
			A100_SetFan34Speed(FAN_SPEED_SLOW);
			A100_SetFan5Speed(FAN_SPEED_SLOW);
	}
	else
	{
			if(Flag_Projector_On != 1)
			{
				Flag_Projector_On = 1;
				A100_SetRGB_Enable((GPIO_PinState)Flag_Projector_On);
			}	
			g_fan_value = FAN_SPEED_SLOW + (ld_temp-2300)*30;
			A100_SetFan12Speed(g_fan_value);
			A100_SetFan34Speed(g_fan_value);
			A100_SetFan5Speed(g_fan_value);
	}	
#endif
}

void SysTaskDispatch(void)
{
    SysTask0ms();  	

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
}

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
