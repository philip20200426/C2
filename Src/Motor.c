#include "Motor.h"
#include "tim.h"
#include "stdio.h"
#include "string.h"
#include "serial_communication.h"

extern void Uart_Send_Response(uint16_t command, uint8_t* data, uint8_t size );
void MotorStop_UartRes(uint8_t result);

#define BONGINGNUM  300
static uint8_t MotorPositon = MOTOR_CENTER;
static int bongingcount = BONGINGNUM;

void MotorLimit_DealWith(uint8_t lr)
{
	bongingcount = BONGINGNUM;
	if(lr == LIMIT_LEFT)
		MotorPositon = MOTOR_LEFT;
	else 	if(lr == LIMIT_RIGHT)
		MotorPositon = MOTOR_RIGHT;
	
	MotorStop_UartRes(MotorPositon);
}

#ifndef MOTOR_DRV8428
//A3916
uint32_t MotorStepcnt = 0;
uint8_t MotorDirection = 0;
uint8_t MotorStep = 0;

void Motor_Stop(void);

void Motor_Exit_Sleep(void)
{
	uint8_t val;
	
	val = HAL_GPIO_ReadPin(MOTOR_GPIO_Port,SLEEP_Pin);	
	if(val == 0) {
		HAL_GPIO_WritePin(MOTOR_GPIO_Port, SLEEP_Pin, GPIO_PIN_SET);
		HAL_Delay(3);
	}
}

void Motor_Drive_8step(uint8_t step)
{
	Motor_Exit_Sleep();
	switch(step)
	{
		case 0x08:
			HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN2_Pin, GPIO_PIN_RESET);
	 		HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN3_Pin, GPIO_PIN_RESET);
  		HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN4_Pin, GPIO_PIN_RESET);
			break;
		
		case 0x07:
			HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN2_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN3_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN4_Pin, GPIO_PIN_SET);
			break;
		
		case 0x06:
			HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN2_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN3_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN4_Pin, GPIO_PIN_SET);
			break;

		case 0x05:
			HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN2_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN3_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN4_Pin, GPIO_PIN_SET);
			break;

		case 0x04:
			HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN2_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN3_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN4_Pin, GPIO_PIN_RESET);
			break;
		
		case 0x03:
			HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN2_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN3_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN4_Pin, GPIO_PIN_RESET);
			break;
		
		case 0x02:
			HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN2_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN4_Pin, GPIO_PIN_RESET);
			break;
		
		case 0x01:
		  HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN4_Pin, GPIO_PIN_RESET);
			break;
	}
}

void Motor_Drive_4step(uint8_t step)
{
	Motor_Exit_Sleep();
	switch(step)
	{
		case 0x04:
			HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN2_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN3_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN4_Pin, GPIO_PIN_SET);
			break;
		
		case 0x03:
			HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN2_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN3_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN4_Pin, GPIO_PIN_SET);
			break;
		
		case 0x02:
			HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN2_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN3_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN4_Pin, GPIO_PIN_RESET);
			break;
		
		case 0x01:
		  HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN4_Pin, GPIO_PIN_RESET);
			break;
	}
}

void Motor_Stop(void)
{
	MotorStepcnt=0;
	
	HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO_Port,IN4_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO_Port, SLEEP_Pin, GPIO_PIN_RESET);//enter sleep
}

void MotorStop_UartRes(uint8_t result)
{
	Motor_Stop();
	Uart_Send_Response(CMD_SET_FOCUSMOTOR, &result, 1);
}


void Motor_Foreward_4step(void)
{
	if(MotorStep<4)
		MotorStep++;
	else
		MotorStep=1;
	
	Motor_Drive_4step(MotorStep);
}

void Motor_Foreward_8step(void)
{
	if(MotorStep<8)
		MotorStep++;
	else
		MotorStep=1;

	Motor_Drive_8step(MotorStep);
}

void Motor_Backward_8step(void)
{
	if(MotorStep >1)
		MotorStep--;
	else
		MotorStep=8; 
	
	Motor_Drive_8step(MotorStep);
}

void Motor_Backward_4step(void)
{
	if(MotorStep>1)
		MotorStep--;
	else
		MotorStep=4;
	
	Motor_Drive_4step(MotorStep);
}

void Motor_Set_Para(uint8_t Dir, uint32_t Step) 
{
	MotorDirection=Dir;
	MotorStepcnt=Step;//(360*85/7.5) = 4080 one cycle
}


void Motor_Step_AutoRun_8step(void)
{
	while(MotorStepcnt--) {
		bongingcount--;
		if(1==MotorDirection)
			Motor_Foreward_8step();
		else
			Motor_Backward_8step();
		
		HAL_Delay(1);
	}
	
	if(MotorPositon != MOTOR_CENTER) {
	 if(bongingcount < 0)
		 MotorPositon = MOTOR_CENTER;
	}	
	MotorStop_UartRes(MotorPositon);
}

void Motor_Step_AutoRun_4step(void)
{
	while(MotorStepcnt--) {
		bongingcount--;
		if(1==MotorDirection)
			Motor_Foreward_4step();
		else
			Motor_Backward_4step();
		
		HAL_Delay(1);
	}
	
	if(MotorPositon != MOTOR_CENTER) {
	 if(bongingcount < 0)
		 MotorPositon = MOTOR_CENTER;
	}	
	MotorStop_UartRes(MotorPositon);
}

uint8_t Motor_start(uint8_t dir, uint16_t steps)
{
	Motor_Set_Para(dir, steps);
	Motor_Step_AutoRun_8step();
	
	return 0;
}

void StepperVar_Init(void)
{

}
#else //TI DRV8428

typedef struct Stepper_Driving_Stu
{
	uint16_t count_steps; //	
	uint16_t num_steps; // number of steps to move
	uint16_t speed; //speed of motor once running
	
} Stepper_Driving_Stu_t;

Stepper_Driving_Stu_t gStepper;

void StepperVar_Init(void)
{
	memset(&gStepper, 0, sizeof(struct Stepper_Driving_Stu));
	gStepper.num_steps = 1;
}

void Motor_Stop(void)
{
	HAL_TIM_PWM_Stop_IT(&htim16,TIM_CHANNEL_1);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET); //enter sleep	
}

void MotorStop_UartRes(uint8_t result)
{
	Motor_Stop();
	gStepper.count_steps = 0;
	
	Uart_Send_Response(CMD_SET_FOCUSMOTOR, &result, 1);
}

uint8_t Motor_start(uint8_t dir, uint16_t steps)
{	
	printf("Motor_start:%d dir:%d\r\n", steps, dir);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, (GPIO_PinState)!dir);
	HAL_Delay(1);

	gStepper.num_steps = steps;
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET); //exit SLEEP
	
	//HAL_TIM_OC_Start_IT(&htim16, TIM_CHANNEL_1);	
	HAL_TIM_PWM_Start_IT(&htim16, TIM_CHANNEL_1);
	return 0;
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
#if 0
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		/* Get the Capture Compare Register value */
		//uint32_t uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		//printf("ReadCapturedValue: %d \r\n",uhCapture);
		gStepper.count_steps ++;
		if(gStepper.count_steps == gStepper.num_steps) 
		{
			HAL_TIM_OC_Stop_IT(&htim16, TIM_CHANNEL_1);
		}
		printf("ReadStepsValue: %d \r\n",gStepper.count_steps);
	}
#endif
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		gStepper.count_steps++;
		bongingcount--;
		//printf("HAL_TIM_PWM_PulseFinishedCallback: %d \r\n",cnt);
		if(gStepper.count_steps == gStepper.num_steps)
		 {
			 if(MotorPositon != MOTOR_CENTER) {
				 if(bongingcount < 0)
					 MotorPositon = MOTOR_CENTER;
			 }
				MotorStop_UartRes(MotorPositon);
		 }
 } 
}
#endif
