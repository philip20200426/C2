#ifndef __motor_H
#define __motor_H
#ifdef __cplusplus
 extern "C" {
#endif
#include "stm32g0xx_hal.h"
#include "main.h"

#define MOTOR_GPIO_Port GPIOC
#define IN1_Pin GPIO_PIN_8
#define IN2_Pin GPIO_PIN_7
#define IN3_Pin GPIO_PIN_6
#define IN4_Pin GPIO_PIN_9
#define SLEEP_Pin GPIO_PIN_11

void Motor_Drive_8step(uint8_t);
void Motor_Drive_4step(uint8_t);
void Motor_Foreward_4step(void);
void Motor_Foreward_8step(void);
void Motor_Backward_4step(void);
void Motor_Backward_8step(void);

#ifdef __cplusplus
}
#endif
#endif 
