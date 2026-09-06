/**
 * @file Servo.c
 * @brief TIM3 PWM servo driver — PA6 CH1, PA7 CH2
 *
 * 50Hz (20ms), pulse 0.5ms~2.5ms maps to 0~180 deg (SG90)
 * Timer clock: APB1 x2 = 200MHz, PSC=199 -> 1MHz, ARR=19999 -> 20ms
 */
#include "Servo.h"
#include "main.h"

TIM_HandleTypeDef htim3;

#define SERVO_PWM_MIN_US   500U
#define SERVO_PWM_MAX_US   2500U

static uint16_t angle_to_pulse(float angle_deg)
{
  if (angle_deg < 0.f) {
    angle_deg = 0.f;
  } else if (angle_deg > 180.f) {
    angle_deg = 180.f;
  }
  return (uint16_t)(SERVO_PWM_MIN_US +
                    (angle_deg / 180.f) * (float)(SERVO_PWM_MAX_US - SERVO_PWM_MIN_US));
}

static void servo_set_pulse(uint32_t channel, uint16_t pulse_us)
{
  __HAL_TIM_SET_COMPARE(&htim3, channel, pulse_us);
}

static void tim3_hw_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /**TIM3 GPIO: PA6=CH1, PA7=CH2 */
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void MX_TIM3_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  tim3_hw_init();

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
}

void SERVO_Init(void)
{
  MX_TIM3_Init();
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  SERVO1_ANGLE(90.f);
  SERVO2_ANGLE(90.f);
}

void SERVO1_ANGLE(float angle_deg)
{
  servo_set_pulse(TIM_CHANNEL_1, angle_to_pulse(angle_deg));
}

void SERVO2_ANGLE(float angle_deg)
{
  servo_set_pulse(TIM_CHANNEL_2, angle_to_pulse(angle_deg));
}

void SERVO1_OPEN(void){
  SERVO1_ANGLE(90.0f);
}
void SERVO1_CLOSE(void){
  SERVO1_ANGLE(0.0f);
}
void SERVO2_OPEN(void){
  SERVO2_ANGLE(90.0f);
}
void SERVO2_CLOSE(void){
  SERVO2_ANGLE(0.0f);
}
