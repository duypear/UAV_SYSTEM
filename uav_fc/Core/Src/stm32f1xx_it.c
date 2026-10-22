/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
#include "gps.h"

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/

void NMI_Handler(void)
{
  while (1) {
  }
}

void HardFault_Handler(void)
{
  while (1) {
  }
}

void MemManage_Handler(void)
{
  while (1) {
  }
}

void BusFault_Handler(void)
{
  while (1) {
  }
}

void UsageFault_Handler(void)
{
  while (1) {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/******************************************************************************/

void USART1_IRQHandler(void)
{
  uart1_irq_handler();
}

void USART3_IRQHandler(void)
{
  gps_uart_irq_handler();
}

void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim3);
}

void TIM4_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim4);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
