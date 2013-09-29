/**
  ******************************************************************************
  * @file    stm32fxxx_it.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    19-March-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides all exceptions handler and peripherals interrupt
  *          service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "usb_irq.h"
#include "usb_core.h"
#include "usbd_core.h"
#include "usbd_hid_core.h"
#include "usb_conf.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
extern USB_OTG_CORE_HANDLE           USB_OTG_dev;
extern uint32_t USBD_OTG_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
#ifdef USB_OTG_HS_DEDICATED_EP1_ENABLED 
extern uint32_t USBD_OTG_EP1IN_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
extern uint32_t USBD_OTG_EP1OUT_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
#endif

/******************************************************************************/
/*             Cortex-M Processor Exceptions Handlers                         */
/******************************************************************************/
/**
* @brief  This function handles EXTI15_10_IRQ Handler.
* @param  None
* @retval None
*/
#ifdef USE_USB_OTG_FS  
void OTG_FS_WKUP_IRQHandler(void)
{
  if(USB_OTG_dev.cfg.low_power)
  {
    /* Reset SLEEPDEEP and SLEEPONEXIT bits */
    SCB->SCR &= (uint32_t)~((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
    
#ifdef USE_STM3210C_EVAL
    
    SystemInit();
#else    
    /* After wake-up from sleep mode, reconfigure the system clock */
    RCC_HSEConfig(RCC_HSE_ON);
    
    /* Wait till HSE is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET)
    {}
    
    /* Enable PLL */
    RCC_PLLCmd(ENABLE);
    
    /* Wait till PLL is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {}
    
    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    
    /* Wait till PLL is used as system clock source */
    while (RCC_GetSYSCLKSource() != 0x08)
    {}
    
#endif    
    USB_OTG_UngateClock(&USB_OTG_dev);
  }
  EXTI_ClearITPendingBit(EXTI_Line18);
}
#endif

/**
* @brief  This function handles EXTI15_10_IRQ Handler.
* @param  None
* @retval None
*/
#ifdef USE_USB_OTG_HS  
void OTG_HS_WKUP_IRQHandler(void)
{
  
  if (remote_wakeup ==1)
  {
    remote_wakeup = 0;
  }
  else
  {
    if(USB_OTG_dev.cfg.low_power)
    {
      /* Reset SLEEPDEEP and SLEEPONEXIT bits */
      SCB->SCR &= (uint32_t)~((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
      
      /* After wake-up from sleep mode, reconfigure the system clock */
      RCC_HSEConfig(RCC_HSE_ON);
      
      /* Wait till HSE is ready */
      while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET)
      {}
      
      /* Enable PLL */
      RCC_PLLCmd(ENABLE);
      
      /* Wait till PLL is ready */
      while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
      {}
      
      /* Select PLL as system clock source */
      RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
      
      /* Wait till PLL is used as system clock source */
      while (RCC_GetSYSCLKSource() != 0x08)
      {}
      
      USB_OTG_UngateClock(&USB_OTG_dev);
    }
  }
  EXTI_ClearITPendingBit(EXTI_Line20);
}
#endif

/**
* @brief  This function handles OTG_HS Handler.
* @param  None
* @retval None
*/
#ifdef USE_USB_OTG_HS  
void OTG_HS_IRQHandler(void)
#else
void OTG_FS_IRQHandler(void)
#endif
{
  USBD_OTG_ISR_Handler (&USB_OTG_dev);
}

#ifdef USB_OTG_HS_DEDICATED_EP1_ENABLED 
/**
* @brief  This function handles EP1_IN Handler.
* @param  None
* @retval None
*/
void OTG_HS_EP1_IN_IRQHandler(void)
{
  USBD_OTG_EP1IN_ISR_Handler (&USB_OTG_dev);
}

/**
* @brief  This function handles EP1_OUT Handler.
* @param  None
* @retval None
*/
void OTG_HS_EP1_OUT_IRQHandler(void)
{
  USBD_OTG_EP1OUT_ISR_Handler (&USB_OTG_dev);
}
#endif

/******************************************************************************/
/*                 STM32Fxxx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32fxxx.s).                                               */
/******************************************************************************/

/**
* @brief  This function handles PPP interrupt request.
* @param  None
* @retval None
*/
/*void PPP_IRQHandler(void)
{
}*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
