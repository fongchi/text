#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include "stm32f0xx_conf.h"
#include "stdbool.h"
//#include "miscellaneous.h"


typedef enum 
{
  LED1 = 0,
  LED2 = 1,
} Led_TypeDef;


#define RTC_CLOCK_SOURCE_LSE   /* LSE used as RTC source clock */

#define BUZZ_TIMEOUT                    1
#define LED_TIMEOUT                     2
#define KEY_TIMEOUT                     3
#define BZ_LED1_TIMEOUT                 4
#define MOTOR_TIMEOUT                   5
#define RESET_KEY_DOWN                  6


#define LEDn                             2

#define LED1_PIN                         GPIO_Pin_8
#define LED1_GPIO_PORT                   GPIOC
#define LED1_GPIO_CLK                    RCC_AHBPeriph_GPIOC
  
#define LED2_PIN                         GPIO_Pin_9
#define LED2_GPIO_PORT                   GPIOC
#define LED2_GPIO_CLK                    RCC_AHBPeriph_GPIOC

#define BZ_PIN                           GPIO_Pin_1              //  BZ
#define BZ_GPIO_PORT                     GPIOA
#define BZ_GPIO_CLK                      RCC_AHBPeriph_GPIOA

#define NFC_INTERFACE_PIN                GPIO_Pin_0
#define NFC_INTERFACE_PORT               GPIOB
#define NFC_INTERFACE_CLK                RCC_AHBPeriph_GPIOB

#define LCD_BL                            GPIO_Pin_1  //1/7
#define LCD_DC                            GPIO_Pin_2  //2/8
#define NFC_NSS_PIN                       GPIO_Pin_4  //4              /* PA.04 */
#define NFC_NSS_GPIO_PORT                 GPIOA                        /* GPIOA */
#define NFC_NSS_GPIO_PORT2                GPIOB 
#define NFC_NSS_GPIO_PORT3                GPIOC
#define NFC_NSS_GPIO_CLK                  RCC_AHBPeriph_GPIOA
#define NFC_NSS_SOURCE                    GPIO_PinSource4
#define NFC_NSS_AF                        GPIO_AF_0

#define NFC_IRQ_IN_PIN                 GPIO_Pin_9
#define NFC_IRQ_IN_GPIO_PORT           GPIOA
#define NFC_IRQ_IN_GPIO_CLK            RCC_AHBPeriph_GPIOA
#define NFC_IRQ_IN_SOURCE              GPIO_PinSource9

#define NFC_IRQ_OUT_PIN                 GPIO_Pin_10
#define NFC_IRQ_OUT_GPIO_PORT           GPIOA
#define NFC_IRQ_OUT_GPIO_CLK            RCC_AHBPeriph_GPIOA
#define NFC_IRQ_OUT_SOURCE              GPIO_PinSource10

// set state on SPI_NSS pin
#define RFTRANS_95HF_NSS_LOW() 					GPIO_ResetBits(NFC_NSS_GPIO_PORT, NFC_NSS_PIN)
#define RFTRANS_95HF_NSS_HIGH()  				GPIO_SetBits  (NFC_NSS_GPIO_PORT, NFC_NSS_PIN)
// set state on IRQ_In pin
#define RFTRANS_95HF_IRQIN_LOW() 				GPIO_ResetBits(NFC_IRQ_IN_GPIO_PORT, NFC_IRQ_IN_PIN)	
#define RFTRANS_95HF_IRQIN_HIGH()  			GPIO_SetBits(NFC_IRQ_IN_GPIO_PORT, NFC_IRQ_IN_PIN)

#define RFTRANS_95HF_INF_LOW() 				GPIO_ResetBits(NFC_INTERFACE_PORT, NFC_INTERFACE_PIN)	
#define RFTRANS_95HF_INF_HIGH()  			GPIO_SetBits(NFC_INTERFACE_PORT, NFC_INTERFACE_PIN)

#define BUZZ_TIMEOUT                    1
#define LED_TIMEOUT                     2
#define KEY_TIMEOUT                     3
#define BZ_LED1_TIMEOUT                 4
#define MOTOR_TIMEOUT                   5
#define RESET_KEY_DOWN                  6

#define DEBUG_COM                      USART2
#define DEBUG_COM_CLK                  RCC_APB1Periph_USART2

#define DEBUG_COM_TX_PIN               GPIO_Pin_2
#define DEBUG_COM_TX_GPIO_PORT         GPIOA
#define DEBUG_COM_TX_GPIO_CLK          RCC_AHBPeriph_GPIOA
#define DEBUG_COM_TX_SOURCE            GPIO_PinSource2
#define DEBUG_COM_TX_AF                GPIO_AF_1

#define DEBUG_COM_RX_PIN               GPIO_Pin_3
#define DEBUG_COM_RX_GPIO_PORT         GPIOA
#define DEBUG_COM_RX_GPIO_CLK          RCC_AHBPeriph_GPIOA
#define DEBUG_COM_RX_SOURCE            GPIO_PinSource3
#define DEBUG_COM_RX_AF                GPIO_AF_1
#define DEBUG_COM_IRQn                 USART2_IRQn

#define EXTI_RFTRANS_95HF_PREEMPTION_PRIORITY	1
#define EXTI_RFTRANS_95HF_IRQ_CHANNEL					EXTI4_15_IRQn

/* -------------------------------------------------------------------------- 
* Delay TIMER configuration (ms)
* -------------------------------------------------------------------------- */ 
#define TIMER_DELAY														TIM3
#define TIMER_DELAY_PERIOD										47
#define TIMER_DELAY_PRESCALER									1000
#define TIMER_DELAY_CLOCK											RCC_APB1Periph_TIM3
																						
/* -------------------------------------------------------------------------- 
* Delay TIMER configuration (탎)
* --------------------------------------------------------------------------- */ 
#define TIMER_US_DELAY												TIM3
#define TIMER_US_DELAY_PERIOD									47
#define TIMER_US_DELAY_PRESCALER							100
#define TIMER_US_DELAY_CLOCK									RCC_APB1Periph_TIM3
#define TIMER_DELAY_PREEMPTION_PRIORITY		  	0
#define TIMER_DELAY_IRQ_CHANNEL							  TIM3_IRQn

/** 
 * @brief  TIMER definitions 
 */
 
/* -------------------------------------------------------------------------- 
* timeout configuration (us)
* --------------------------------------------------------------------------
* 72 MHz / 72 = 1MHz (1us )
* 1탎 * 1000 + 1탎 ~= 1ms	
* -------------------------------------------------------------------------- */
#define TIMER_TIMEOUT													TIM15
#define TIMER_TIMEOUT_PERIOD									1000
#define TIMER_TIMEOUT_PRESCALER								48
#define TIMER_TIMEOUT_CLOCK										RCC_APB2Periph_TIM15
#define TIMER_TIMEOUT_PREEMPTION_PRIORITY			2
#define TIMER_TIMEOUT_IRQ_CHANNEL							TIM15_IRQn


/* -------------------------------------------------------------------------- 
* timeout configuration (ms)
* --------------------------------------------------------------------------
* 72 MHz / 72 = 1MHz (1us )
* 1탎 * 1000 + 1탎 ~= 1ms	
* -------------------------------------------------------------------------- */
#define APPLI_TIMER_TIMEOUT										TIM14
#define APPLI_TIMER_TIMEOUT_PERIOD						1000
#define APPLI_TIMER_TIMEOUT_PRESCALER					48
#define APPLI_TIMER_TIMEOUT_CLOCK							RCC_APB1Periph_TIM14
#define APPLI_TIMER_TIMEOUT_PREEMPTION_PRIORITY			1
#define APPLI_TIMER_TIMEOUT_IRQ_CHANNEL							TIM14_IRQn

void SL_LEDInit(Led_TypeDef Led);
void SL_LEDOn(Led_TypeDef Led);
void SL_LEDOff(Led_TypeDef Led);
void SL_LEDToggle(Led_TypeDef Led);
void SL_GPIO_INIT(void);
void Interrupts_Config( void );
void Timer_Config( void );
void Timer_RCC_Config( void );
void Timer_Structure_Config( void );
void delay_ms( uint16_t delay );
void delay_us( uint16_t delay );
void delayHighPriority_ms( uint16_t delay );
void decrement_delay( void );

void drvInt_TimeoutTimerConfig( void );
void drvInt_AppliTimeoutTimerConfig( void );
void StartTimeOut( uint16_t delay );
void StopTimeOut( void );
void StartAppliTimeOut( uint16_t delay );
void StopAppliTimeOut( void );

void IRQOut_Config( void );

void SendSPINSSPulse( void );

void drvInt_Enable_Reply_IRQ( void );
void drvInt_Enable_RFEvent_IRQ( void );
void drvInt_Disable_95HF_IRQ( void );

/* SPI or UART configuration */
#ifdef CR95HF
void SPINSS_Config( void );
#endif

#endif
