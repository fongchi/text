
#include "hwconfig.h"


GPIO_TypeDef* GPIO_PORT[LEDn] = {LED1_GPIO_PORT, LED2_GPIO_PORT};
const uint16_t GPIO_PIN[LEDn] = {LED1_PIN, LED2_PIN};
const uint32_t GPIO_CLK[LEDn] = {LED1_GPIO_CLK, LED2_GPIO_CLK};

static __IO uint16_t					counter_delay_ms;
uint16_t delay_appli = 0;
uint16_t delay_timeout = 0;

extern __IO uint8_t						uTimeOut;
extern volatile bool					uAppliTimeOut;

/* Private functions Prototype -----------------------------------------------*/
static void TimerDelay_us_Config	( void );
static void TimerDelay_ms_Config	( void );

/**
  * @brief  Configures LED GPIO.
  * @param  Led: Specifies the Led to be configured. 
  *          This parameter can be one of following parameters:
  *            @arg LED1
  *            @arg LED2
  * @retval None
  */
void SL_LEDInit(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* Enable the GPIO_LED Clock */
  RCC_AHBPeriphClockCmd(GPIO_CLK[Led], ENABLE);

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_PIN[Led];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIO_PORT[Led], &GPIO_InitStructure);
  GPIO_PORT[Led]->BSRR = GPIO_PIN[Led];
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on. 
  *          This parameter can be one of following parameters:
  *            @arg LED1
  *            @arg LED2
  * @retval None
  */
void SL_LEDOn(Led_TypeDef Led)
{
  GPIO_PORT[Led]->BRR = GPIO_PIN[Led];
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off. 
  *          This parameter can be one of following parameters:
  *            @arg LED1
  *            @arg LED2
  * @retval None
  */
void SL_LEDOff(Led_TypeDef Led)
{
  GPIO_PORT[Led]->BSRR = GPIO_PIN[Led];
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled. 
  *          This parameter can be one of following parameters:
  *            @arg LED1
  *            @arg LED2

  * @retval None
  */
void SL_LEDToggle(Led_TypeDef Led)
{
  GPIO_PORT[Led]->ODR ^= GPIO_PIN[Led];
}


/**
  * @brief  Configures SL001 GPIO.
  *         input pin - NFC_IRQ_OUT
  *         output pin - BZ, SPI1 NSS, NFC_IRQ_IN, NFC_Interface
  * @param  None
  * @retval None
  */
void SL_GPIO_INIT(void)
{
   GPIO_InitTypeDef  GPIO_InitStructure;
  GPIO_SetBits(GPIOA,GPIO_Pin_9);
	GPIO_ResetBits(GPIOB,GPIO_Pin_4);
  /* Enable the GPIO Clock */
  RCC_AHBPeriphClockCmd(NFC_NSS_GPIO_CLK | BZ_GPIO_CLK |NFC_IRQ_IN_GPIO_CLK | NFC_IRQ_OUT_GPIO_CLK | NFC_INTERFACE_CLK, ENABLE);

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = BZ_PIN | NFC_NSS_PIN | NFC_IRQ_IN_PIN ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA , &GPIO_InitStructure);     // BZ_GPIO_PORT , NFC_NSS_GPIO_PORT, NFC_IRQ_IN_GPIO_PORT are all GPIOA
  
  GPIO_InitStructure.GPIO_Pin = NFC_INTERFACE_PIN ;
  GPIO_Init(NFC_INTERFACE_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = NFC_IRQ_OUT_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_Init(NFC_IRQ_OUT_GPIO_PORT, &GPIO_InitStructure); 
}

/**
 *	@brief  Timeout timer config
 *  @param  None
 *  @retval None
 */
void drvInt_TimeoutTimerConfig(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
/* -------------------------------------------------------------------------- 
	 * TimeOut TIMER configuration
	 * -------------------------------------------------------------------------- 
	 * 48 MHz / 4800 = 10KHz (100us)
	 * 100us * 300 + 100us ~= 30ms	
	 * -------------------------------------------------------------------------- */
	TIM_TimeBaseStructure.TIM_Period 					= TIMER_TIMEOUT_PERIOD;     
	TIM_TimeBaseStructure.TIM_Prescaler 			= TIMER_TIMEOUT_PRESCALER;       
	TIM_TimeBaseStructure.TIM_ClockDivision 	= TIM_CKD_DIV1;      
	TIM_TimeBaseStructure.TIM_CounterMode 		= TIM_CounterMode_Down;	  
	/* Update the timeout timer (TIM3) 	*/
	TIM_TimeBaseInit(TIMER_TIMEOUT, &TIM_TimeBaseStructure);
	
	TIM_UpdateRequestConfig(TIMER_TIMEOUT, TIM_UpdateSource_Global);
	
	TIM_ClearITPendingBit(TIMER_TIMEOUT, TIM_IT_Update);
		
	/* Enable TIMER Update interrupt */
	TIM_ITConfig(TIMER_TIMEOUT, TIM_IT_Update, ENABLE);
	
	/* Disable timer	*/
	TIM_Cmd(TIMER_TIMEOUT, DISABLE);
}


/**
 *	@brief  This function starts the time out used to avoid the STM32 freeze
 *  @param  delay : delay in tenth of milliseconds (100us).
 *  @retval None
 */
void StartTimeOut( uint16_t delay )
{
	/* Set the TimeOut flag to false */
	uTimeOut 	 = false;
	delay_timeout = delay;
	/* Set the timer counter */
	TIM_SetCounter(TIMER_TIMEOUT, delay);
  /* Enable the Time out timer */
	TIM_Cmd(TIMER_TIMEOUT, ENABLE);
}

/**
 *	@brief  Stop the timer used for the time out
 *  @param  None
 *  @retval None
 */
void StopTimeOut( void )
{	
	/* Disable the Time out timer */
	TIM_Cmd(TIMER_TIMEOUT, DISABLE);	
}

/**
 *	@brief  Timeout timer config
 *  @param  None
 *  @retval None
 */
void drvInt_AppliTimeoutTimerConfig(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
/* -------------------------------------------------------------------------- 
	 * TimeOut TIMER configuration
	 * -------------------------------------------------------------------------- 
	 * 48 MHz / 48 = 1MHz (1us)
	 * 1us * 1000 + 1us ~= 1ms	
	 * -------------------------------------------------------------------------- */
	TIM_TimeBaseStructure.TIM_Period 					= APPLI_TIMER_TIMEOUT_PERIOD;     
	TIM_TimeBaseStructure.TIM_Prescaler 			= APPLI_TIMER_TIMEOUT_PRESCALER;       
	TIM_TimeBaseStructure.TIM_ClockDivision 	= TIM_CKD_DIV1;      
	TIM_TimeBaseStructure.TIM_CounterMode 		= TIM_CounterMode_Down;	  
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	/* Update the timeout timer (TIM14) 	*/
	TIM_TimeBaseInit(APPLI_TIMER_TIMEOUT, &TIM_TimeBaseStructure);
	
	TIM_UpdateRequestConfig(APPLI_TIMER_TIMEOUT, TIM_UpdateSource_Global);
	
	TIM_ClearITPendingBit(APPLI_TIMER_TIMEOUT, TIM_IT_Update);
		
	/* Enable TIMER Update interrupt */
	TIM_ITConfig(APPLI_TIMER_TIMEOUT, TIM_IT_Update, ENABLE);

	/* Disable timer	*/
	TIM_Cmd(APPLI_TIMER_TIMEOUT, DISABLE);
}

/**
 *	@brief  This function starts the time out used to avoid the STM32 freeze
 *  @param  delay : delay in tenth of milliseconds (100us).
 *  @retval None
 */
void StartAppliTimeOut( uint16_t delay )
{
	/* Set the TimeOut flag to false */
	uAppliTimeOut 	 = false;
	delay_appli = delay;
	TIM_SetCounter(APPLI_TIMER_TIMEOUT, delay);
	/* TIM2 enable counter */
  TIM_Cmd(APPLI_TIMER_TIMEOUT, ENABLE);
}

/**
 *	@brief  Stop the timer used for the time out
 *  @param  None
 *  @retval None
 */
void StopAppliTimeOut( void )
{	
	/* Disable the Time out timer */
	TIM_Cmd(APPLI_TIMER_TIMEOUT, DISABLE);	
}



/**
 *	@brief  This function configures the Timers
 *  @param  None
 *  @retval None
 */
void Timer_Config( void )
{
	Timer_RCC_Config( );
	Timer_Structure_Config( );
}


/**
 *	@brief  This function configures RCC for the Timers
 *  @param  None
 *  @retval None
 */
void Timer_RCC_Config( void )
{
	/*	enable TIM3 & TIM6 & TIM14 */
	RCC_APB1PeriphClockCmd(		TIMER_TIMEOUT_CLOCK 	|
														APPLI_TIMER_TIMEOUT_CLOCK 	|
														TIMER_DELAY_CLOCK 		,
														ENABLE);
}

/**
 *	@brief  Structure configuration for the Timers
 *  @param  None
 *  @retval None
 */
void Timer_Structure_Config( void )
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;	
	
	/* TIM3 driver timeout */
	drvInt_TimeoutTimerConfig();
	
	/* TIM4 appli timeout */
	drvInt_AppliTimeoutTimerConfig();
	
	/* -------------------------------------------------------------------------- 
	 * Delay TIMER configuration
	 * --------------------------------------------------------------------------
	 * 72 MHz / 72 = 1MHz (1µs)
	 * 1µs * 1000 + 1µs ~= 1ms	
	 * -------------------------------------------------------------------------- */ 
	TIM_TimeBaseStructure.TIM_Period 			= TIMER_DELAY_PERIOD;      
	TIM_TimeBaseStructure.TIM_Prescaler 		= (TIMER_DELAY_PRESCALER-1);       
	TIM_TimeBaseStructure.TIM_ClockDivision 	= TIM_CKD_DIV1;    
	TIM_TimeBaseStructure.TIM_CounterMode 		= TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIMER_DELAY, &TIM_TimeBaseStructure);
	
	TIM_UpdateRequestConfig(TIMER_DELAY, TIM_UpdateSource_Global);
	
	TIM_ClearITPendingBit(TIMER_DELAY, TIM_IT_Update);
		
	/* Enable TIMER Update interrupt */
	TIM_ITConfig(TIMER_DELAY, TIM_IT_Update, ENABLE);

}

/**
 *	@brief Structure configuration for the Timer2 in ms
 *  @param  None
 *  @retval None
 */
static void TimerDelay_ms_Config( void )
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	/* -------------------------------------------------------------------------- 
	 * Delay TIMER configuration
	 * -------------------------------------------------------------------------- */ 
	TIM_TimeBaseStructure.TIM_Period 					= TIMER_DELAY_PERIOD;      
	TIM_TimeBaseStructure.TIM_Prescaler 			= TIMER_DELAY_PRESCALER;       
	TIM_TimeBaseStructure.TIM_ClockDivision 	= TIM_CKD_DIV1;    
	TIM_TimeBaseStructure.TIM_CounterMode 		= TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIMER_DELAY, &TIM_TimeBaseStructure);
	
	TIM_UpdateRequestConfig(TIMER_DELAY, TIM_UpdateSource_Global);
	
	TIM_ClearITPendingBit(TIMER_DELAY, TIM_IT_Update);
		
	/* Enable TIMER Update interrupt */
	TIM_ITConfig(TIMER_DELAY, TIM_IT_Update, ENABLE);

	/* Disable timer	*/
	TIM_Cmd(TIMER_DELAY, DISABLE);

}

/**
 *	@brief Structure configuration for the Timer2 in us
 *  @param  None
 *  @retval None
 */
static void TimerDelay_us_Config( void )
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	/* -------------------------------------------------------------------------- 
	* Delay TIMER configuration (us)
	* -------------------------------------------------------------------------- */ 
	TIM_TimeBaseStructure.TIM_Period 			= TIMER_US_DELAY_PERIOD;      
	TIM_TimeBaseStructure.TIM_Prescaler 		= TIMER_US_DELAY_PRESCALER;       
	TIM_TimeBaseStructure.TIM_ClockDivision 	= TIM_CKD_DIV1;    
	TIM_TimeBaseStructure.TIM_CounterMode 		= TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIMER_US_DELAY, &TIM_TimeBaseStructure);
	
	TIM_UpdateRequestConfig(TIMER_US_DELAY, TIM_UpdateSource_Global);
	
	TIM_ClearITPendingBit(TIMER_US_DELAY, TIM_IT_Update);
		
	/* Enable TIMER Update interrupt */
	TIM_ITConfig(TIMER_US_DELAY, TIM_IT_Update, ENABLE);

	/* Disable timer	*/
	TIM_Cmd(TIMER_DELAY, DISABLE);

}


/**
 *	@brief  Time delay in millisecond
 *  @param  delay : delay in ms.
 *  @retval none
 */
void delay_ms(uint16_t delay)
{
	counter_delay_ms = (delay+1);
	
	TimerDelay_ms_Config ();
	
	TIM_SetCounter(TIMER_DELAY, 0);
	/* TIM3 enable counter */
  TIM_Cmd(TIMER_DELAY, ENABLE);
	/* Wait for 'delay' milliseconds */
	//while(counter_delay_ms != 0);
	/* TIM3 disable counter */
	TIM_Cmd(TIMER_DELAY, DISABLE);
}


/**
 *	@brief  Time delay in microsecond
 *  @param  delay : delay in us.
 *  @retval none
 */
void delay_us(uint16_t delay)
{
	counter_delay_ms = delay/100 + 1;
	
	TimerDelay_us_Config ();

	TIM_SetCounter(TIMER_US_DELAY, 0);
	/* TIM3 enable counter */
  TIM_Cmd(TIMER_US_DELAY, ENABLE);
	/* Wait for 'delay' us */
	while(counter_delay_ms != 0);
	/* TIM3 disable counter */
	TIM_Cmd(TIMER_US_DELAY, DISABLE);
}

/**
 *	@brief  This function decrements the counter every millisecond used by the function delay_ms
 *  @param  None
 *  @retval None
 */
void decrement_delay(void)
{
	if(counter_delay_ms != 0)
	{
		/* Decrements the counter */ 
		counter_delay_ms--;
	}
}


/**
 *	@brief  Configures the interrupts
 *  @param  None
 *  @retval None
 */
void Interrupts_Config (void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

#ifdef SPI_INTERRUPT_MODE_ACTIVATED
		/* Enable and set RF transceiver IRQ to the lowest priority */
  	NVIC_InitStructure.NVIC_IRQChannel 										= EXTI_RFTRANS_95HF_IRQ_CHANNEL; 
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= EXTI_RFTRANS_95HF_PREEMPTION_PRIORITY;
		NVIC_InitStructure.NVIC_IRQChannelCmd 								= ENABLE;
		NVIC_Init(&NVIC_InitStructure); 						
#endif /* SPI_INTERRUPT_MODE_ACTIVATED */

	/* Enable and set TIMER IRQ used for timeout : TIM15*/
	NVIC_InitStructure.NVIC_IRQChannel 											= TIMER_TIMEOUT_IRQ_CHANNEL;
	NVIC_InitStructure.NVIC_IRQChannelPriority 		          = TIMER_TIMEOUT_PREEMPTION_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd 									= ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable and set TIMER IRQ used for appli timeout : TIM14*/
	NVIC_InitStructure.NVIC_IRQChannel 											= APPLI_TIMER_TIMEOUT_IRQ_CHANNEL;
	NVIC_InitStructure.NVIC_IRQChannelPriority 		          = APPLI_TIMER_TIMEOUT_PREEMPTION_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd 									= ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* Enable and set TIMER IRQ used for delays : TIM3*/
	NVIC_InitStructure.NVIC_IRQChannel 					 						= TIMER_DELAY_IRQ_CHANNEL;
	NVIC_InitStructure.NVIC_IRQChannelPriority 		          = TIMER_DELAY_PREEMPTION_PRIORITY;

	NVIC_InitStructure.NVIC_IRQChannelCmd 									= ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

/**
 *	@brief  this function sends a negative pulse on SPI_NSS pin
 */
void SendSPINSSPulse(void)
{
		RFTRANS_95HF_NSS_HIGH();
    delay_ms(5);
		//delayHighPriority_ms(1);
		RFTRANS_95HF_NSS_LOW();
  delay_ms(5);
		//delayHighPriority_ms(1);
		RFTRANS_95HF_NSS_HIGH();
}

