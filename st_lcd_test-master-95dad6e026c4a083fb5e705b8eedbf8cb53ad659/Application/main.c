/**
  ******************************************************************************
  * @file    SPI/SPI_TwoBoards/DataExchangeInterrupt/main.c
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    24-July-2014
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
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
#include "main.h"
#include "stm320518_eval_lcd.h"

/** @addtogroup STM32F0xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup SPI_DataExchangeInterrupt
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MESSAGE1   "STM32F0xx CortexM0  " 
#define LINE(x) ((x) * (((sFONT *)LCD_GetFont())->Height))
#ifdef USE_STM320518_EVAL
  #define MESSAGE2   "   STM320518-EVAL   "
  #define MESSAGE3   "  Turn RV3(PC.01)    "
#else 
  #define MESSAGE2   "   STM32072B-EVAL   " 
  #define MESSAGE3   "  Turn RV3(PC.00)    "
#endif /* USE_STM320518_EVAL */
#define MESSAGE4   "   Potentiometer     "
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
SPI_InitTypeDef  SPI_InitStructure;

uint8_t TxBuffer[2] = {0x00,0x55};
uint8_t RxBuffer [RXBUFFERSIZE];

__IO uint8_t Rx_Idx = 0x00;
__IO uint8_t Tx_Idx = 0x00;

__IO uint8_t CmdTransmitted = 0x00;
__IO uint8_t CmdReceived = 0x00;
__IO uint8_t CmdStatus = 0x00;
__IO uint32_t TimeOut = 0x0;
uint8_t text2[50];
__IO uint16_t CCR3_Val = 10000;
__IO uint16_t CCR4_Val = 20000;
uint16_t PrescalerValue = 0;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
__IO uint8_t						uTimeOut;
volatile bool					uAppliTimeOut;
__IO uint16_t  ADC1ConvertedValue = 0, ADC1ConvertedVoltage = 0;
uint16_t ADCC1=0,ADCC1V=0;

/* Private function prototypes -----------------------------------------------*/
static void SPI_Config(void);
static void SysTickConfig(void);
void TIM3_Config(void);
static void TimeOut_UserCallback(void);
static TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength, uint8_t DataMask);
static void Fill_Buffer(uint8_t *pBuffer, uint16_t BufferLength);
void Display_Init(void);
static void Display(void);
static void ADC_Config(void);
static void ADC_Config2(void);
static void Display_Init(void);
//void ADC_Catch(uint16_t ADCC1,uint16_t ADCC1V,uint16_t ADC1ConvertedValue,uint16_t ADC1ConvertedVoltage);
typedef struct meowadc;
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
  this is done through SystemInit() function which is called from startup
  file (startup_stm32f0xx.s) before to branch to application main.
  To reconfigure the default setting of SystemInit() function, refer to
  system_stm32f0xx.c file
  */ 
  int i = 0;
  /* SPI configuration ------------------------------------------------------*/
  SPI_Config();
  
  /* SysTick configuration ---------------------------------------------------*/
  SysTickConfig();
 
  SL_GPIO_INIT();
	Display_Init();
  ADC_Config();
  /* Initialize LEDs mounted on STM320518-EVAL board */
  SL_LEDInit(LED1);
  SL_LEDInit(LED2);
  /* TIM Configuration */
  TIM3_Config();       //0301
 // Display_Init();
	/* configure the interuptions  */
	Interrupts_Config();
	/* configure the timers  */
	Timer_Config( );  
  
  GPIO_ResetBits(NFC_NSS_GPIO_PORT, NFC_NSS_PIN);
  GPIO_ResetBits(NFC_NSS_GPIO_PORT, LCD_DC);
  SPI_SendData8(SPIx, 0x21);
  SPI_SendData8(SPIx, 0xB1);
  SPI_SendData8(SPIx, 0x04);
  SPI_SendData8(SPIx, 0x13);
  SPI_SendData8(SPIx, 0x20);
  SPI_SendData8(SPIx, 0x0C);
  delay_ms(150);
  GPIO_SetBits(NFC_NSS_GPIO_PORT, LCD_DC);
  
	
	
  for( i = 0; i < 504; i++)
  {
    if((i>100) && (i<200))
		{
      SPI_SendData8(SPIx, 0xBB);
			
		}
    else
		{
      SPI_SendData8(SPIx, 0x00);
			
		}
  }
	
	
  Display();

	
   while (1)
  {
		
		//SL_GPIO_INIT();
		ADC_Config();
		while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
    
    
    ADCC1 =ADC_GetConversionValue(ADC1);
    ADCC1V = (ADCC1 *3300)/0xFFF;
		
		delay_ms(50);
    ADC_Config2();
		while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
		
		ADC1ConvertedValue = ADC_GetConversionValue(ADC1);
    ADC1ConvertedVoltage=( ADC1ConvertedValue*5000)/0xFFF;
		
		LCD_String("                                                                                   " ,84);
		LCD_XY_Set(0,0);                                                                             //84
		LCD_String("PM2.5" ,5);
		LCD_XY_Set(0,1);
		LCD_String("Value=  " ,8);
		LCD_XY_Print_DecNumb(9,1,ADCC1);
		LCD_XY_Set(0,2);
		LCD_String("Voltage=" ,8);
		LCD_XY_Print_DecNumb(9,2,ADCC1V);
		LCD_XY_Set(0,3);
		LCD_String("Air Quality" ,11);
		LCD_XY_Set(0,4);
		LCD_String("Value=  " ,8);
		LCD_XY_Print_DecNumb(9,4,ADC1ConvertedValue);
		LCD_XY_Set(0,5);
		LCD_String("Voltage=" ,8);
		LCD_XY_Print_DecNumb(9,5,ADC1ConvertedVoltage);
		
		delay_ms(5000);
		
    break;
	}
}
/*uint16_t ADCCatch(uint16_t ADCC1,uint16_t ADCC1V,uint16_t ADC1ConvertedValue,uint16_t ADC1ConvertedVoltage)
{
	while(1)
	{
		LCD_String("                                                                                   " ,84);
		LCD_XY_Set(0,0);                                                                               
		LCD_String("PM2.5" ,5);
		LCD_XY_Set(0,1);
		LCD_String("Value=  " ,8);
		LCD_XY_Print_DecNumb(9,1,ADCC1);
		LCD_XY_Set(0,2);
		LCD_String("Voltage=" ,8);
		LCD_XY_Print_DecNumb(9,2,ADCC1V);
		LCD_XY_Set(0,3);
		LCD_String("Air Quality" ,11);
		LCD_XY_Set(0,4);
		LCD_String("Value=  " ,8);
		LCD_XY_Print_DecNumb(9,4,ADC1ConvertedValue);
		LCD_XY_Set(0,5);
		LCD_String("Voltage=" ,8);
		LCD_XY_Print_DecNumb(9,5,ADC1ConvertedVoltage);
		delay_ms(1000);
	}
}
*/
/**
  * @brief  Configure the TIM IRQ Handler.
  * @param  None
  * @retval None
  */
static void ADC_Config(void)
{
  ADC_InitTypeDef     ADC_InitStructure;
  GPIO_InitTypeDef    GPIO_InitStructure;
  
  /* GPIOC Periph clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
  
  /* ADC1 Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  

  /* Configure ADC Channel11 as analog input */
#ifdef USE_STM320518_EVAL
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ;
	
#else
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ;
#endif /* USE_STM320518_EVAL */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  /* ADCs DeInit */  
  ADC_DeInit(ADC1);
  

  /* Initialize ADC structure */
  ADC_StructInit(&ADC_InitStructure);
  
  /* Configure the ADC1 in continuous mode with a resolution equal to 12 bits  */
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; 
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
  ADC_Init(ADC1, &ADC_InitStructure); 
  
  /* Convert the ADC1 Channel 11 with 239.5 Cycles as sampling time */ 
#ifdef USE_STM320518_EVAL
  ADC_ChannelConfig(ADC1, ADC_Channel_11 , ADC_SampleTime_239_5Cycles);
#else
  ADC_ChannelConfig(ADC1, ADC_Channel_10 , ADC_SampleTime_239_5Cycles);
#endif /* USE_STM320518_EVAL */

  /* ADC Calibration */
  ADC_GetCalibrationFactor(ADC1);
  
  /* Enable the ADC peripheral */
  ADC_Cmd(ADC1, ENABLE);     
  
  /* Wait the ADRDY flag */
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY)); 
  
  /* ADC1 regular Software Start Conv */ 
  ADC_StartOfConversion(ADC1);
  
}
static void ADC_Config2(void)
{
  ADC_InitTypeDef     ADC_InitStructure;
  GPIO_InitTypeDef    GPIO_InitStructure;
  
  /* GPIOC Periph clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  
  /* ADC1 Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  

  /* Configure ADC Channel11 as analog input */
#ifdef USE_STM320518_EVAL
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ;
	
#else
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ;
#endif /* USE_STM320518_EVAL */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  /* ADCs DeInit */  
  ADC_DeInit(ADC1);
  

  /* Initialize ADC structure */
  ADC_StructInit(&ADC_InitStructure);
  
  /* Configure the ADC1 in continuous mode with a resolution equal to 12 bits  */
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; 
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
  ADC_Init(ADC1, &ADC_InitStructure); 
  
  /* Convert the ADC1 Channel 11 with 239.5 Cycles as sampling time */ 
#ifdef USE_STM320518_EVAL
  ADC_ChannelConfig(ADC1, ADC_Channel_13 , ADC_SampleTime_239_5Cycles);
#else
  ADC_ChannelConfig(ADC1, ADC_Channel_12 , ADC_SampleTime_239_5Cycles);
#endif /* USE_STM320518_EVAL */

  /* ADC Calibration */
  ADC_GetCalibrationFactor(ADC1);
  
  /* Enable the ADC peripheral */
  ADC_Cmd(ADC1, ENABLE);     
  
  /* Wait the ADRDY flag */
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY)); 
  
  /* ADC1 regular Software Start Conv */ 
  ADC_StartOfConversion(ADC1);
  
}

void Display(void)
{
  uint32_t v=0,mv=0;
  uint8_t text[50];

  v=(ADC1ConvertedVoltage)/1000;
  mv = (ADC1ConvertedVoltage%1000)/100;
  
  /* Set the LCD Back Color and Text Color*/
  LCD_SetBackColor(White);
  LCD_SetTextColor(Blue);
  /* Display */
  LCD_DisplayStringLine(LINE(3), MESSAGE3);
  LCD_DisplayStringLine(LINE(4), MESSAGE4);
  /* Display voltage value */
  LCD_DisplayStringLine(LINE(6),text);
}

void Display_Init(void)
{
  /* Initialize the LCD */
#ifdef USE_STM320518_EVAL
    STM320518_LCD_Init();

#endif /* USE_STM320518_EVAL */
  
  /* Clear the LCD */ 
  LCD_Clear(White);

  /* Set the LCD Text size */
  LCD_SetFont(&Font8x12);

  /* Set the LCD Back Color and Text Color*/
  LCD_SetBackColor(Blue);
  LCD_SetTextColor(White);

  /* Display */
 // LCD_DisplayStringLine(LINE(0x13), "  ADC conversion example (Basic example)");
  
  /* Set the LCD Text size */
  LCD_SetFont(&Font16x24);

  LCD_DisplayStringLine(LINE(0), MESSAGE1);
  LCD_DisplayStringLine(LINE(1), MESSAGE2);
 
  /* Set the LCD Back Color and Text Color*/
  LCD_SetBackColor(White);
  LCD_SetTextColor(Blue);
	
	//LCD_String("meoww",5);
   
}

void TIM3_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* Enable the TIM3 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* -----------------------------------------------------------------------
    TIM3 Configuration: Output Compare Timing Mode:
    
    In this example TIM3 input clock (TIM3CLK) is set to APB1 clock (PCLK1),  
      => TIM3CLK = PCLK1 = SystemCoreClock = 48 MHz
          
    To get TIM3 counter clock at 6 MHz, the prescaler is computed as follows:
       Prescaler = (TIM3CLK / TIM3 counter clock) - 1
       Prescaler = (PCLK1 /6 MHz) - 1
                                                  
    CC3 update rate = TIM3 counter clock / CCR3_Val = 439.4 Hz
    ==> Toggling frequency = 219.7 Hz
    
    CC4 update rate = TIM3 counter clock / CCR4_Val = 878.9 Hz
    ==> Toggling frequency = 439.4 Hz

    Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f2xx.c file.
     Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
     function to update SystemCoreClock variable value. Otherwise, any configuration
     based on this variable will be incorrect.    
  ----------------------------------------------------------------------- */   

  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) (SystemCoreClock  / 1000000) - 1;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 65535;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* Prescaler configuration */
  TIM_PrescalerConfig(TIM3, PrescalerValue, TIM_PSCReloadMode_Immediate);

  /* Output Compare Timing Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  /* Output Compare Timing Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR3_Val;

  TIM_OC3Init(TIM3, &TIM_OCInitStructure);

  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);

  /* Output Compare Timing Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR4_Val;

  TIM_OC4Init(TIM3, &TIM_OCInitStructure);

  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Disable);
   
  /* TIM Interrupts enable */
  //TIM_ITConfig(TIM3, TIM_IT_CC3 | TIM_IT_CC4, ENABLE);
  TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
  
}



/**
  * @brief  Returns NbrOfDataToTransfer value.
  * @param  None
  * @retval Tx Buffer Size (NbrOfDataToTransfer1).
  */
uint8_t GetVar_NbrOfData(void)
{
  return DATA_SIZE;
}

/**
* @brief  Basic management of the timeout situation.
* @param  None.
* @retval None.
*/
static void TimeOut_UserCallback(void)
{
  /* User can add his own implementation to manage TimeOut Communication failure */
  /* Block communication and all processes */
  while (1)
  {  
  }
}

/**
  * @brief  Configures the SPI Peripheral.
  * @param  None
  * @retval None
  */



static void SPI_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Enable the SPI periph */
  RCC_APB2PeriphClockCmd(SPIx_CLK, ENABLE);
  
  /* Enable SCK, MOSI, MISO and NSS GPIO clocks */
  RCC_AHBPeriphClockCmd(SPIx_SCK_GPIO_CLK | SPIx_MISO_GPIO_CLK | SPIx_MOSI_GPIO_CLK, ENABLE);
  
  GPIO_PinAFConfig(SPIx_SCK_GPIO_PORT, SPIx_SCK_SOURCE, SPIx_SCK_AF);
  GPIO_PinAFConfig(SPIx_MOSI_GPIO_PORT, SPIx_MOSI_SOURCE, SPIx_MOSI_AF);
  GPIO_PinAFConfig(SPIx_MISO_GPIO_PORT, SPIx_MISO_SOURCE, SPIx_MISO_AF);
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  /* SPI SCK pin configuration */
  GPIO_InitStructure.GPIO_Pin = SPIx_SCK_PIN;
  GPIO_Init(SPIx_SCK_GPIO_PORT, &GPIO_InitStructure);

  /* SPI  MOSI pin configuration */
  GPIO_InitStructure.GPIO_Pin =  SPIx_MOSI_PIN;
  GPIO_Init(SPIx_MOSI_GPIO_PORT, &GPIO_InitStructure);

  /* SPI MISO pin configuration */
  GPIO_InitStructure.GPIO_Pin = SPIx_MISO_PIN;
  GPIO_Init(SPIx_MISO_GPIO_PORT, &GPIO_InitStructure);
  
  /* Configure I/O for Chip select */
	GPIO_InitStructure.GPIO_Pin = NFC_NSS_PIN | LCD_DC | LCD_BL;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 
	GPIO_Init(NFC_NSS_GPIO_PORT, &GPIO_InitStructure);
	
	/* SPI_NSS  = High Level  */
	GPIO_SetBits(NFC_NSS_GPIO_PORT, NFC_NSS_PIN);
  GPIO_SetBits(NFC_NSS_GPIO_PORT, LCD_DC);
  GPIO_SetBits(NFC_NSS_GPIO_PORT3, LCD_BL);

  
  /* SPI configuration -------------------------------------------------------*/
  SPI_I2S_DeInit(SPIx);
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_DataSize = SPI_DATASIZE;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
 
	SPI_Init(SPIx, &SPI_InitStructure);

  /* Initialize the FIFO threshold */
  SPI_RxFIFOThresholdConfig(SPIx, SPI_RxFIFOThreshold_QF);
  
  /* Enable the Rx buffer not empty interrupt */
//  SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_RXNE, ENABLE);
  /* Enable the SPI Error interrupt */
//  SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_ERR, ENABLE);
  /* Data transfer is performed in the SPI interrupt routine */
 	/* Enable SPI */
	SPI_Cmd(SPIx, ENABLE); 
  
  /* Configure the SPI interrupt priority */
//  NVIC_InitStructure.NVIC_IRQChannel = SPIx_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Configure a SysTick Base time to 10 ms.
  * @param  None
  * @retval None
  */
static void SysTickConfig(void)
{
  /* Setup SysTick Timer for 10ms interrupts  */
  if (SysTick_Config(SystemCoreClock / 100))
  {
    /* Capture error */
    while (1);
  }

  /* Configure the SysTick handler priority */
  NVIC_SetPriority(SysTick_IRQn, 0x0);
}

/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval PASSED: pBuffer1 identical to pBuffer2
  *         FAILED: pBuffer1 differs from pBuffer2
  */
static TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength, uint8_t DataMask)
{
  while (BufferLength--)
  {
    if (((*pBuffer1) & DataMask) != *pBuffer2)
    {
      return FAILED;
    }
    pBuffer1++;
    pBuffer2++;
  }

  return PASSED;
}

/**
  * @brief  Fills buffer.
  * @param  pBuffer: pointer on the Buffer to fill
  * @param  BufferLength: size of the buffer to fill
  * @retval None
  */
static void Fill_Buffer(uint8_t *pBuffer, uint16_t BufferLength)
{
  uint16_t index = 0;

  /* Put in global buffer same values */
  for (index = 0; index < BufferLength; index++ )
  {
    pBuffer[index] = 0x00;
  }
}



#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
