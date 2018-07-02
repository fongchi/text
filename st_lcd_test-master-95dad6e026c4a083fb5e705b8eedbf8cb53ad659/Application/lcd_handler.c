 /**
  ******************************************************************************
  * @file    lcd_handler.c
  * @author  HY R&D Team
  * @version V1.0
  * @date    08/20/2016
  * @brief   This file provides set of firmware to drive LCD 
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, HOYEN TECH SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2016 HOYEN TECH Co., Ltd </center></h2>
*/
/*#include "qm_common.h"
	#include "qm_gpio.h"
	#include "qm_spi.h"
	#include "qm_pinmux.h"
	#include "qm_interrupt.h"
	#include "qm_scss.h"
	#include "intel_iot_conf.h"*/ 
#include "hwconfig.h"
#include "string.h"
#include "ASCII.h"
#include "lcd_handler.h"


/* Local Defines */
#define SPI_CLOCK_DIV (256)
#define LCD_CMD    (0)
#define LCD_DATA   (1)
#define LCD_X     (84)
#define LCD_MAX_X (14)
#define LCD_Y     (48)
#define LCD_MAX_Y (6)
#define BUFFERSIZE (LCD_X*LCD_Y)
uint16_t i=0;
uint16_t j=0;
/* Private values */
//qm_spi_config_t cfg;
//LCD_transfer_t polled_xfer_desc;

/* Private functions */
void lcd_private_write(uint8_t DC, uint8_t *data, uint16_t len);


/*void LcdCharacter2(int meow)
{
  uint8_t str_buf[10];
  int index = 0;
  str_buf[index] = 0x00;
	
	for ( index = 0; index < meow; index++)
	{
		str_buf[index + 1] =  ASCII[j][index];
	}
	
  str_buf[index] = 0x00;
  lcd_private_write(LCD_DATA, str_buf, 6);
}
*/
/**
 * @brief  LcdCLear, Clear the LCD
 * @param  none
 * @retval none  
*/
void LCD_Clear_ASCII(void)
{
	uint8_t set_xy[] = {0x80, 0x40};
  int index = 0;
  uint8_t clean_buf[BUFFERSIZE / 8]; 
  for (index = 0; index < BUFFERSIZE / 8; index++)
  {
    clean_buf[index] = 0;
  }
  lcd_private_write(LCD_DATA, clean_buf, index);
 
  lcd_private_write(LCD_CMD, set_xy, 2);
}

/**
 * @brief  lcd_private_write
 * @param  uint8 DC: Data mode, uint8_t *data: Write data, uint16_t len: Write Data Length
 * @retval none  
*/
/*
void lcd_private_write(uint8_t DC, uint8_t *data, uint16_t len)
{
  polled_xfer_desc.tx = data;
  polled_xfer_desc.tx_len = len;
  if (DC) 
    qm_gpio_set_pin(QM_GPIO_0, PIN_DC);
  else 
    qm_gpio_clear_pin(QM_GPIO_0, PIN_DC);

  qm_spi_transfer(QM_SPI_MST_0, &polled_xfer_desc);
}
*/


void lcd_private_write(uint8_t DC, uint8_t data[], uint16_t len)
{
//  SPI_TypeDef.DR = data;
//  SPI_TypeDef.TXCRCR = len;
	int j;
	uint8_t databuf[100];
	for(j=0;j<50;j++)
	  databuf[j]=data[j];
	
  if (DC) 
    GPIO_SetBits(NFC_NSS_GPIO_PORT, LCD_DC);
  else 
    GPIO_ResetBits(NFC_NSS_GPIO_PORT, LCD_DC);
	for(i=0;i<len;i++)
	{
  SPI_SendData8(SPI1,databuf[i]);
	}
}

/**
 * @brief  LcdCharacter, Show a charactor on the LCD
 * @param  char 
 * @retval none  
*/
void LcdCharacter(char character)
{
  uint8_t str_buf[10];
  int index = 0;
  str_buf[index] = 0x00;
  for ( index = 0; index < 5; index++)
  {
    str_buf[index + 1] =  ASCII[character - 0x20][index];
  }
  str_buf[index+1] = 0x00;
  lcd_private_write(LCD_DATA, str_buf, 7);
}

/**
 * @brief  LCD_String, Write string on the LCD
 * @param  char *characters: Write string, int len: Write string Length
 * @retval none  
*/
void LCD_String(char *characters, int len)
{
  int idx = 0;
  while (*characters)
  {
    LcdCharacter(characters[idx]);
    idx++;
    if (idx >= len)
      break;   
  }
}


/**
 * @brief  LCD_XY_Set, Set cursor to (X,Y) 
 * @param  uint8_t x, uint8_t y
 * @retval none  
*/
void LCD_XY_Set(uint8_t x, uint8_t y)
{
  uint8_t set_xy[2];
  set_xy[0] = 0x80 | (x * 6);
  set_xy[1] = 0x40 | y;
  lcd_private_write(LCD_CMD, set_xy, 2);
}


/**
 * @brief  LCD_XY_Range_Clear, Set cursor to (X0,Y0) then clear LCD to (X1, Y1)
 * @param  uint8_t x0 : Set cursor to x0
 * @param  uint8_t y0 : Set cursor to y0
 * @param  uint8_t x1 : clear to X1 
 * @param  uint8_t y1 : clear to Y1
 *
 * @retval 0: success, -1 : fail  
*/
int LCD_XY_Range_Clear(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
  int idx = 0;
  int x_clear_len = 0; 
  int y_clear_len = 0;
  uint8_t clean_byte[6];
 
  memset(clean_byte, 0, 6);

  if (x1 - x0 > LCD_MAX_X || y1 - y0 > LCD_MAX_Y)
    return -1;

  x_clear_len = x1 - x0;
  y_clear_len = y1 - y0;
  LCD_XY_Set(x0, y0);
  for (idx = 0; idx < x_clear_len; idx++)
  {
    LCD_XY_Set(x0 + idx, y0);
    lcd_private_write(LCD_DATA, clean_byte, 6);
  }
  for (idx = 0; idx < y_clear_len; idx++)
  {
    LCD_XY_Set(x0, y0 + idx);
    lcd_private_write(LCD_DATA, clean_byte, 6);
  }
  return 0;
}

int LCD_XY_Print(uint8_t x, uint8_t y, uint8_t *strb, uint16_t len)
{
  if (x > LCD_MAX_X)
    return -1;
  if (y > LCD_MAX_Y)
    return -1;

  LCD_XY_Set(x, y);
  LCD_String((char *)strb, len);
  return 0;
}

void LCD_XY_Print_DecNumb(uint8_t x, uint8_t y, int dec)
{
  int i = 0;
  int numbs = 0;
  int t = 0, j = 0;
  uint8_t StrBuf[4];
  if (dec >= 1000 && dec < 10000)
  {
    numbs = 4;
    t = 1000;
  }
  else if (dec >= 100 && dec < 1000)
  {
    numbs = 3;
    t = 100;
  }
  else if (dec >= 10 && dec < 100)
  {
    numbs = 2;
    t = 10;
  }
  else if (dec < 10 && dec >= 0)
  {
    numbs = 1;
    t = 1;
  }
  else
    return;
  for (i = 0; i < numbs; i++)
  {
    StrBuf[i] = ((dec - j) / t) + 0x30;
    j = (dec / t) * t;
    t /= 10;
  }
  LCD_XY_Set(x, y);
  StrBuf[numbs + 1] = 0;
  LCD_String((char *)StrBuf, numbs);
}

void LCD_XY_Print_SymIdx(uint8_t x, uint8_t y, int char_sym)
{
  uint8_t str_buf[10];
  int index = 0;
  str_buf[index] = 0x00;
  for ( index = 0; index < 5; index++)
    str_buf[index + 1] =  ASCII[char_sym][index];
  str_buf[index + 1] = 0x00;
  LCD_XY_Set(x, y);
  lcd_private_write(LCD_DATA, str_buf, 6);
}


/**
 * @brief  LCD_Init, Configure IO as SPI interface 
 * @param  none
 * @retval none  
*/


