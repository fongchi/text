 /**
  ******************************************************************************
  * @file    lcd_handler.h
  * @author  HY R&D Team
  * @version V1.0
  * @date    08/20/2016
  * @brief   This file provides set of LCD header files
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
#ifndef LCD_HANDLER_H_
#define LCD_HANDLER_H_

void LCD_Init(void);
void LCD_Clear_ASCII(void);
void LCD_XY_Set(uint8_t x, uint8_t y);
void LCD_String(char *characters, int len);
int LCD_XY_Print(uint8_t x, uint8_t y, uint8_t *strb, uint16_t len);
int LCD_XY_Range_Clear(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void LCD_XY_Print_SymIdx(uint8_t x, uint8_t y, int char_sym);
void LCD_XY_Print_DecNumb(uint8_t x, uint8_t y, int dec);

#endif /* LCD_HANDLER_H_ */
