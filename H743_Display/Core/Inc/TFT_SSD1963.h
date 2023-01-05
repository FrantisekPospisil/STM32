/*
 * TFT_SSD1963.h
 *
 *  Created on: 22. 10. 2022
 *      Author: František Pospíšil
 */

#ifndef INC_TFT_SSD1963_H_
#define INC_TFT_SSD1963_H_

#include "main.h"

#define		ColBlack		0x0000		// cerna
#define		ColNavy			0x000F		// namorni modra
#define		ColBlue			0x001F		// modra
#define		ColDarkBlue		0x01CF		// tmave modra
#define		ColDarkGreen	0x03E0		// tmave zelena
#define		ColDarkCyan		0x03EF		// tmave azurova
#define		ColDarkRed		0x7800		// tmave cervena
#define		ColGreen		0x07E0		// zelena
#define		ColCyan			0x07FF		// azurova
#define		ColGrayBlue		0x5458		// seda modra
#define		ColMaroon		0x7800		// hnedocervena
#define		ColPurple		0x780F		// nachova
#define		ColOlive		0x7BEF		// olivova
#define		ColLightBlue	0x7D7C		// svetle modra
#define		ColPorpo		0x801F		//
#define		ColDarkGrey		0x8410		// tmave seda
#define		ColGrey			0x8430		// seda
#define		ColLGreyGreen	0xA651		// svetle sedozelena
#define		ColLightGrey	0xC618		// svetle seda
#define		ColRed			0xF800		// cervena
#define		ColMagneta		0xF81F		// purpurova
#define		ColOrange		0xFC08		// oranz
#define		ColYellow		0xFFE0		// zluta
#define		ColWhite		0xFFFF		// bila

#define TFT_DATA (uint32_t*)0xC0040000
#define TFT_COMMAND (uint32_t*)0xC0000000

/**
  * @brief sends an data to the controller display SSD1963
  * @param uint16_t data
  * @retval none
  */
void TFT_SendData(uint16_t val);

/**
  * @brief sends an instruction to the controller display SSD1963
  * @param uint16_t instruction
  * @retval none
  */
void TFT_SendCommand(uint16_t val);

/**
  * @brief sends an instruction and data
  * @param
  * @retval none
  */
void TFT_WriteRegister(uint16_t command, uint16_t val);

/**
  * @brief initializes the display with the controller SSD1963
  * @param
  * @retval none
  */
void TFT_Init(void);

/**
  * @brief sets the workspace
  * @param StartX, EndX, StartY, EndY
  * @retval none
  */
void TFT_WindowSet(uint16_t StartX, uint16_t EndX, uint16_t StartY, uint16_t EndY);

/**
  * @brief sets the color on the entire screen
  * @param color
  * @retval none
  */
void TFT_WindowFull(uint16_t val);

/**
  * @brief sets the color on the selected desktop
  * @param StartX, EndX, StartY, EndY, Color
  * @retval none
  */
void TFT_ColorBox(uint16_t StartX, uint16_t EndX, uint16_t StartY, uint16_t EndY, uint16_t color);

/**
  * @brief draws a big point
  * @param X, Y, Color
  * @retval none
  */
void TFT_DrawBigPoint(uint16_t x, uint16_t y, uint16_t color);

/**
  * @brief draws a small point
  * @param X, Y, Color
  * @retval none
  */
void TFT_DrawPoint(uint16_t x, uint16_t y, uint16_t color);

/**
  * @brief draws a line
  * @param X1, Y1, X2, Y2, Color
  * @retval none
  */
void TFT_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);

/**
  * @brief draws a rectangle
  * @param X1, Y1, X2, Y2, Color
  * @retval none
  */
void TFT_DrawBox(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);

/**
  * @brief draws a circle
  * @param X, Y, Radius, Color
  * @retval none
  */
void TFT_DrawCircle(uint16_t x, uint16_t y, uint16_t radius, uint16_t color);

/**
  * @brief print char
  * @param Font, ColumnFlex, ChAscii, ColFont, ColBack, X, Y
  * @retval consumed columns
  */
uint8_t TFT_PrintChar( uint8_t Font, uint8_t ColumnFlex, uint8_t ChAscii, uint16_t ColFront, uint16_t ColBack, uint16_t x, uint16_t y );

/**
  * @brief print string
  * @param Font, String, ColFont, ColBack, X, Y
  * @retval
  */
void TFT_PrintString( uint8_t Font, const char *String, uint16_t ColFront, uint16_t ColBack, uint16_t x, uint16_t y );
uint32_t TFT_Mypow( uint8_t m, uint8_t n );

/**
  * @brief print number
  * @param Font, Number, Length, Decimal, ColFont, ColBack, X, Y
  * @retval
  */
void TFT_PrintNumber( uint8_t Font, uint32_t Num, uint8_t NumLen, uint8_t Decimal, uint16_t ColFront, uint16_t ColBack, uint16_t x, uint16_t y );

/**
  * @brief print number for clock DS3231
  * @param Font, Number, ColFont, ColBack, X, Y
  * @retval
  */
void TFT_PrintNumberClock( uint8_t Font, uint8_t Num, uint16_t ColFront, uint16_t ColBack, uint16_t x, uint16_t y );

/**
  * @brief print hexadecimal number
  * @param Font, Number, ColFont, ColBack, X, Y
  * @retval
  */
void TFT_PrintHexNumber( uint8_t Font, uint32_t Num, uint16_t ColFront, uint16_t ColBack, uint16_t x, uint16_t y );

#endif /* INC_TFT_SSD1963_H_ */
