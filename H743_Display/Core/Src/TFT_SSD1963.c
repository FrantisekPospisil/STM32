/*
 * TFT_SSD1963.c
 *
 *  Created on: 22. 10. 2022
 *      Author: František Pospíšil
 */

#include "TFT_SSD1963.h"

#include "TFT_Tahoma11x13.h"
#include "TFT_Tahoma15x16.h"
#include "TFT_Tahoma30x32.h"

/**
  * @brief sends an data to the controller display SSD1963
  * @param uint16_t data
  * @retval none
  */
void TFT_SendData(uint16_t val) {
//	HAL_SRAM_Write_16b(&hsram1, TFT_DATA, &val, 1);
	*TFT_DATA = val;
}

/**
  * @brief sends an instruction to the controller display SSD1963
  * @param uint16_t instruction
  * @retval none
  */
void TFT_SendCommand(uint16_t val) {
//	HAL_SRAM_Write_16b(&hsram1, TFT_COMMAND, &val, 1);
	*TFT_COMMAND = val;
}

/**
  * @brief sends an instruction and data
  * @param
  * @retval none
  */
void TFT_WriteRegister(uint16_t command, uint16_t val) {
	TFT_SendCommand( command );
	TFT_SendData( val );
}

/**
  * @brief initializes the display with the controller SSD1963
  * @param
  * @retval none
  */
void TFT_Init(void) {
	HAL_GPIO_WritePin(FMC_RES_GPIO_Port, FMC_RES_Pin, GPIO_PIN_RESET);
	HAL_Delay( 100 );
	HAL_GPIO_WritePin(FMC_RES_GPIO_Port, FMC_RES_Pin, GPIO_PIN_SET);
	HAL_Delay( 10 );
	TFT_SendCommand( 0x0001 );
	HAL_Delay( 50 );
	TFT_SendCommand( 0x00E2 );
	TFT_SendData( 0x001E );					// 0x0023
	TFT_SendData( 0x0002 );					// 0x0002
	TFT_SendData( 0x0054 );					// originally 0x0004
	TFT_WriteRegister( 0x00E0, 0x0001 );	// PLL enable
	HAL_Delay( 20 );
	TFT_WriteRegister( 0x00E0, 0x0003 );
	HAL_Delay( 20 );
	TFT_SendCommand( 0x0001 );				// software reset
	HAL_Delay( 100 );

	TFT_SendCommand( 0x00E6 );   			//SET PCLK freq=9.5MHz  ; pixel clock frequency
	TFT_SendData( 0x0003 );					// 0x0004
	TFT_SendData( 0x00FF );					// 0x0093
	TFT_SendData( 0x00FF );					// 0x00E0

	TFT_SendCommand( 0x00B0 );				// set LCD specification
	TFT_SendData( 0x0024 );					// must by 0x0024 for correct color order
	TFT_SendData( 0x0000 );					//LCD panel mode
	TFT_SendData( 0x0003 );					//SET horizontal size=800-1 HightByte
	TFT_SendData( 0x001F );				    //SET horizontal size=800-1 LowByte
	TFT_SendData( 0x0001 );					//SET vertical size=480-1 HightByte
	TFT_SendData( 0x00DF );					//SET vertical size=480-1 LowByte
	TFT_SendData( 0x0000 );					//SET even/odd line RGB seq.=RGB

	TFT_SendCommand( 0x00B4 );				//SET Horizontal Period (8 parameters)
	TFT_SendData( 0x0003 );					//1000 HT
	TFT_SendData( 0x00E8 );
	TFT_SendData( 0x0000 );					//51 HPS
	TFT_SendData( 0x0033 );
	TFT_SendData( 0x0008 );					//8 HPW
	TFT_SendData( 0x0000 );
	TFT_SendData( 0x0003 );					//3 LPS
	TFT_SendData( 0x0000 );

	TFT_SendCommand( 0x00B6 );		 		//SET Vertical Period (7 parameters),
	TFT_SendData( 0x0002 );					//530 VT
	TFT_SendData( 0x0012 );
	TFT_SendData( 0x0000 );					//24 VPS
	TFT_SendData( 0x0018 );
	TFT_SendData( 0x0003 );					//3 VPW
	TFT_SendData( 0x0000 );					//23 FPS
	TFT_SendData( 0x0017 );

	TFT_SendCommand( 0x002B );				//Set column address
	TFT_SendData( 0x0000 );
	TFT_SendData( 0x0000 );
	TFT_SendData( 0x0001 );
	TFT_SendData( 0x00DF );

	TFT_SendCommand( 0x002A );
	TFT_SendData( 0x0000 );
	TFT_SendData( 0x0000 );
	TFT_SendData( 0x0003 );
	TFT_SendData( 0x001F );


	TFT_WriteRegister( 0x00BA, 0x000F );	//GPIO[3:0] out 1, nebo 0x0000
	TFT_SendCommand( 0x00B8 );				// reset GPIO0, nebo 0x00B8
	TFT_SendData( 0x0000 );					//GPIO3=input, GPIO[2:0]=output
	TFT_SendData( 0x0001 );					//GPIO0 normal

	TFT_WriteRegister( 0x0036, 0x0000 );	//rotation!!!!!!!!! rotates the display, originally 31
	TFT_WriteRegister( 0x003A, 0x0050 );	// pixel data interface
	TFT_WriteRegister( 0x00F0, 0x0003 );	// 565 format barvy
	HAL_Delay( 5 );
	TFT_WriteRegister( 0x0026, 0x0008 );	// Display ON
	TFT_SendCommand( 0x0029 );				// Display ON
	TFT_SendCommand( 0x00BE );				// set PWM for B/L
	TFT_SendData( 0x0001 );					// frequency PWM
	TFT_SendData( 0x00FF );					// value PWM
	TFT_SendData( 0x0001 );					// PWM enable
	TFT_SendData( 0x00F0 );
	TFT_SendData( 0x0000 );
	TFT_SendData( 0x0000 );
	TFT_WriteRegister( 0x00D0, 0x000D );
	TFT_WriteRegister( 0x000B, 0x0064 );
	HAL_GPIO_WritePin(TFT_LED_GPIO_Port, TFT_LED_Pin, GPIO_PIN_SET);
}

/**
  * @brief sets the workspace
  * @param StartX, EndX, StartY, EndY
  * @retval none
  */
void TFT_WindowSet(uint16_t StartX, uint16_t EndX, uint16_t StartY, uint16_t EndY) {
	*TFT_COMMAND = 0x002A;			// SET page address
	*TFT_DATA = StartX >> 8;
	*TFT_DATA = StartX;
	*TFT_DATA = EndX >> 8;
	*TFT_DATA = EndX;

	*TFT_COMMAND = 0x002B;			// SET column address
	*TFT_DATA = StartY >> 8;
	*TFT_DATA = StartY;
	*TFT_DATA = EndY >> 8;
	*TFT_DATA = EndY;
}

/**
  * @brief sets the color on the entire screen
  * @param color
  * @retval none
  */
void TFT_WindowFull(uint16_t val) {
	uint16_t x, y;
	TFT_WindowSet( 0x0000, 0x031F, 0x0000, 0x01DF );
	TFT_SendCommand( 0x002C );
	for( x = 0; x < 480; x++ )
	{
		for( y = 0; y < 800; y++ )
		{
			*TFT_DATA = val;	// SET color
		}
	}
}

/**
  * @brief sets the color on the selected desktop
  * @param StartX, EndX, StartY, EndY, Color
  * @retval none
  */
void TFT_ColorBox(uint16_t StartX, uint16_t EndX, uint16_t StartY, uint16_t EndY, uint16_t color) {
	uint16_t x, y;
	TFT_WindowSet( StartX, EndX, StartY, EndY );
	TFT_SendCommand( 0x002C );
	for( x = StartX; x <= EndX; x++ )
	{
		for ( y = StartY; y <= EndY; y++ )
		{
			*TFT_DATA = color;	// SET color
		}
	}
}

/**
  * @brief draws a big point
  * @param X, Y, Color
  * @retval none
  */
void TFT_DrawBigPoint(uint16_t x, uint16_t y, uint16_t color) {
	TFT_ColorBox( x - 1, x + 1, y - 1, y + 1, color );
}

/**
  * @brief draws a small point
  * @param X, Y, Color
  * @retval none
  */
void TFT_DrawPoint(uint16_t x, uint16_t y, uint16_t color) {
	TFT_WindowSet( x, x, y, y );
	TFT_SendCommand( 0x002C );
	TFT_SendData( color );
}

/**
  * @brief draws a line
  * @param X1, Y1, X2, Y2, Color
  * @retval none
  */
void TFT_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
	int16_t dx, dy, stepx, stepy, fraction;

	dy = y2 - y1;
	dx = x2 - x1;
	if (dy < 0)
	{
		dy = -dy;
		stepy = -1;
	}
	else stepy = 1;
	if (dx < 0)
	{
		dx = -dx;
		stepx = -1;
	}
	else stepx = 1;
	dx <<= 1;
	dy <<= 1;
	TFT_DrawPoint( x1, y1, color );
	if (dx > dy)
	{
		fraction = dy - (dx >> 1);
		while( x1 != x2 )
		{
			if (fraction >= 0)
			{
				y1			+= stepy;
				fraction	-= dx;
			}
			x1			+= stepx;
			fraction	+= dy;
			TFT_DrawPoint( x1, y1, color );
		}
	}
	else
	{
		fraction = dx - (dy >> 1);
		while( y1 != y2 )
		{
			if (fraction >= 0)
			{
				x1			+= stepx;
				fraction	-= dy;
			}
			y1			+= stepy;
			fraction	+= dx;
			TFT_DrawPoint( x1, y1, color );
		}
	}
}

/**
  * @brief draws a rectangle
  * @param X1, Y1, X2, Y2, Color
  * @retval none
  */
void TFT_DrawBox(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
	TFT_DrawLine( x1, y1, x2, y1, color);	// up
	TFT_DrawLine( x1, y2, x2, y2, color);	// down
	TFT_DrawLine( x2, y1, x2, y2, color);	// right
	TFT_DrawLine( x1, y1, x1, y2, color);	// left
}

/**
  * @brief draws a circle
  * @param X, Y, Radius, Color
  * @retval none
  */
void TFT_DrawCircle(uint16_t x, uint16_t y, uint16_t radius, uint16_t color) {
	int16_t xc = 0;
	int16_t yc, p;

	yc = radius;
	p = 3 - (radius << 1);
	while( xc <= yc)
	{
		TFT_DrawPoint( x + xc, y - yc, color );
		TFT_DrawPoint( x - xc, y - yc, color );
		TFT_DrawPoint( x + yc, y - xc, color );
		TFT_DrawPoint( x - yc, y - xc, color );
		TFT_DrawPoint( x + xc, y + yc, color );
		TFT_DrawPoint( x - xc, y + yc, color );
		TFT_DrawPoint( x + yc, y + xc, color );
		TFT_DrawPoint( x - yc, y + xc, color );
		if (p < 0)
		p += ( xc++ << 2 ) + 6;
		else
		p += (( xc++ - yc-- ) << 2 ) + 10;
	}
}

/**
  * @brief print char
  * @param Font, ColumnFlex, ChAscii, ColFont, ColBack, X, Y
  * @retval consumed columns
  */
uint8_t TFT_PrintChar( uint8_t Font, uint8_t ColumnFlex, uint8_t ChAscii, uint16_t ColFront, uint16_t ColBack, uint16_t x, uint16_t y ) {
	uint16_t PointerStart = 0;	// pointer at the beginning of a letter
	uint16_t PointerEnd = 0;	// pointer at the end of a letter
	uint8_t  Column = 0;		// the number of columns is variable
	uint8_t  ColumnFix = 0;		// the number of columns fixed
	uint8_t  CounterBits;		// counts transmitted bits
	uint8_t  CounterColumn;		// counts columns
	uint8_t  Bits = 0;			// transmitted bit
	uint8_t  Row = 0;			// number of lines
	uint8_t  i, u;

	ChAscii -= 0x20;			// subtracts 32, the character ' ' has a value 0
	switch( Font )
	{
		case 0:
		{
			Column = Tahoma11x13[( ChAscii * 4 ) + 8  ];					// the number of columns displayed
			ColumnFix = 11;
			Row    = Tahoma11x13[ ChAscii + 6 ];							// the number of rows
			Row = 13;
			PointerStart =  Tahoma11x13[( ChAscii * 4 ) + 10 ] * 256;		// the places the letter starts
			PointerStart += Tahoma11x13[( ChAscii * 4 ) + 9  ];
			ChAscii++;
			PointerEnd =  Tahoma11x13[( ChAscii * 4 ) + 10 ] * 256;			// where the letters ends
			PointerEnd += Tahoma11x13[( ChAscii * 4 ) + 9  ];
			break;
		}
		case 1:	// Tahoma 12
		{
			Column = Tahoma15x16[( ChAscii * 4 ) + 8  ];					// the number of columns displayed
			ColumnFix = 12;
			Row    = Tahoma15x16[ ChAscii + 6 ];							// the number of rows displayed
			Row = 16;
			PointerStart =  Tahoma15x16[( ChAscii * 4 ) + 10 ] * 256;
			PointerStart += Tahoma15x16[( ChAscii * 4 ) + 9  ];
			ChAscii++;
			PointerEnd =  Tahoma15x16[( ChAscii * 4 ) + 10 ] * 256;
			PointerEnd += Tahoma15x16[( ChAscii * 4 ) + 9  ];
			break;
		}
		case 2:	// Tahoma 24
		{
			Column = Tahoma30x32[( ChAscii * 4 ) + 8  ];					// the number of columns displayed
			ColumnFix = 20;
			Row    = Tahoma30x32[ ChAscii + 6 ];							// the number of rows displayed
			Row = 32;
			PointerStart =  Tahoma30x32[( ChAscii * 4 ) + 10 ] * 256;
			PointerStart += Tahoma30x32[( ChAscii * 4 ) + 9  ];
			ChAscii++;
			PointerEnd =  Tahoma30x32[( ChAscii * 4 ) + 10 ] * 256;
			PointerEnd += Tahoma30x32[( ChAscii * 4 ) + 9  ];
			break;
		}
	}

	PointerEnd++;
	if ( ColumnFlex == 1)
	{
		TFT_WindowSet( x, x + Column + 1, y, y + Row - 1 );					// sets a floating desktop on the display
		u = 1;																// number of additional columns per letter
	}
	else
	{
		TFT_WindowSet( x, x + ColumnFix, y, y + Row - 1 );					// sets a fixed desktop on the display
		u = ColumnFix - Column;												// number of additional columns per letter
	}
	TFT_SendCommand( 0x002C );												// will send data to the desktop

	CounterBits = 8;														// counter of the transmitted bits
	CounterColumn = 0;														// counter columns
	while( PointerEnd != PointerStart )										// end when all character data is sent
	{
		if( CounterColumn == 0 )
		{
			for ( i = 0; i < u; i++ )
			{
				TFT_SendData( ColBack );									// column before the letter
			}
		}
		if( CounterBits == 8 )												// at the eighth bit, it takes another byte
		{
			CounterBits = 0;												// resets the bit counter
			switch( Font )
			{
				case 0:
				{
					Bits = Tahoma11x13[ PointerStart ];						// takes a new byte from the correct character set
					break;
				}
				case 1:
				{
					Bits = Tahoma15x16[ PointerStart ];
					break;
				}
				case 2:
				{
					Bits = Tahoma30x32[ PointerStart ];
					break;
				}
			}
			PointerStart++;													// counter of bytes to send
		}
		if( Bits & 0x01 ) *TFT_DATA = ColFront;								// sends the color of the letter
		else TFT_SendData( ColBack );
		Bits = Bits >> 1;													// it moves to the next bit
		CounterColumn++;													// it moves to the next column
		CounterBits++;														// bit counter
		if( CounterColumn == Column )										// check if bit is last
		{
			CounterColumn = 0;												// counter column
			CounterBits = 8;												// new eight bits
			*TFT_DATA = ColBack;											// column after letter
		}
	}
	u++;
	Column += u;
	return Column;															// returns the number of columns consumed
}

/**
  * @brief print string
  * @param Font, String, ColFont, ColBack, X, Y
  * @retval
  */
void TFT_PrintString( uint8_t Font, const char *String, uint16_t ColFront, uint16_t ColBack, uint16_t x, uint16_t y ) {
	uint8_t Column1;

	while( *String )
	{
		Column1 = TFT_PrintChar( Font, 1, *String, ColFront, ColBack, x, y );
		x += Column1;
		String++;
	}
}

uint32_t TFT_Mypow( uint8_t m, uint8_t n ) {
	uint32_t result = 1;
	while( n-- ) result *= m;				// loop of multiplication
	return result;
}

/**
  * @brief print number
  * @param Font, Number, Length, Decimal, ColFont, ColBack, X, Y
  * @retval
  */
void TFT_PrintNumber( uint8_t Font, uint32_t Num, uint8_t NumLen, uint8_t Decimal, uint16_t ColFront, uint16_t ColBack, uint16_t x, uint16_t y ) {
	uint8_t t, temp;
	uint8_t enshow = 0;

	for(t = 0; t < NumLen; t++)
	{
		temp = ( Num / TFT_Mypow( 10, NumLen - t - 1 )) % 10;
		if (( enshow == 0 ) && ( t < ( NumLen - 1 )) && ( t < ( NumLen - Decimal - 1 )))
		{
			if ( temp == 0 )
			{
				x += TFT_PrintChar( Font, 0, ' ', ColFront, ColBack, x, y );
				continue;
			}
			else enshow = 1;
		}
		x += TFT_PrintChar( Font, 0, temp + '0', ColFront, ColBack, x, y );
		if(( t == ( NumLen - Decimal - 1 )) && ( t < ( NumLen - 1 )))
		{
			x += TFT_PrintChar( Font, 1, ',', ColFront, ColBack, x, y );
		}
	}
}

/**
  * @brief print number for clock DS3231
  * @param Font, Number, ColFont, ColBack, X, Y
  * @retval
  */
void TFT_PrintNumberClock( uint8_t Font, uint8_t Num, uint16_t ColFront, uint16_t ColBack, uint16_t x, uint16_t y ) {
	x += TFT_PrintChar(Font, 1, '0' + ( Num >> 4 ), ColFront, ColBack, x, y);
	TFT_PrintChar(Font, 1, '0' + ( Num & 0x0F ), ColFront, ColBack, x, y);
}

/**
  * @brief print hexadecimal number
  * @param Font, Number, ColFont, ColBack, X, Y
  * @retval
  */
void TFT_PrintHexNumber( uint8_t Font, uint32_t Num, uint16_t ColFront, uint16_t ColBack, uint16_t x, uint16_t y ) {
	uint8_t temp;

	x += TFT_PrintChar(Font, 0, '0', ColFront, ColBack, x, y);
	x += TFT_PrintChar(Font, 0, 'x', ColFront, ColBack, x, y);

	temp = (uint8_t) (0x0000000F & (Num >> 28));
	if( temp > 9 ) temp = temp + 39;
	x += TFT_PrintChar(Font, 0, temp + 0x30, ColFront, ColBack, x, y);
	temp = (uint8_t) (0x0000000F & (Num >> 24));
	if( temp > 9 ) temp = temp + 39;
	x += TFT_PrintChar(Font, 0, temp + 0x30, ColFront, ColBack, x, y);
	temp = (uint8_t) (0x0000000F & (Num >> 20));
	if( temp > 9 ) temp = temp + 39;
	x += TFT_PrintChar(Font, 0, temp + 0x30, ColFront, ColBack, x, y);
	temp = (uint8_t) (0x0000000F & (Num >> 16));
	if( temp > 9 ) temp = temp + 39;
	x += TFT_PrintChar(Font, 0, temp + 0x30, ColFront, ColBack, x, y);
	temp = (uint8_t) (0x0000000F & (Num >> 12));
	if( temp > 9 ) temp = temp + 39;
	x += TFT_PrintChar(Font, 0, temp + 0x30, ColFront, ColBack, x, y);
	temp = (uint8_t) (0x0000000F & (Num >>  8));
	if( temp > 9 ) temp = temp + 39;
	x += TFT_PrintChar(Font, 0, temp + 0x30, ColFront, ColBack, x, y);
	temp = (uint8_t) (0x0000000F & (Num >>  4));
	if( temp > 9 ) temp = temp + 39;
	x += TFT_PrintChar(Font, 0, temp + 0x30, ColFront, ColBack, x, y);
	temp = (uint8_t) (0x0000000F & Num);
	if( temp > 9 ) temp = temp + 39;
	x += TFT_PrintChar(Font, 0, temp + 0x30, ColFront, ColBack, x, y);
}
