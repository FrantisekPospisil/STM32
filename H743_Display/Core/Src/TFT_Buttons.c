/*
 * TFT_Buttons.c
 *
 *  Created on: 22. 10. 2022
 *      Author: František Pospíšil
 */


#include "main.h"
#include "TFT_Buttons.h"
#include "TFT_SSD1963.h"
#include "i2c.h"

/**
  * @brief will convert the number for DS3231
  * @param number
  * @retval number
  */
uint8_t ClockNumer( uint8_t val) {
uint8_t NumReturn = 0;
uint8_t Num;

	Num = (val / 10) << 4;
	NumReturn = Num | (val % 10 );
	return NumReturn;
}

/**
  * @brief checks the clock button and return 1
  * @param string, ColFront, X, Y
  * @retval key press
  */
uint8_t TFT_ButtonClockCheck( uint16_t x, uint16_t y ) {
uint8_t NumReturn;
	if(( TouchXresult > (x + 5)) && ( TouchXresult < (x + 175)) && ( TouchYresult > (y + 5)) && ( TouchYresult < (y + 85))) {
		NumReturn = 1;
		TouchXresult = 0;
		TouchYresult = 0;
		TouchTimer = 300;
		Beep(1000, 10);
	}
	else {
		NumReturn = 0;
	}
	return NumReturn;
}

/**
  * @brief draws clock from DS3231 in box
  * @param ReDraw, ColFront, ColBox, ColBack, X, Y
  * @retval none
  */
void TFT_ButtonClock( uint8_t ReDraw, uint16_t ColFront, uint16_t ColBox, uint16_t ColBack, uint16_t x, uint16_t y ) {
	if( ReDraw == 1 ) {
		TFT_ColorBox(x + 4, x + 179, y + 4, y + 89, ColBack);
		TFT_DrawBox(x + 3, y + 3, x + 180, y + 90, ColBox);
	}
	BufClock[0] = 0;
	HAL_I2C_Master_Transmit(&hi2c1, 0xD0, BufClock, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c1, 0xD0, BufClock, 6, HAL_MAX_DELAY);


	TFT_PrintNumberClock(2, BufClock[2], ColFront, ColBack, x + 15, y + 10 );	// hodiny
	TFT_PrintChar(2, 1, ':', ColFront, ColBack, x + 55, y + 10);
	TFT_PrintNumberClock(2, BufClock[1], ColFront, ColBack, x + 70, y + 10 );	// minuty
	TFT_PrintChar(2, 1, ':', ColFront, ColBack, x + 110, y + 10);
	TFT_PrintNumberClock(2, BufClock[0], ColFront, ColBack, x + 125, y + 10 );	// sekundy
	TFT_PrintNumberClock(2, BufClock[5], ColFront, ColBack, x + 40, y + 50 );	// mesice
	TFT_PrintChar(2, 1, '/', ColFront, ColBack, x + 80, y + 50);
	TFT_PrintNumberClock(2, BufClock[4], ColFront, ColBack, x + 95, y + 50 );	// dny
}

/**
  * @brief returns the number from the keypad
  * @param Max digits, ColFront, ColBox, ColBack, X, Y
  * @retval Number from the keypad
  */
uint16_t TFT_KeypadNumberCheck( uint8_t DigitsMax, uint16_t ColFront, uint16_t ColBox, uint16_t ColBack, uint16_t x, uint16_t y ) {
uint16_t KeyResult = 0;
uint8_t KeyNum;
uint8_t KeyCounter = 0;

	while( KeyCounter < DigitsMax ) {
		KeyNum = 12;
		if( TFT_ButtonNumberCheck( x, y + 60 ) == 1 ) KeyNum = 1;
		if( TFT_ButtonNumberCheck( x +  80, y + 60 ) == 1 ) KeyNum = 2;
		if( TFT_ButtonNumberCheck( x + 160, y + 60) == 1 ) KeyNum = 3;
		if( TFT_ButtonNumberCheck( x, y + 140 ) == 1 ) KeyNum = 4;
		if( TFT_ButtonNumberCheck( x + 80, y + 140 ) == 1 ) KeyNum = 5;
		if( TFT_ButtonNumberCheck( x + 160, y + 140 ) == 1 ) KeyNum = 6;
		if( TFT_ButtonNumberCheck( x, y + 220 ) == 1 ) KeyNum = 7;
		if( TFT_ButtonNumberCheck( x + 80, y + 220 ) == 1 ) KeyNum = 8;
		if( TFT_ButtonNumberCheck( x + 160, y + 220 ) == 1 ) KeyNum = 9;
		if(( TFT_ButtonStringCheck( x + 80, y + 300 ) == 1 ) && ( KeyNum > 0 )) break;
		if( TFT_ButtonNumberCheck( x, y + 300 ) == 1 ) KeyNum = 0;
		if( KeyNum < 12 ) {
			KeyResult = ( KeyResult * 10 ) + KeyNum;
			KeyCounter++;
			TFT_PrintNumber( 2, KeyResult, DigitsMax, 0, ColFront, ColBack, x + 200 - (DigitsMax * 16), y + 14 );
		}
	}
	return KeyResult;
}

/**
  * @brief draws a numeric keypad and text box for result
  * @param string, ColFront, ColBox, ColBack, X, Y
  * @retval none
  */
void TFT_KeypadNumber( const char *String, uint16_t ColFront, uint16_t ColBox, uint16_t ColBack, uint16_t x, uint16_t y ) {
	TFT_ColorBox( x, x + 240, y, y + 380, ColBlack );
	TFT_ColorBox( x + 4, x + 236, y + 4, y + 56, ColBack );
	TFT_DrawBox(x + 3, y + 3, x + 237, y + 57, ColBox );
	TFT_PrintString( 2, String, ColFront, ColBack, x + 10, y + 14 );

	TFT_ButtonNumber( '1', ColFront, ColBox, ColBack, x, y + 60 );
	TFT_ButtonNumber( '2', ColFront, ColBox, ColBack, x +  80, y + 60);
	TFT_ButtonNumber( '3', ColFront, ColBox, ColBack, x + 160, y + 60);
	TFT_ButtonNumber( '4', ColFront, ColBox, ColBack, x, y + 140 );
	TFT_ButtonNumber( '5', ColFront, ColBox, ColBack, x +  80, y + 140 );
	TFT_ButtonNumber( '6', ColFront, ColBox, ColBack, x + 160, y + 140 );
	TFT_ButtonNumber( '7', ColFront, ColBox, ColBack, x, y + 220 );
	TFT_ButtonNumber( '8', ColFront, ColBox, ColBack, x +  80, y + 220 );
	TFT_ButtonNumber( '9', ColFront, ColBox, ColBack, x + 160, y + 220 );
	TFT_ButtonString( "   OK", ColFront, ColBox, ColBack, x + 80, y + 300 );
	TFT_ButtonNumber( '0', ColFront, ColBox, ColBack, x, y + 300 );
}

/**
  * @brief returns 1, when a key is pressed
  * @param X, Y
  * @retval key press
  */
uint8_t TFT_ButtonStringCheck( uint16_t x, uint16_t y ) {
uint8_t NumReturn;
	if(( TouchXresult > (x + 5)) && ( TouchXresult < (x + 155)) && ( TouchYresult > (y + 5)) && ( TouchYresult < (y + 75))) {
		NumReturn = 1;
		TouchXresult = 0;
		TouchYresult = 0;
		TouchTimer = 300;
		Beep(1000, 10);
	}
	else {
		NumReturn = 0;
	}
	return NumReturn;
}

/**
  * @brief draws a string in box for keyboard
  * @param Number, ColFront, ColBox, ColBack, X, Y
  * @retval none
  */
void TFT_ButtonString( const char *String,  uint16_t ColFront, uint16_t ColBox, uint16_t ColBack, uint16_t x, uint16_t y ) {
	TFT_ColorBox(x + 4, x + 156, y + 4, y + 76, ColBack);
	TFT_DrawBox(x + 3, y + 3, x + 157, y + 77, ColBox);
	TFT_PrintString( 1, String, ColFront, ColBack, x + 10, y + 24);
}

/**
  * @brief returns 1, when a key is pressed
  * @param X, Y
  * @retval key press
  */
uint8_t TFT_ButtonNumberCheck( uint16_t x, uint16_t y ) {
uint8_t NumReturn;
	if(( TouchXresult > (x + 5)) && ( TouchXresult < (x + 75)) && ( TouchYresult > (y + 5)) && ( TouchYresult < (y + 75))) {
		NumReturn = 1;
		TouchXresult = 0;
		TouchYresult = 0;
		TouchTimer = 300;
		Beep(1000, 10);
	}
	else {
		NumReturn = 0;
	}
	return NumReturn;
}

/**
  * @brief draws a number in box for keyboard
  * @param Number, ColFront, ColBox, ColBack, X, Y
  * @retval none
  */
void TFT_ButtonNumber( uint8_t Number,  uint16_t ColFront, uint16_t ColBox, uint16_t ColBack, uint16_t x, uint16_t y ) {
	TFT_ColorBox(x + 4, x + 76, y + 4, y + 76, ColBack);
	TFT_DrawBox(x + 3, y + 3, x + 77, y + 77, ColBox);
	TFT_PrintChar( 2, 0, Number, ColFront, ColBack, x + 30, y + 24);
}
