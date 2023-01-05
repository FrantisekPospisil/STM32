/*
 * TFT_Buttons.h
 *
 *  Created on: 22. 10. 2022
 *      Author: František Pospíšil
 */

#ifndef INC_TFT_BUTTONS_H_
#define INC_TFT_BUTTONS_H_

#include "main.h"

/**
  * @brief will convert the number for DS3231
  * @param number
  * @retval number
  */
uint8_t ClockNumer( uint8_t val);

/**
  * @brief checks the clock button and return 1
  * @param string, ColFront, X, Y
  * @retval key press
  */
uint8_t TFT_ButtonClockCheck( uint16_t x, uint16_t y );

/**
  * @brief draws clock from DS3231 in box
  * @param ReDraw, ColFront, ColBox, ColBack, X, Y
  * @retval none
  */
void TFT_ButtonClock( uint8_t ReDraw, uint16_t ColFront, uint16_t ColBox, uint16_t ColBack, uint16_t x, uint16_t y );

/**
  * @brief returns the number from the keypad
  * @param Max digits, ColFront, ColBox, ColBack, X, Y
  * @retval Number from the keypad
  */
uint16_t TFT_KeypadNumberCheck( uint8_t KeyMax, uint16_t ColFront, uint16_t ColBox, uint16_t ColBack, uint16_t x, uint16_t y );

/**
  * @brief draws a numeric keypad and text box for result
  * @param string, ColFront, ColBox, ColBack, X, Y
  * @retval none
  */
void TFT_KeypadNumber( const char *String, uint16_t ColFront, uint16_t ColBox, uint16_t ColBack, uint16_t x, uint16_t y );

/**
  * @brief returns 1, when a key is pressed
  * @param X, Y
  * @retval key press
  */
uint8_t TFT_ButtonStringCheck( uint16_t x, uint16_t y );

/**
  * @brief draws a string in box for keyboard
  * @param Number, ColFront, ColBox, ColBack, X, Y
  * @retval none
  */
void TFT_ButtonString( const char *String,  uint16_t ColFront, uint16_t ColBox, uint16_t ColBack, uint16_t x, uint16_t y );

/**
  * @brief returns 1, when a key is pressed
  * @param X, Y
  * @retval key press
  */
uint8_t TFT_ButtonNumberCheck( uint16_t x, uint16_t y );

/**
  * @brief draws a number in box for keyboard
  * @param Number, ColFront, ColBox, ColBack, X, Y
  * @retval none
  */
void TFT_ButtonNumber( uint8_t Number,  uint16_t ColFront, uint16_t ColBox, uint16_t ColBack, uint16_t x, uint16_t y );

#endif /* INC_TFT_BUTTONS_H_ */
