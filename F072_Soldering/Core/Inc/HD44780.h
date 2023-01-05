/*
 * HD44780.h
 *
 *  Created on: Dec 26, 2022
 *      Author: František Pospíšil
 */

#ifndef INC_HD44780_H_
#define INC_HD44780_H_

#include "main.h"

void LCD_Write( uint8_t val );								// zapise kod na displej
void LCD_WriteComm( uint8_t val );							// zapise ridici kod na displej
void LCD_WriteData( uint8_t val );							// zapise data na displej
void LCD_Clear( void );										// vymaze obsah displeje a nastavi kurzor na zacatek
void LCD_Init( void );										// provede inicializaci displeje
void LCD_Position( uint8_t y, uint8_t x );					// nastavi pozici kurzoru
void LCD_WriteCString( const char* retezec );				// vypise konstantni retezec
unsigned long int LCD_Mypow( unsigned char m, unsigned char n );
void LCD_PrintNumber( uint64_t Num, uint8_t NumLen, uint8_t Decimal );
void LCD_PrTemp( uint16_t val );							// vypisuje teplotu se znamenkem
void LCD_PrPwm( uint16_t val );								// vypisuje %
void LCD_PrHex( uint8_t val );								// dva znaky - 8 bitu
void delay(uint32_t del);									// hloupy delay

#endif /* INC_HD44780_H_ */
