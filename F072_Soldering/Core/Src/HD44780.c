/*
 * HD44780.c
 *
 *  Created on: Dec 26, 2022
 *      Author: František Pospíšil
 */


/**
******************************************************************************
* @file    EXTI/HD44780.h
* @author  Frantisek Pospisil
* @version V1.0.0
* @date    28-01-2020
* @brief   function for display
******************************************************************************/

/* Private function prototypes -----------------------------------------------*/
#include "HD44780.h"
#include "main.h"

// ===========================================================================
void delay(uint32_t del)		// hloupy delay, jednotka je 410ns pri taktu hodin 48MHz
{
	volatile uint32_t i;
	for(i=0;i<del;i++){};
}

// ===========================================================================
void LCD_PrHex(uint8_t val) 		// dva znaky - 8 bitu
{
	unsigned char znak;
	znak = val / 16;
	if (znak < 10)
	LCD_WriteData(znak+0x30);
	else
	LCD_WriteData(znak+0x37);
	znak = val % 16;
	if (znak < 10)
	LCD_WriteData(znak+0x30);
	else
	LCD_WriteData(znak+0x37);
}

// ===========================================================================
void LCD_PrPwm(uint16_t val) 		// vypisuje procenta 6 znaku
{
	unsigned char znak;
	unsigned char nula = 0;
	znak = val / 1000;
	val = val % 1000;
	if (znak!=0)
	{
		LCD_WriteData( znak + 0x30 );
		nula = 1;
	}
	else LCD_WriteData( ' ' );
	znak = val / 100;
	val = val % 100;
	if ((znak!=0) || (nula!=0))
	{
		LCD_WriteData( znak + 0x30 );
	}
	else LCD_WriteData( ' ' );
	znak = val / 10;
	LCD_WriteData( znak + 0x30 );
	LCD_WriteData( ',' );
	LCD_WriteData((val % 10) + 0x30);
	LCD_WriteCString( "%  " );
}

// ===========================================================================
void LCD_PrTemp(uint16_t val) 		// vypisuje teplotu 7 znaku
{
	unsigned char znak;
	unsigned char nula = 0;
	znak = val / 1000;
	val = val % 1000;
	if (znak!=0)
	{
		LCD_WriteData( znak + 0x30 );
		nula = 1;
	}
	else LCD_WriteData( ' ' );
	znak = val / 100;
	val = val % 100;
	if ((znak!=0) || (nula!=0))
	{
		LCD_WriteData( znak + 0x30 );
	}
	else LCD_WriteData( ' ' );
	znak = val / 10;
	LCD_WriteData( znak + 0x30 );
	LCD_WriteData( ',' );
	LCD_WriteData((val % 10) + 0x30);
	LCD_WriteData( 0xDF ); // znak stupen
	LCD_WriteCString( "C  " );
}

/*********************************************************************************
*
* Function Name : LCD_Mypow
* Description	: vypocita n-tou mocninu cisla m
*
*********************************************************************************/
unsigned long int LCD_Mypow( unsigned char m, unsigned char n )
{
	uint32_t result = 1;		// deklaruje promennou pro vypocet vysledku
	while( n-- ) result *= m;	// smycka pro nasobeni
	return result;				// vysledek
}


/*********************************************************************************
*
* Function Name : LCD_PrintNumber
* Description	: napise cislo
* hodnota, pocet znaku, pocet desetinnych mist
*
*********************************************************************************/
void LCD_PrintNumber( uint64_t Num, uint8_t NumLen, uint8_t Decimal )
{
	uint8_t t, temp;
	uint8_t enshow = 0;

	for(t = 0; t < NumLen; t++)
	{
		temp = ( Num / LCD_Mypow( 10, NumLen - t - 1 )) % 10;
		if (( enshow == 0 ) && ( t < ( NumLen - 1 )) && ( t < ( NumLen - Decimal - 1 )))
		{
			if ( temp == 0 )
			{
				LCD_WriteData( ' ' );
				continue;
			}
			else enshow = 1;
		}
		LCD_WriteData( temp + '0' );
		if(( t == ( NumLen - Decimal - 1 )) && ( t < ( NumLen - 1 )))
		{
			LCD_WriteData( ',' );
		}
	}
}

// ===========================================================================
void LCD_WriteCString( const char* retezec ) // vypise konstantni retezec
{
  while(*retezec != '\0')
  {
    LCD_WriteData(*retezec);
	retezec +=1;
  }
}

// ===========================================================================
void LCD_Position( uint8_t y, uint8_t x ) // nastavi pozici kurzoru
{
	switch( y )
	{
		case 0: x+=0x00; break;
		case 1: x+=0x40; break;
		case 2: x+=0x14; break;
		case 3: x+=0x54; break;
	}
	LCD_WriteComm( x | 0x80 );
}

// ===========================================================================
void LCD_Init( void ) // inicializuje displej
{
	LL_GPIO_ResetOutputPin(LCD_E_GPIO_Port, LCD_E_Pin);
	LCD_Clear();
	LCD_WriteComm( 0x28 );			// select 8-bit mode
	LL_mDelay( 5 );					// 5 ms
	LCD_WriteComm( 0x08 );			// select 8-bit mode
	LCD_Clear();
	LCD_WriteComm( 0x06 );			// select 8-bit mode
	LCD_WriteComm( 0x0C );			// select
}

// ===========================================================================
void LCD_Clear( void ) // vymaze obsah displeje a nastavi kurzor na zacatek
{
	LCD_WriteComm( 1 );
	LL_mDelay( 2 );					// 2 ms
}

// ===========================================================================
void LCD_WriteData( uint8_t val ) // zapise data na displej
{
	LL_GPIO_SetOutputPin(LCD_RS_GPIO_Port, LCD_RS_Pin);
	LCD_Write( val );
}

// ===========================================================================
void LCD_WriteComm( uint8_t val ) // zapise ridici kod na displej
{
	LL_GPIO_ResetOutputPin(LCD_RS_GPIO_Port, LCD_RS_Pin);
	LCD_Write( val );
}

// ===========================================================================
void LCD_Write( uint8_t val )		// zapise kod na displej
{
	LL_GPIO_WriteOutputPort( GPIOB, ( LL_GPIO_ReadOutputPort(GPIOB) & 0xFFF0 ) | (( val >> 4 ) & 0x0F ));
	delay(5);
	LL_GPIO_SetOutputPin(LCD_E_GPIO_Port, LCD_E_Pin);
	LL_GPIO_ResetOutputPin(LCD_E_GPIO_Port, LCD_E_Pin);
	LL_GPIO_WriteOutputPort( GPIOB, ( LL_GPIO_ReadOutputPort(GPIOB) & 0xFFF0 ) | ( val & 0x0F ));
	delay(5);
	LL_GPIO_SetOutputPin(LCD_E_GPIO_Port, LCD_E_Pin);
	LL_GPIO_ResetOutputPin(LCD_E_GPIO_Port, LCD_E_Pin);
	delay( 100 );	// hodnota 100 pro oranzovy displej
}
