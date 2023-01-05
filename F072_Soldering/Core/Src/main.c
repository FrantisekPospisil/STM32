/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "HD44780.h"
#include "Eeprom.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t		System				= 0;		// ridi zobrazeni na displeji 	0 - normalni,
											//								1 - MENU - prechazi tlacitkem rot koderu
											//								2 - EEPROM - uklada se do pameti po konci rezimu MENU
											//								3 - Standby - kdyz je dlouho stejna teplota
											//								4 - ERROR - chyba teplomeru

uint8_t		Menu				= 0;		// pozice v MENU, 0 - 8 jsou pro zmenu promenych, 9 - 11 zobrazeni stavu promennych, 12 - exit

uint16_t	TimerBeep			= 0;		// doba trvani tonu, generovano PWM
uint16_t	TimerStart			= 0;		// pocita milisekundy od startu, zastavi se na 10 sekund
uint16_t	TimerSec			= 0;		// pocita cas v milisekundach, pretece pri dosazeni sekundy v preruseni SYSTICK

uint8_t		Temperature[4];					// Buffer pro cteni z MAX31855
uint32_t	TemperatureIn		= 0;		// Teplota obvodu MAX31855
uint32_t	TemperatureOut		= 0;		// Teplota termoclanku
uint16_t	MeanArray[200];					// Pole pro prumerovani hodnot z MAX31855
uint8_t		MeanPointer			= 0;		// Ukazatel v poli
uint16_t	MeanMax				= 40;		// EEPROM - velikost pole pro prumerovani pri mereni teploty
uint32_t	MeanAdd				= 0;		// Scitani pro prumer
uint32_t	TemperatureMeasure	= 0;		// namerena teplota po zprumerovani

uint32_t	AdResult			= 0;		// namerene napeti, prepocitane na volty

uint16_t	TemperatureMin		= 1000;		// EEPROM - minimalni teplota
uint16_t	TemperatureMax		= 4000;		// EEPROM - maximalni teplota
uint16_t	TemperatureSet0		= 1400;		// EEPROM - nastavena teplota
uint16_t	TemperatureSet		= 2700;		// nastavena teplota v normalnim rezimu, kopiruje se z TempSet0
uint16_t	TemperatureSetStandby = 0;		// zaloha teploty, ktera bude vracena po opusteni Standby rezimu

uint16_t	StandbyTemperature	= 1000;		// EEPROM - pozadovana teplota v Standby rezimu
uint16_t	StandbyDifference	= 80;		// EEPROM - rozdil teplot ktery nesmi byt prekrocen, aby zacal Standby rezim
uint16_t	StandbyTimerMax		= 300;		// EEPROM - timer, ktery pocita cas kdy se nesmi menit teplota pro dosazeni standby rezimu
uint16_t	TimerStandby		= 0;		// casovac v sekundach, po dosazeni StandbyTimerMax se pusti standby rezim

uint16_t	Const1				= 500;		// EEPROM - konstanta PID regulatoru proporcionalni
uint16_t	Const2				= 20;		// EEPROM - konstanta PID regulatoru integralni
int16_t		Difference			= 0;		// PID - proporcionalni odchylka
int32_t		DifferenceIntegral	= 0;		// PID - integralni odchylka
int16_t		SolderPWM			= 0;		// vypocteny vykon v desetinach procenta

uint16_t	CounterTemperature	= 0;		// pocitadlo, ktere ma pri dosazeni pozadovane teploty hodnotu 1, pak je melodie a pak hodnota 2

uint8_t		EepromBuffer[30];				// Buffer pro zapis a cteni z EEPROM

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  /* * * * * * * * * * * * * * * * * * * * */
  /* * * Aktivace PWM pro pajku        * * */
  /* * * * * * * * * * * * * * * * * * * * */
  LL_TIM_EnableCounter(TIM1);								// PWM pajky
  LL_TIM_EnableAllOutputs(TIM1);
  LL_TIM_SetAutoReload(TIM1, 1000);							// perioda PWM signalu v ns
  LL_TIM_OC_SetCompareCH1(TIM1, 0);
  TemperatureSet = 100;

  LL_mDelay( 250 );

  LL_SYSTICK_EnableIT();							// aktivuje preruseni po 1ms
  LL_ADC_Enable(ADC1);
  LL_ADC_REG_StartConversion(ADC1);

  /* * * * * * * * * * * * * * * * * * * * * * * * * * */
  /* * * Precte hodnoty promennych z EEPROM        * * */
  /* * * * * * * * * * * * * * * * * * * * * * * * * * */

  EEPROM_SPI_INIT(&hspi2);
  EEPROM_SPI_ReadBuffer(EepromBuffer, (uint16_t)0x03, (uint16_t)22 );
  if((EepromBuffer[18] == 0xA5) && (EepromBuffer[19] == 0x62)){
	  TemperatureMin = EepromBuffer[ 0] * 256 + EepromBuffer[ 1];
	  TemperatureMax = EepromBuffer[ 2] * 256 + EepromBuffer[ 3];
	  TemperatureSet0 = EepromBuffer[ 4] * 256 + EepromBuffer[ 5];
	  StandbyTemperature = EepromBuffer[ 6] * 256 + EepromBuffer[ 7];
	  StandbyDifference = EepromBuffer[ 8] * 256 + EepromBuffer[ 9];
	  StandbyTimerMax = EepromBuffer[10] * 256 + EepromBuffer[11];
	  MeanMax = EepromBuffer[12] * 256 + EepromBuffer[13];
	  Const1 = EepromBuffer[14] * 256 + EepromBuffer[15];
	  Const2 = EepromBuffer[16] * 256 + EepromBuffer[17];
  }

  /* * * * * * * * * * * * * * * * * * * * * * * * * * */
  /* * * Aktivuje display a zahraje melodii        * * */
  /* * * * * * * * * * * * * * * * * * * * * * * * * * */

  LCD_Init();
  LL_mDelay( 250 );
  LL_GPIO_SetOutputPin(LCD_LED_GPIO_Port, LCD_LED_Pin);		// zapne podsviceni
  LL_GPIO_SetOutputPin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin);	// CS obvodu mereni teploty
  LL_GPIO_SetOutputPin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin);	// CS obvodu EEPROM
  LCD_Init();
  LCD_Position( 0, 0 );
  LCD_WriteCString( " 03_Soldering   " );
  LCD_Position( 1, 0 );
  LCD_WriteCString( " 30.12.2022     " );
  Music();
  LCD_Clear();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
	  /* * * * * * * * * * * * * * * * * * * * * * * * * * */
	  /* * * Ovlada teplomer                           * * */
	  /* * * * * * * * * * * * * * * * * * * * * * * * * * */

	  if((TimerStart > 4000) && (TimerStart < 5000)) TemperatureSet = TemperatureSet0;	// nastavi pozadovanou teplotu 4 sekundy od startu
	  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
	  HAL_SPI_Receive(&hspi1, Temperature, 4, 1000);								// precte data z termoclanku
	  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
	  TemperatureIn = (Temperature[3] >> 4) | (Temperature[2] << 4);
	  TemperatureIn = TemperatureIn * 625 / 1000;									// vypoctena teplota obvodu a zaokrouhleno na desetiny stupne
	  TemperatureOut = (Temperature[1] >> 2) | (Temperature[0] << 6);
	  TemperatureOut &= 0x03FFF;
	  TemperatureOut = TemperatureOut * 25 / 10;									// vypoctena teplota clanku a zaokrouhleno na desetiny stupne
	  MeanArray[MeanPointer] = TemperatureOut;
	  MeanAdd += MeanArray[MeanPointer];											// pricita k prumeru
	  if(MeanPointer < MeanMax) MeanPointer++; else MeanPointer = 0;
	  MeanAdd -= MeanArray[MeanPointer];											// odecita od prumeru posledni z pole
	  TemperatureMeasure = (uint16_t) (MeanAdd / MeanMax);							// vypocte prumer

	  if(((Temperature[3] & 0x0F) != 0x00) && ((Temperature[3] & 0x0F) != 0x02)) {	// hleda chybu teplomeru krome pripojeni ke GND
		  System = 4;
		  TemperatureMeasure = 6000;
		  LL_TIM_OC_SetCompareCH1(TIM1, 0);
	  }
	  if(((Temperature[3] & 0x0F) == 0x00) && (System == 4)) System = 0;

	  LL_mDelay(50);

	  /* * * * * * * * * * * * * * * * * * * * * * * * * * */
	  /* * * Meri napajeci napeti a vypne zarizeni     * * */
	  /* * * * * * * * * * * * * * * * * * * * * * * * * * */

	  AdResult = LL_ADC_REG_ReadConversionData12(ADC1);								// zmeri napajeci napeti
	  AdResult = ( AdResult * 303 ) >> 14;											// y = 0.0185 x
	  if((AdResult < 15) && (TimerStart > 5000)) {									// low voltage, it will shut down
		  LL_TIM_OC_SetCompareCH2(TIM1, 0);											// stops PWM heating
		  LL_ADC_Disable(ADC1);
		  LL_GPIO_ResetOutputPin(LCD_LED_GPIO_Port, LCD_LED_Pin);					// turns off the backlight
		  LCD_Position( 0, 0 );
		  LCD_WriteCString( "Low voltage, it " );
		  LCD_Position( 1, 0 );
		  LCD_WriteCString( "will shut down  " );
		  MusicEnd();
		  LL_SYSTICK_DisableIT();													// stops SYSTICK
		  while (1) {}																// vypinani je dokonceno
	  }

	  /* * * * * * * * * * * * * * * * * * * * * * * * * * */
	  /* * * Zobrazeni na displeji v beznem rezimu     * * */
	  /* * * * * * * * * * * * * * * * * * * * * * * * * * */

	  switch(System & 0x0F) {														// promenna System je ovladana rotacnim koderem v programu preruseni
	  case 0: {
		  LCD_Position( 0, 0 );														// normalni zobrazeni
		  LCD_WriteCString( "Tsold = " );
		  LCD_PrTemp( TemperatureMeasure );											// namerena teplota
		  LCD_WriteCString( " " );
		  LCD_Position( 1, 0 );
		  LCD_PrTemp( TemperatureSet );													// pozadovana teplota
		  LCD_WriteCString( "P =" );
		  LCD_PrintNumber(SolderPWM / 10, 3, 0);									// vykon zaokruhleny na cela procenta
		  LCD_WriteCString( "% " );													// konec normalniho zobrazeni
		  break;
	  }

	  /* * * * * * * * * * * * * * * * * * * * * * * * * * */
	  /* * * Zobrazeni na displeji v rezimu MENU       * * */
	  /* * * * * * * * * * * * * * * * * * * * * * * * * * */

	  case 1: {
		  LCD_Position( 0, 0 );														// zobrazeni MENU
		  switch(Menu) {
		  case 0: {
			  LCD_WriteCString( "1 Teplota Tmin  " );
	  		  LCD_Position( 1, 0 );
	  		  LCD_PrTemp( TemperatureMin );											// minimalni teplota, kterou lze nastavit
	  		  LCD_WriteCString( "      " );
	  		  break;
		  }
		  case 1: {
			  LCD_WriteCString( "2 Teplota Tmax  " );
			  LCD_Position( 1, 0 );
			  LCD_PrTemp( TemperatureMax );											// maximalni teplota, kteru lze nastavit
			  LCD_WriteCString( "      " );
			  break;
		  }
		  case 2: {
			  LCD_WriteCString( "3 Teplota Tset  " );
			  LCD_Position( 1, 0 );
			  LCD_PrTemp( TemperatureSet0 );										// pozadovana teplota po zapnuti
			  LCD_WriteCString( "      " );
			  break;
		  }
		  case 3: {
	  		  LCD_WriteCString( "4 Teplota Standb" );
	  		  LCD_Position( 1, 0 );
	  		  LCD_PrTemp( StandbyTemperature );										// teplota ve Standby rezimu
	  		  LCD_WriteCString( "      " );
	  		  break;
		  }
		  case 4: {
			  LCD_WriteCString( "5 Standby Diff  " );
			  LCD_Position( 1, 0 );
			  LCD_PrPwm( StandbyDifference );										// rozdil teploty, ktery nesmi byt prekrocen, aby byl Standby rezim
			  LCD_WriteCString( "       " );
			  break;
		  }
		  case 5: {
			  LCD_WriteCString( "6 Standby timer " );
			  LCD_Position( 1, 0 );
			  LCD_PrintNumber( StandbyTimerMax, 4, 0 );								// cas, za ktery se prepina do Standby
			  LCD_WriteCString( " sec      " );
			  break;
		  }
		  case 6: {
			  LCD_WriteCString( "7 Velikost pole " );
			  LCD_Position( 1, 0 );
			  LCD_PrintNumber( MeanMax, 4, 0 );										// velikost pole pro prumerovani pri mereni
			  LCD_WriteCString( "           " );
			  break;
		  }
		  case 7: {
	  		  LCD_WriteCString( "8 Proporcionalni" );
	  		  LCD_Position( 1, 0 );
	  		  LCD_PrintNumber( Const1, 4, 0 );										// konstanta pro proporcionalni regulaci
	  		  LCD_WriteCString( "           " );
	  		  break;
		  }
		  case 8: {
	  		  LCD_WriteCString( "9 Integralni    " );
	  		  LCD_Position( 1, 0 );
	  		  LCD_PrintNumber( Const2, 4, 0 );										// konstanta pro integralni regulaci
	  		  LCD_WriteCString( "           " );
	  		  break;
		  }
		  case 9: {
	  		  LCD_WriteCString( "10 Rozdil + int " );
	  		  LCD_Position( 1, 0 );
	  		  LCD_PrintNumber( Difference, 4, 0 );									// vypisuje proporcionalni a integralni rozdil
	  		  LCD_WriteCString( "  " );
	  		  LCD_PrintNumber( DifferenceIntegral, 6, 0 );
	  		  break;
		  }
		  case 10: {
	  		  LCD_WriteCString( "11 Prumerovani  " );
	  		  LCD_Position( 1, 0 );
	  		  LCD_PrTemp( TemperatureOut );											// vypisuje aktualni namerenou a prumernou teplotu
	  		  LCD_WriteCString( " " );
	  		  LCD_PrTemp( TemperatureMeasure );
	  		  break;
		  }
		  case 11: {
	  		  LCD_WriteCString( "12 Teplota okoli" );
	  		  LCD_Position( 1, 0 );
	  		  LCD_PrTemp( TemperatureIn );											// vypisuje teplotu okoli
	  		  LCD_WriteCString( "       " );
	  		  break;
		  }
		  case 12: {
	  		  LCD_WriteCString( "Napajeci napeti:" );
	  		  LCD_Position( 1, 0 );
	  		  LCD_PrintNumber( AdResult, 2, 0);										// vypisuje napajeci napeti
	  		  LCD_WriteCString( " Voltu - - - " );
	  		  break;
		  }
		  case 13: {
	  		  LCD_WriteCString( "Exit menu       " );								// konec MENU, promenna System pujde na hodnotu 2, aby se ulozilo do EEPROM
	  		  LCD_Position( 1, 0 );
	  		  LCD_WriteCString( "                " );
	  		  break;
		  }
		  }
		  if ((System == 0x11) && (Menu < 9)) {										// zobrazuje zmenu hodnot
			  LCD_WriteCString( "^" );
		  } else {
			  LCD_WriteCString( " " );
		  }
		  break;

	  }

	  /* * * * * * * * * * * * * * * * * * * * * * * * * * */
	  /* * * Zobrazeni na displeji na konci MENU       * * */
	  /* * * * * * * * * * * * * * * * * * * * * * * * * * */

	  case 2: {
		  LCD_Position( 0, 0 );
		  LCD_WriteCString( "   W R I T E    " );
		  LCD_Position( 1, 0 );
		  LCD_WriteCString( "   E E P R O M  " );
		  LL_mDelay( 500 );
		  LCD_Clear();
		  System = 0;

		  EepromBuffer[ 0] = TemperatureMin >> 8;										// zapise promenne do EEPROM
		  EepromBuffer[ 1] = (uint8_t) (TemperatureMin);
		  EepromBuffer[ 2] = TemperatureMax >> 8;
		  EepromBuffer[ 3] = (uint8_t) (TemperatureMax);
		  EepromBuffer[ 4] = TemperatureSet0 >> 8;
		  EepromBuffer[ 5] = (uint8_t) (TemperatureSet0);
		  EepromBuffer[ 6] = StandbyTemperature >> 8;
		  EepromBuffer[ 7] = (uint8_t) (StandbyTemperature);
		  EepromBuffer[ 8] = StandbyDifference >> 8;
		  EepromBuffer[ 9] = (uint8_t) (StandbyDifference);
		  EepromBuffer[10] = StandbyTimerMax >> 8;
		  EepromBuffer[11] = (uint8_t) (StandbyTimerMax);
		  EepromBuffer[12] = MeanMax >> 8;
		  EepromBuffer[13] = (uint8_t) (MeanMax);
		  EepromBuffer[14] = Const1 >> 8;
		  EepromBuffer[15] = (uint8_t) (Const1);
		  EepromBuffer[16] = Const2 >> 8;
		  EepromBuffer[17] = (uint8_t) (Const2);
		  EepromBuffer[18] = 0xA5;
		  EepromBuffer[19] = 0x62;

		  EEPROM_SPI_INIT(&hspi2);
		  EEPROM_SPI_WriteBuffer(EepromBuffer, (uint16_t)0x03, (uint16_t)26 );

		  break;
	  }

	  /* * * * * * * * * * * * * * * * * * * * * * * * * * */
	  /* * * Zobrazeni na displeji v rezimu Standby    * * */
	  /* * * * * * * * * * * * * * * * * * * * * * * * * * */

	  case 3: {
		  LCD_Position( 1, 0 );														// Standby zobrazeni a zadana teplota
		  LCD_WriteCString( "Tsold = " );
		  LCD_PrTemp( TemperatureMeasure );
		  LCD_WriteCString( " " );
		  LCD_Position( 0, 0 );
		  LCD_PrTemp( TemperatureSet );
		  LCD_WriteCString( "* * Standby * * " );
		  TemperatureSet = TemperatureSetStandby;
		  break;
	  }

	  /* * * * * * * * * * * * * * * * * * * * * * * * * * */
	  /* * * Zobrazeni chyby teplomeru                 * * */
	  /* * * * * * * * * * * * * * * * * * * * * * * * * * */

	  case 4: {																		// zobrazeni kdyz je chyba teplomeru
		  switch(Temperature[3] & 0x0F) {
		  case 1: {
			  LCD_Clear();
			  LCD_Position( 0, 0 );
			  LCD_WriteCString( "!  E R R O R ! !" );
			  LCD_Position( 1, 0 );
			  LCD_WriteCString( "Open circuit!   " );
			  break;
		  }
		  case 2: {
			  LCD_Clear();
			  LCD_Position( 0, 0 );
			  LCD_WriteCString( "!  E R R O R ! !" );
			  LCD_Position( 1, 0 );
			  LCD_WriteCString( "Short to GND!   " );
			  break;
		  }
		  case 4: {
			  LCD_Clear();
			  LCD_Position( 0, 0 );
			  LCD_WriteCString( "!  E R R O R ! !" );
			  LCD_Position( 1, 0 );
			  LCD_WriteCString( "Short to Vcc!   " );
			  break;
		  }
		  default: {
			  LCD_Clear();
			  LCD_Position( 0, 0 );
			  LCD_WriteCString( "!  E R R O R ! !" );
			  LCD_Position( 1, 0 );
			  LCD_WriteCString( "* * * * * * * * " );
		  }
		  }
		  Beep( 2000, 100);
		  LL_mDelay( 1000 );
	  }
	  }
	  if(CounterTemperature == 1) {														// pocitadlo, ktere hlida dosazeni pozadovane teploty
		  CounterTemperature = 2;
		  MusicUp();
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
  {
  }
  LL_RCC_HSI48_Enable();

   /* Wait till HSI48 is ready */
  while(LL_RCC_HSI48_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI48);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI48)
  {

  }
  LL_SetSystemCoreClock(48000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Beep( uint32_t Tone, uint16_t Time) {
	Tone = 1000000 / Tone;
	LL_TIM_EnableCounter(TIM2);
	LL_TIM_EnableAllOutputs(TIM2);
	LL_TIM_SetAutoReload(TIM2, Tone);
	LL_TIM_OC_SetCompareCH1(TIM2, 200);
	TimerBeep = Time;
}

void Music( void ) {
	  Beep( TONE_H1, TIME_2a );
	  LL_mDelay( TIME_2 );
	  Beep( TONE_A1, TIME_3a );
	  LL_mDelay( TIME_3 );
	  Beep( TONE_A1, TIME_2a );
	  LL_mDelay( TIME_2 );
	  Beep( TONE_G1, TIME_3a );
	  LL_mDelay( TIME_3 );
	  Beep( TONE_G1, TIME_2a );
	  LL_mDelay( TIME_2 );
	  Beep( TONE_Fx1, TIME_3a );
	  LL_mDelay( TIME_3 );
	  Beep( TONE_A1, TIME_2a );
	  LL_mDelay( TIME_2 );
	  Beep( TONE_G1, TIME_3a );
	  LL_mDelay( TIME_3 );
	  Beep( TONE_G1, TIME_2a );
	  LL_mDelay( TIME_2 );
	  Beep( TONE_Fx1, TIME_3a );
	  LL_mDelay( TIME_3 );
	  Beep( TONE_E1, TIME_1a );
	  LL_mDelay( TIME_1 );
}

void MusicEnd(void) {
	  Beep( TONE_C4, 50 );
	  LL_mDelay( 60 );
	  Beep( TONE_C3, 50 );
	  LL_mDelay( 60 );
	  Beep( TONE_C2, 50 );
	  LL_mDelay( 60 );
	  Beep( TONE_C1, 50 );
	  LL_mDelay( 60 );
}

void MusicUp(void) {
	  Beep( TONE_C1, 50 );
	  LL_mDelay( 60 );
	  Beep( TONE_C2, 50 );
	  LL_mDelay( 60 );
	  Beep( TONE_C3, 50 );
	  LL_mDelay( 60 );
	  Beep( TONE_C4, 50 );
	  LL_mDelay( 60 );
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
