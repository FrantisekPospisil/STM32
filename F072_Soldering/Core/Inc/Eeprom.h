/*
 * Eeprom.h
 *
 *  Created on: 17. 11. 2022
 *      Author: František Pospíšil
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include "main.h"

#define EEPROM_WREN		0x06		// write enable
#define EEPROM_WRDI		0x04		// write disable
#define EEPROM_RDSR		0x05		// read status register
#define EEPROM_WRSR		0x01		// write status register
#define EEPROM_READ		0x03		// read from memory array
#define EEPROM_WRITE	0x02		// write to memory array

#define EEPROM_WIP_FLAG		0x01	// write in progress WIP flag
#define EEPROM_PAGESIZE		32
#define EEPROM_BUFFER_SIZE	32

#define EEPROM_CS_HIGH()	LL_GPIO_SetOutputPin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin)
#define EEPROM_CS_LOW()		LL_GPIO_ResetOutputPin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin)
#define EEPROM_H_HIGH()		LL_GPIO_SetOutputPin(SPI2_H_GPIO_Port, SPI2_H_Pin)
#define EEPROM_H_LOW()		LL_GPIO_ResetOutputPin(SPI2_H_GPIO_Port, SPI2_H_Pin)
#define EEPROM_W_HIGH()		LL_GPIO_SetOutputPin(SPI2_W_GPIO_Port, SPI2_W_Pin)
#define EEPROM_W_LOW()		LL_GPIO_ResetOutputPin(SPI2_W_GPIO_Port, SPI2_W_Pin)

/*
#define EEPROM_CS_HIGH()	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET)
#define EEPROM_CS_LOW()		HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET)
#define EEPROM_H_HIGH()		HAL_GPIO_WritePin(SPI2_H_GPIO_Port, SPI2_H_Pin, GPIO_PIN_SET)
#define EEPROM_H_LOW()		HAL_GPIO_WritePin(SPI2_H_GPIO_Port, SPI2_H_Pin, GPIO_PIN_RESET)
#define EEPROM_W_HIGH()		HAL_GPIO_WritePin(SPI2_W_GPIO_Port, SPI2_W_Pin, GPIO_PIN_SET)
#define EEPROM_W_LOW()		HAL_GPIO_WritePin(SPI2_W_GPIO_Port, SPI2_W_Pin, GPIO_PIN_RESET)
*/

typedef enum {					// seznam vysledku operace
	EEPROM_STATUS_PENDING,
	EEPROM_STATUS_COMPLETE,
	EEPROM_STATUS_ERROR
} EepromOperations;

void EEPROM_SPI_INIT(SPI_HandleTypeDef * hspi);

EepromOperations EEPROM_SPI_WritePage(uint8_t* pBuffer, uint16_t WriteAddr, uint16_t NumByteToWrite);
EepromOperations EEPROM_SPI_WriteBuffer(uint8_t* pBuffer, uint16_t WriteAddr, uint16_t NumByteToWrite);
EepromOperations EEPROM_SPI_ReadBuffer(uint8_t* pBuffer, uint16_t ReadAddr, uint16_t NumByteToRead);
uint8_t EEPROM_SPI_WaitStandbyState(void);

uint8_t EEPROM_SendByte(uint8_t byte);
void sEE_WriteEnable(void);
void sEE_WriteDisable(void);
void sEE_WriteStatusRegister(uint8_t regval);

void EEPROM_SPI_SendInstruction(uint8_t* instruction, uint8_t size);


#endif /* INC_EEPROM_H_ */
