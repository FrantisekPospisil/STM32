/*
 * Eeprom.c
 *
 *  Created on: 17. 11. 2022
 *      Author: František Pospíšil
 */

#include "Eeprom.h"

SPI_HandleTypeDef * EEPROM_SPI;
uint8_t		EEPROM_StatusByte;
uint8_t		RxBuffer[EEPROM_BUFFER_SIZE] = {0};

/**
 * @brief Init EEPROM SPI
 *
 * @param hspi Pointer to SPI struct handler
 */
void EEPROM_SPI_INIT(SPI_HandleTypeDef * hspi) {
	EEPROM_SPI = hspi;
}

EepromOperations EEPROM_SPI_WritePage(uint8_t* pBuffer, uint16_t WriteAddr, uint16_t NumByteToWrite) {
	while (EEPROM_SPI->State != HAL_SPI_STATE_READY) {
		HAL_Delay(1);
	}
	HAL_StatusTypeDef spiTransmitStatus;

	sEE_WriteEnable();

	uint8_t header[3];
	header[0] = EEPROM_WRITE;
	header[1] = WriteAddr >> 8;
	header[2] = WriteAddr;

	EEPROM_W_HIGH();
	EEPROM_H_HIGH();
	EEPROM_CS_LOW();

	EEPROM_SPI_SendInstruction((uint8_t*)header, 3);

	for (uint8_t i = 0; i < 5; i++) {
		spiTransmitStatus = HAL_SPI_Transmit(EEPROM_SPI, pBuffer, NumByteToWrite, 100);
		if (spiTransmitStatus == HAL_BUSY) {
			HAL_Delay(5);
		} else {
			break;
		}
	}

	EEPROM_CS_HIGH();
	EEPROM_SPI_WaitStandbyState();
	sEE_WriteDisable();

	if (spiTransmitStatus == HAL_ERROR) {
		return EEPROM_STATUS_ERROR;
	} else {
		return EEPROM_STATUS_COMPLETE;
	}
}

EepromOperations EEPROM_SPI_WriteBuffer(uint8_t* pBuffer, uint16_t WriteAddr, uint16_t NumByteToWrite) {
	uint16_t NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;
	uint16_t sEE_DataNum = 0;

	EepromOperations pageWriteStatus = EEPROM_STATUS_PENDING;

	Addr = WriteAddr % EEPROM_PAGESIZE;
	count = EEPROM_PAGESIZE - Addr;
	NumOfPage = NumByteToWrite / EEPROM_PAGESIZE;
	NumOfSingle = NumByteToWrite % EEPROM_PAGESIZE;

	if (Addr == 0) {
		if (NumOfPage == 0) {
			sEE_DataNum = NumByteToWrite;
			pageWriteStatus = EEPROM_SPI_WritePage(pBuffer, WriteAddr, sEE_DataNum);
			if (pageWriteStatus != EEPROM_STATUS_COMPLETE) {
				return pageWriteStatus;
			}
		} else {
			while (NumOfPage--) {
				sEE_DataNum = EEPROM_PAGESIZE;
				pageWriteStatus = EEPROM_SPI_WritePage(pBuffer, WriteAddr, sEE_DataNum);
				if (pageWriteStatus != EEPROM_STATUS_COMPLETE) {
					return pageWriteStatus;
				}

				WriteAddr += EEPROM_PAGESIZE;
				pBuffer += EEPROM_PAGESIZE;
			}
			sEE_DataNum = NumOfSingle;
			pageWriteStatus = EEPROM_SPI_WritePage(pBuffer, WriteAddr, sEE_DataNum);
			if (pageWriteStatus != EEPROM_STATUS_COMPLETE) {
				return pageWriteStatus;
			}
		}
	} else {
		if (NumOfPage == 0) {
			if (NumOfSingle > count) {
				temp = NumOfSingle - count;
				sEE_DataNum = count;
				pageWriteStatus = EEPROM_SPI_WritePage(pBuffer, WriteAddr, sEE_DataNum);
				if (pageWriteStatus != EEPROM_STATUS_COMPLETE) {
					return pageWriteStatus;
				}
				WriteAddr += count;
				pBuffer += count;
				sEE_DataNum = temp;
				pageWriteStatus = EEPROM_SPI_WritePage(pBuffer, WriteAddr, sEE_DataNum);
			} else {
				sEE_DataNum = NumByteToWrite;
				pageWriteStatus = EEPROM_SPI_WritePage(pBuffer, WriteAddr, sEE_DataNum);
			}
			if (pageWriteStatus != EEPROM_STATUS_COMPLETE) {
				return pageWriteStatus;
			}
		} else {
			NumByteToWrite -= count;
			NumOfPage = NumByteToWrite / EEPROM_PAGESIZE;
			NumOfSingle = NumByteToWrite % EEPROM_PAGESIZE;
			sEE_DataNum = count;
			pageWriteStatus = EEPROM_SPI_WritePage(pBuffer, WriteAddr, sEE_DataNum);
			if (pageWriteStatus != EEPROM_STATUS_COMPLETE) {
				return pageWriteStatus;
			}
			WriteAddr += count;
			pBuffer += count;
			while (NumOfPage--) {
				sEE_DataNum = EEPROM_PAGESIZE;
				pageWriteStatus = EEPROM_SPI_WritePage(pBuffer, WriteAddr, sEE_DataNum);
				if (pageWriteStatus != EEPROM_STATUS_COMPLETE) {
					return pageWriteStatus;
				}
				WriteAddr += EEPROM_PAGESIZE;
				pBuffer += EEPROM_PAGESIZE;
			}
			if (NumOfSingle != 0) {
				sEE_DataNum = NumOfSingle;
				pageWriteStatus = EEPROM_SPI_WritePage(pBuffer, WriteAddr, sEE_DataNum);
				if (pageWriteStatus != EEPROM_STATUS_COMPLETE) {
					return pageWriteStatus;
				}
			}
		}
	}
	return EEPROM_STATUS_COMPLETE;
}

EepromOperations EEPROM_SPI_ReadBuffer(uint8_t* pBuffer, uint16_t ReadAddr, uint16_t NumByteToRead) {
	while (EEPROM_SPI->State != HAL_SPI_STATE_READY) {
		HAL_Delay(1);
	}

	uint8_t header[3];

	header[0] = EEPROM_READ;
	header[1] = ReadAddr >> 8;
	header[2] = ReadAddr;

	EEPROM_W_HIGH();
	EEPROM_H_HIGH();
	EEPROM_CS_LOW();

	EEPROM_SPI_SendInstruction(header, 3);

	while (HAL_SPI_Receive(EEPROM_SPI, (uint8_t*)pBuffer, NumByteToRead, 200) == HAL_BUSY) {
		HAL_Delay(1);
	}
	EEPROM_CS_HIGH();

	return EEPROM_STATUS_COMPLETE;
}

uint8_t EEPROM_SPI_WaitStandbyState(void) {
	uint8_t sEEstatus[1] = { 0x00 };
	uint8_t command[1] = { EEPROM_RDSR };

	EEPROM_W_HIGH();
	EEPROM_H_HIGH();
	EEPROM_CS_LOW();

	EEPROM_SPI_SendInstruction((uint8_t*)command, 1);

	do {
		while (HAL_SPI_Receive(EEPROM_SPI, (uint8_t*)sEEstatus, 1, 200) == HAL_BUSY) {
			HAL_Delay(1);
		};
		HAL_Delay(1);
	} while ((sEEstatus[0] & EEPROM_WIP_FLAG) == SET);
	EEPROM_CS_HIGH();
	return 0;
}

uint8_t EEPROM_SendByte(uint8_t byte) {
	uint8_t answerByte;

	while (EEPROM_SPI->State == HAL_SPI_STATE_RESET) {
		HAL_Delay(1);
	}
	if (HAL_SPI_Transmit(EEPROM_SPI, &byte, 1, 200) != HAL_OK) {
		Error_Handler();
	}
	while (EEPROM_SPI->State == HAL_SPI_STATE_RESET) {
		HAL_Delay(1);
	}
	if (HAL_SPI_Receive(EEPROM_SPI, &answerByte, 1, 200) != HAL_OK) {
		Error_Handler();
	}
	return (uint8_t)answerByte;
}

void sEE_WriteEnable(void) {
	EEPROM_W_HIGH();
	EEPROM_H_HIGH();
	EEPROM_CS_LOW();

	uint8_t command[1] = { EEPROM_WREN };
	EEPROM_SPI_SendInstruction((uint8_t*)command, 1);

	EEPROM_CS_HIGH();
}

void sEE_WriteDisable(void) {
	EEPROM_W_HIGH();
	EEPROM_H_HIGH();
	EEPROM_CS_LOW();

	uint8_t command[1] = { EEPROM_WRDI };
	EEPROM_SPI_SendInstruction((uint8_t*)command, 1);

	EEPROM_CS_HIGH();
}

void sEE_WriteStatusRegister(uint8_t regval) {
	uint8_t command[2];

	command[0] = EEPROM_WRSR;
	command[1] = regval;

	sEE_WriteEnable();

	EEPROM_W_HIGH();
	EEPROM_H_HIGH();
	EEPROM_CS_LOW();

	EEPROM_SPI_SendInstruction((uint8_t*)command, 2);

	EEPROM_CS_HIGH();

	sEE_WriteDisable();
}

void EEPROM_SPI_SendInstruction(uint8_t *instruction, uint8_t size) {
	while (EEPROM_SPI->State == HAL_SPI_STATE_RESET) {
		HAL_Delay(1);
	}

	if (HAL_SPI_Transmit(EEPROM_SPI, (uint8_t*)instruction, (uint16_t)size, 200) != HAL_OK) {
		Error_Handler();
	}
}
