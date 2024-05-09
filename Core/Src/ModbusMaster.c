/*
 * ModbusMaster.c
 *
 *  Created on: May 1, 2024
 *      Author: KARAN
 */
#include "stm32f1xx_hal.h"
#include "main.h"
#include "modbus_crc.h"
#include "ModbusMaster.h"
#include "string.h"
UART_HandleTypeDef *mbUart;

uint16_t mbHoldingReg[MAX_HOLDING_REG];
uint16_t mbInputReg[MAX_INPUT_REG];
#if MAX_DIS_COIL%8
uint8_t mbDisCoils[MAX_DIS_COIL/8+1];
#else
uint8_t mbDisCoils[MAX_DIS_COIL / 8];
#endif

#if MAX_COIL%8
uint8_t mbCoils[MAX_COIL / 8 + 1];
#else
uint8_t mbCoils[MAX_COIL/8];
#endif

uint8_t mbRevBuff[MAX_MB_REV_LEN];
uint8_t mbSlaveAdd = 0;

void mbMasterInit(UART_HandleTypeDef *huart) {
	mbUart = huart;
	HAL_GPIO_WritePin(RS485_GPIO_Port, RS485_Pin, GPIO_PIN_RESET);
	//HAL_UARTEx_ReceiveToIdle_IT(mbUart, mbRevBuff, MAX_MB_REV_LEN);

}

static HAL_StatusTypeDef sendData(uint8_t *data, uint16_t len, uint16_t timeout) {
	HAL_StatusTypeDef res;
	HAL_GPIO_WritePin(RS485_GPIO_Port, RS485_Pin, GPIO_PIN_SET);
	res = HAL_UART_Transmit(mbUart, data, len, timeout);
	HAL_GPIO_WritePin(RS485_GPIO_Port, RS485_Pin, GPIO_PIN_RESET);
	return res;
}

uint8_t mbMasterRead(MB_OP op, uint8_t SlaveAdd, uint16_t StartAdd,
		uint16_t Size, uint16_t timeout) {

	if (op == R_COIL && Size > MAX_COIL)
		return 1;
	else if (op == R_DIS_COIL && Size > MAX_DIS_COIL)
		return 1;
	else if (op == R_HOLD_REG && Size > MAX_HOLDING_REG)
		return 1;
	else if (op == R_INPUT_REG && Size > MAX_INPUT_REG)
		return 1;

	uint8_t TxData[8];
	uint16_t j = 0;
	uint16_t expRecLen;
	if (op == R_COIL || op == R_DIS_COIL) {
		/*  calculating the bytes for the coils if not in multiple of
		 *  8 then add one or just use size divide 8 and add 5 as two for crc
		 *  1 for slave address 1 for function id and 1 for bytes received
		 */
		expRecLen = (Size % 8 ? Size / 8 + 1 : Size / 8) + 5;
	} else {
		/*  calculating the bytes for the Reg by multipling the len by 2 and adding 5 as
		 * 1 or crc 1 for slave address 1 for function id and 1 for bytes received
		 */
		expRecLen = 5 + 2 * Size;
	}
	mbSlaveAdd = SlaveAdd;
	TxData[0] = SlaveAdd;  // slave address
	TxData[1] = op;  // Function code passed in the function
	TxData[2] = (StartAdd >> 8) & 0xFF;
	TxData[3] = StartAdd & 0xFF;
	//The Register address will be 00000000 00000100 = 4 + 40001 = 40005
	TxData[4] = (Size >> 8) & 0xFF;
	TxData[5] = Size & 0xFF;
	// no of registers to read will be 00000000 00000101 = 5 Registers = 10 Bytes
	uint16_t crc = crc16(TxData, 6);
	TxData[6] = crc & 0xFF;   // CRC LOW
	TxData[7] = (crc >> 8) & 0xFF;  // CRC HIGH
	if (sendData(TxData, 8, timeout) != HAL_OK) {
		return 1;
	} else {

		if (HAL_UART_Receive(mbUart, mbRevBuff, expRecLen, timeout) == HAL_OK) {

			uint16_t crcrev = crc16(mbRevBuff, mbRevBuff[2] + 3);
			if (((crcrev & 0x00FF) != mbRevBuff[mbRevBuff[2] + 3])
					|| (((crcrev & 0xFF00) >> 8) != mbRevBuff[mbRevBuff[2] + 4])) {
				return 1;
				// check if the salve address is same or not
			} else if (mbRevBuff[0] != mbSlaveAdd) {
				return 1;
			}
			// copying all the data to holding reg buffer
			else {

				switch (op) {
				case R_COIL: {
					for (int i = 0; i < mbRevBuff[2]; i++) {
						mbCoils[i] = (mbRevBuff[3 + i]);
					}
					break;
				}
				case R_DIS_COIL: {
					for (int i = 0; i < mbRevBuff[2]; i++) {
						mbDisCoils[i] = (mbRevBuff[3 + i]);
					}
					break;
				}
				case R_INPUT_REG: {
					for (int i = 0; i < mbRevBuff[2] / 2; i++) {
						mbInputReg[i] = (mbRevBuff[3 + j] << 8)
								| mbRevBuff[4 + j];
						j += 2;
					}
					break;
				}
				case R_HOLD_REG: {
					for (int i = 0; i < mbRevBuff[2] / 2; i++) {
						mbHoldingReg[i] = (mbRevBuff[3 + j] << 8)
								| mbRevBuff[4 + j];
						j += 2;
					}
					break;
				}
				default:
					break;
				}

				return 0;
			}
		} else
			return 1;
	}

}

MB_ERROR_DESC mbWriteSingleCoil(uint8_t SlaveAdd, uint16_t StartAdd, uint16_t Size,
		uint16_t timeout, uint8_t data) {
    uint8_t TxData[8];
	mbSlaveAdd = SlaveAdd;
	TxData[0] = SlaveAdd;  // slave address
	TxData[1] = W_SINGLE_COIL;  // Force single coil

	TxData[2] = (StartAdd >> 8) & 0xFF; //address high
	TxData[3] = StartAdd & 0xFF; // address low
	//The coil address will be 00000000 00000000 = 0 + 1 = 1
	if (data == 0)
		TxData[4] = 0;
	else
		TxData[4] = 0xFF; // force data high
	TxData[5] = 0;  // force data low

	uint16_t crc = crc16(TxData, 6);
	TxData[6] = crc & 0xFF;   // CRC LOW
	TxData[7] = (crc >> 8) & 0xFF;  // CRC HIGH

	if (sendData(TxData, 8, timeout) != HAL_OK) {
		return DATA_NOT_SEND;
	} else {

		if (HAL_UART_Receive(mbUart, mbRevBuff, 8, timeout) == HAL_OK) {

			uint16_t crcrev = crc16(mbRevBuff, 6);
			if (((crcrev & 0x00FF) != mbRevBuff[6])
					|| (((crcrev & 0xFF00) >> 8) != mbRevBuff[7])) {
				return CRC_NOT_MATCHED;
				// check if the salve address is same or not
			} else if (mbRevBuff[0] != mbSlaveAdd) {
				return SALVE_ADD_NOT_MATCHED;
			} else if (mbRevBuff[4] != TxData[4] || mbRevBuff[5] != TxData[5]) {
				return DATA_VALUE_NOT_MATHCED;
			} else
				return WRITE_SUCCESS;
		} else
			return DATA_NOT_RECEIVED;
	}
}



MB_ERROR_DESC mbWriteMulCoil(uint8_t SlaveAdd, uint16_t StartAdd, uint16_t noofCoils,
		uint16_t timeout, uint8_t *data) {

	    uint16_t bytecount = (noofCoils % 8 ? noofCoils / 8 + 1 : noofCoils / 8);
	    uint16_t len = 9+bytecount;
	    uint8_t TxData[len];
		mbSlaveAdd = SlaveAdd;
		TxData[0] = SlaveAdd;  // slave address
		TxData[1] = W_MUL_COILS;  // Force single coil

		TxData[2] = (StartAdd >> 8) & 0xFF; //address high
		TxData[3] = StartAdd & 0xFF; // address low
		//The coil address will be 00000000 00000000 = 0 + 1 = 1

	    TxData[4] = (noofCoils>>8)&0xFF; // noofCoils high
		TxData[5] = noofCoils&0xFF;  // noofCoils low

		TxData[6] = bytecount;// Byte count

		memcpy(&TxData[7],data,bytecount);

		uint16_t crc = crc16(TxData, len-2);
		TxData[len-2] = crc & 0xFF;   // CRC LOW
		TxData[len-1] = (crc >> 8) & 0xFF;  // CRC HIGH

		if (sendData(TxData, len, timeout) != HAL_OK) {
			return DATA_NOT_SEND;
		} else {

			if (HAL_UART_Receive(mbUart, mbRevBuff, 8, timeout) == HAL_OK) {

				uint16_t crcrev = crc16(mbRevBuff, 6);
				if (((crcrev & 0x00FF) != mbRevBuff[6])
						|| (((crcrev & 0xFF00) >> 8) != mbRevBuff[7])) {
					return CRC_NOT_MATCHED;
					// check if the salve address is same or not
				} else if (mbRevBuff[0] != mbSlaveAdd) {
					return SALVE_ADD_NOT_MATCHED;
				} else if (mbRevBuff[4] != TxData[4] || mbRevBuff[5] != TxData[5]) {
					return DATA_VALUE_NOT_MATHCED;
				} else
					return WRITE_SUCCESS;
			} else
				return DATA_NOT_RECEIVED;
		}
}



MB_ERROR_DESC mbWriteSingleReg(uint8_t SlaveAdd, uint16_t StartAdd, uint16_t Size,
		uint16_t timeout, uint16_t data) {
	uint8_t TxData[8];
	mbSlaveAdd = SlaveAdd;
	TxData[0] = SlaveAdd;  // slave address
	TxData[1] = W_SINGLE_REG;  // Force single coil

	TxData[2] = (StartAdd >> 8) & 0xFF; //address high
	TxData[3] = StartAdd & 0xFF; // address low
	//The coil address will be 00000000 00000000 = 0 + 1 = 1

	TxData[4] = (data >> 8) & 0xFF; //  data high
	TxData[5] = data & 0xFF;  //  data low

	uint16_t crc = crc16(TxData, 6);
	TxData[6] = crc & 0xFF;   // CRC LOW
	TxData[7] = (crc >> 8) & 0xFF;  // CRC HIGH

	if (sendData(TxData, 8, timeout) != HAL_OK) {
		return DATA_NOT_SEND;
	} else {

		if (HAL_UART_Receive(mbUart, mbRevBuff, 8, timeout) == HAL_OK) {

			//uint16_t crcrev = crc16(mbRevBuff, mbRevBuff[2] + 3);
			uint16_t crcrev = crc16(mbRevBuff, 6);
			if (((crcrev & 0x00FF) != mbRevBuff[6])
					|| (((crcrev & 0xFF00) >> 8) != mbRevBuff[7])) {
				return CRC_NOT_MATCHED;
				// check if the salve address is same or not
			} else if (mbRevBuff[0] != mbSlaveAdd) {
				return SALVE_ADD_NOT_MATCHED;
			} else if (mbRevBuff[4] != TxData[4] || mbRevBuff[5] != TxData[5]) {
				return DATA_VALUE_NOT_MATHCED;
			} else
				return WRITE_SUCCESS;
		} else
			return DATA_NOT_RECEIVED;
	}
}



MB_ERROR_DESC mbWriteMulReg(uint8_t SlaveAdd, uint16_t StartAdd, uint16_t noofRegs,
		uint16_t timeout, uint16_t *data) {

		uint8_t j =0;
	    uint16_t len = 9+noofRegs*2;
	    uint8_t TxData[len];
		mbSlaveAdd = SlaveAdd;
		TxData[0] = SlaveAdd;  // slave address
		TxData[1] = W_MUL_REGS;  // Force single coil

		TxData[2] = (StartAdd >> 8) & 0xFF; //address high
		TxData[3] = StartAdd & 0xFF; // address low
		//The coil address will be 00000000 00000000 = 0 + 1 = 1

	    TxData[4] = (noofRegs>>8)&0xFF; // noofCoils high
		TxData[5] = noofRegs&0xFF;  // noofCoils low

		TxData[6] = noofRegs*2;// Byte count

		for(int i = 0;i<noofRegs;i++){
			TxData[7+j] = (data[i]>>8)&0xFF;
			TxData[7+j+1] = data[i]&0xFF;
			j+=2;
		}

		uint16_t crc = crc16(TxData, len-2);
		TxData[len-2] = crc & 0xFF;   // CRC LOW
		TxData[len-1] = (crc >> 8) & 0xFF;  // CRC HIGH

		if (sendData(TxData, len, timeout) != HAL_OK) {
			return DATA_NOT_SEND;
		} else {

			if (HAL_UART_Receive(mbUart, mbRevBuff, 8, timeout) == HAL_OK) {

				uint16_t crcrev = crc16(mbRevBuff, 6);
				if (((crcrev & 0x00FF) != mbRevBuff[6])
						|| (((crcrev & 0xFF00) >> 8) != mbRevBuff[7])) {
					return CRC_NOT_MATCHED;
					// check if the salve address is same or not
				} else if (mbRevBuff[0] != mbSlaveAdd) {
					return SALVE_ADD_NOT_MATCHED;
				} else if (mbRevBuff[4] != TxData[4] || mbRevBuff[5] != TxData[5]) {
					return DATA_VALUE_NOT_MATHCED;
				} else
					return WRITE_SUCCESS;
			} else
				return DATA_NOT_RECEIVED;
		}
}


/*

 uint8_t mbReadHoldReg(uint8_t SlaveAdd, uint16_t StartAdd, uint16_t Size,uint16_t timeout){

 uint8_t TxData[8];
 uint16_t j =0;
 uint16_t expRecLen = 5 + 2*Size;
 mbSlaveAdd = SlaveAdd;
 TxData[0] = SlaveAdd;  // slave address
 TxData[1] = R_HOLD_REG;  // Function code for Read Holding Registers
 TxData[2] = (StartAdd>>8)&0xFF;
 TxData[3] = StartAdd & 0xFF;
 //The Register address will be 00000000 00000100 = 4 + 40001 = 40005
 TxData[4] = (Size>>8)&0xFF;
 TxData[5] = Size & 0xFF;
 // no of registers to read will be 00000000 00000101 = 5 Registers = 10 Bytes
 uint16_t crc = crc16(TxData, 6);
 TxData[6] = crc&0xFF;   // CRC LOW
 TxData[7] = (crc>>8)&0xFF;  // CRC HIGH
 if(sendData(TxData,8,timeout)!= HAL_OK){
 return 1;
 }
 else
 {

 if( HAL_UART_Receive(mbUart, mbRevBuff, expRecLen, timeout)== HAL_OK)
 {

 uint16_t crcrev = crc16(mbRevBuff, mbRevBuff[2]+3);
 if(mbRevBuff[2]/2>MAX_HOLDING_REG){
 return 1;
 // check is CRC is matches for the received data
 }else if(((crcrev&0x00FF) != mbRevBuff[mbRevBuff[2]+3]) ||
 (((crcrev & 0xFF00) >> 8)!= mbRevBuff[mbRevBuff[2]+4]))
 {
 return 1;
 // check if the salve address is same or not
 }else if(mbRevBuff[0]!= mbSlaveAdd){
 return 1;
 }
 // copying all the data to holding reg buffer
 else{
 for(int i =0;i<mbRevBuff[2]/2;i++){
 mbHoldingReg[i] = (mbRevBuff[3+j]<<8)|mbRevBuff[4+j];
 j+=2;
 }
 return 0;
 }
 }
 else return 1;
 }
 }




 uint8_t mbReadInputReg(uint8_t SlaveAdd, uint16_t StartAdd, uint16_t Size,uint16_t timeout){

 uint8_t TxData[8];
 uint16_t j =0;
 uint16_t expRecLen = 5 + 2*Size;
 mbSlaveAdd = SlaveAdd;
 TxData[0] = SlaveAdd;  // slave address
 TxData[1] = R_INPUT_REG;  // Function code for Reading Input  Registers
 TxData[2] = (StartAdd>>8)&0xFF;
 TxData[3] = StartAdd & 0xFF;
 //The Register address will be 00000000 00000100 = 4 + 40001 = 40005
 TxData[4] = (Size>>8)&0xFF;
 TxData[5] = Size & 0xFF;
 // no of registers to read will be 00000000 00000101 = 5 Registers = 10 Bytes
 uint16_t crc = crc16(TxData, 6);
 TxData[6] = crc&0xFF;   // CRC LOW
 TxData[7] = (crc>>8)&0xFF;  // CRC HIGH
 if(sendData(TxData,8,timeout)!= HAL_OK){
 return 1;
 }
 else
 {

 if( HAL_UART_Receive(mbUart, mbRevBuff, expRecLen, timeout)== HAL_OK)
 {

 uint16_t crcrev = crc16(mbRevBuff, mbRevBuff[2]+3);
 if(mbRevBuff[2]/2>MAX_HOLDING_REG){
 return 1;
 // check is CRC is matches for the received data
 }else if(((crcrev&0x00FF) != mbRevBuff[mbRevBuff[2]+3]) ||
 (((crcrev & 0xFF00) >> 8)!= mbRevBuff[mbRevBuff[2]+4]))
 {
 return 1;
 // check if the salve address is same or not
 }else if(mbRevBuff[0]!= mbSlaveAdd){
 return 1;
 }
 // copying all the data to holding reg buffer
 else{
 for(int i =0;i<mbRevBuff[2]/2;i++){
 mbInputReg[i] = (mbRevBuff[3+j]<<8)|mbRevBuff[4+j];
 j+=2;
 }
 return 0;
 }
 }
 else return 1;
 }
 }


 uint8_t mbReadCoil(uint8_t SlaveAdd, uint16_t StartAdd, uint16_t Size,uint16_t timeout){

 uint8_t TxData[8];
 // calculating the bytes for the coils if not in multiple of 8 then add one or just use size mod 8
 uint16_t expRecLen = (Size%8?Size/8+1:Size/8) + 5;
 mbSlaveAdd = SlaveAdd;
 TxData[0] = SlaveAdd;  // slave address
 TxData[1] = R_COIL;  // Function code for Reading Input  Registers
 TxData[2] = (StartAdd>>8)&0xFF;
 TxData[3] = StartAdd & 0xFF;
 //The Register address will be 00000000 00000100 = 4 + 40001 = 40005
 TxData[4] = (Size>>8)&0xFF;
 TxData[5] = Size & 0xFF;
 // no of registers to read will be 00000000 00000101 = 5 Registers = 10 Bytes
 uint16_t crc = crc16(TxData, 6);
 TxData[6] = crc&0xFF;   // CRC LOW
 TxData[7] = (crc>>8)&0xFF;  // CRC HIGH
 if(sendData(TxData,8,timeout)!= HAL_OK){
 return 1;
 }
 else
 {

 if( HAL_UART_Receive(mbUart, mbRevBuff, expRecLen, timeout)== HAL_OK)
 {

 uint16_t crcrev = crc16(mbRevBuff, mbRevBuff[2]+3);
 if(mbRevBuff[2]/2>MAX_HOLDING_REG){
 return 1;
 // check is CRC is matches for the received data
 }else if(((crcrev&0x00FF) != mbRevBuff[mbRevBuff[2]+3]) ||
 (((crcrev & 0xFF00) >> 8)!= mbRevBuff[mbRevBuff[2]+4]))
 {
 return 1;
 // check if the salve address is same or not
 }else if(mbRevBuff[0]!= mbSlaveAdd){
 return 1;
 }
 // copying all the data to holding reg buffer
 else{
 for(int i =0;i<mbRevBuff[2];i++){

 mbCoils[i] = (mbRevBuff[3+i]);
 }
 return 0;
 }
 }
 else return 1;
 }
 }


 uint8_t mbReadDisCoil(uint8_t SlaveAdd, uint16_t StartAdd, uint16_t Size,uint16_t timeout){

 uint8_t TxData[8];
 // calculating the bytes for the coils if not in multiple of 8 then add one or just use size mod 8
 uint16_t expRecLen = (Size%8?Size/8+1:Size/8) + 5;
 mbSlaveAdd = SlaveAdd;
 TxData[0] = SlaveAdd;  // slave address
 TxData[1] = R_DIS_COIL;  // Function code for Reading Input  Registers
 TxData[2] = (StartAdd>>8)&0xFF;
 TxData[3] = StartAdd & 0xFF;
 //The Register address will be 00000000 00000100 = 4 + 40001 = 40005
 TxData[4] = (Size>>8)&0xFF;
 TxData[5] = Size & 0xFF;
 // no of registers to read will be 00000000 00000101 = 5 Registers = 10 Bytes
 uint16_t crc = crc16(TxData, 6);
 TxData[6] = crc&0xFF;   // CRC LOW
 TxData[7] = (crc>>8)&0xFF;  // CRC HIGH
 if(sendData(TxData,8,timeout)!= HAL_OK){
 return 1;
 }
 else
 {

 if( HAL_UART_Receive(mbUart, mbRevBuff, expRecLen, timeout)== HAL_OK)
 {

 uint16_t crcrev = crc16(mbRevBuff, mbRevBuff[2]+3);
 if(mbRevBuff[2]/2>MAX_HOLDING_REG){
 return 1;
 // check is CRC is matches for the received data
 }else if(((crcrev&0x00FF) != mbRevBuff[mbRevBuff[2]+3]) ||
 (((crcrev & 0xFF00) >> 8)!= mbRevBuff[mbRevBuff[2]+4]))
 {
 return 1;
 // check if the salve address is same or not
 }else if(mbRevBuff[0]!= mbSlaveAdd){
 return 1;
 }
 // copying all the data to holding reg buffer
 else{
 for(int i =0;i<mbRevBuff[2];i++){

 mbDisCoils[i] = (mbRevBuff[3+i]);
 }
 return 0;
 }
 }
 else return 1;
 }
 }
 */
