/*
 * ModbusMaster.h
 *
 *  Created on: May 1, 2024
 *      Author: KARAN
 */

#ifndef INC_MODBUSMASTER_H_
#define INC_MODBUSMASTER_H_

// maximum size of data which uart can receive from slave at a time
#define MAX_MB_REV_LEN 32

#define MAX_HOLDING_REG 5
#define MAX_INPUT_REG 5
#define MAX_DIS_COIL 16
#define MAX_COIL 10


typedef enum{
	DATA_NOT_SEND = 0,
	DATA_NOT_RECEIVED,
	CRC_NOT_MATCHED,
	SALVE_ADD_NOT_MATCHED,
	DATA_VALUE_NOT_MATHCED,
	GREATER_THEN_MAX,
	READ_SUCCES,
	WRITE_SUCCESS
}MB_ERROR_DESC;



typedef enum{
	R_COIL = 0X1,
	R_DIS_COIL = 0X2,
	R_HOLD_REG = 0X3,
	R_INPUT_REG = 0X4,
	W_SINGLE_COIL = 0X5,
	W_SINGLE_REG = 0X6,
	W_MUL_COILS = 0X0F,
	W_MUL_REGS = 0X10
}MB_OP;

// initialize the modbus master
/*
 * @param : huart handler in stm32
 *
 * @return: none
 * */
void mbMasterInit(UART_HandleTypeDef *huart);


// Read function for mmodbus master
/*
 * @param: MB_OP code from the enum
 * @param: slave address
 * @param: Register starting address
 * @param: number of register to be read
 * @param: timeout
 *
 * @return: 0 if successful and 1 if any error
 * */
uint8_t mbMasterRead(MB_OP op,uint8_t SlaveAdd, uint16_t StartAdd, uint16_t Size,uint16_t timeout);


//
//// read the holding registers from the slave
///*
// * @param: slave address
// * @param: Register starting address
// * @param: number of register to be read
// * @param: timeout
// *
// * @return: 0 if successful and 1 if any error
// * */
//
//uint8_t mbReadHoldReg(uint8_t SlaveAdd, uint16_t StartAdd, uint16_t Size,uint16_t timeout);
//
//// read the Input registers from the slave
///*
// * @param: slave address
// * @param: Register starting address
// * @param: number of register to be read
// * @param: timeout
// *
// * @return: 0 if successful and 1 if any error
// * */
//uint8_t mbReadInputReg(uint8_t SlaveAdd, uint16_t StartAdd, uint16_t Size,uint16_t timeout);
//
//
//
//
//// read the Coils from the slave
///*
// * @param: slave address
// * @param: Coils starting address
// * @param: number of Coils to be read
// * @param: timeout
// *
// * @return: 0 if successful and 1 if any error*/
//
//uint8_t mbReadCoil(uint8_t SlaveAdd, uint16_t StartAdd, uint16_t Size,uint16_t timeout);
//
//// read the Coils from the slave
///*
// * @param: slave address
// * @param: Discrete Coils starting address
// * @param: number of Coils to be read
// * @param: timeout
// *
// * @return: 0 if successful and 1 if any error*/
//uint8_t mbReadDisCoil(uint8_t SlaveAdd, uint16_t StartAdd, uint16_t Size,uint16_t timeout);
//

// Write a single Coils from the slave
/*
 * @param: slave address
 * @param: Coils starting address
 * @param: number of Coils to be write
 * @param: timeout
 * @param: 1 for high and 0 for low
 *
 * @return: 0 if successful and 1 if any error*/


MB_ERROR_DESC mbWriteSingleCoil(uint8_t SlaveAdd, uint16_t StartAdd, uint16_t Size,
		uint16_t timeout, uint8_t data);

// Write a single Register from the slave
/*
 * @param: slave address
 * @param: Register starting address
 * @param: number of Register to be write
 * @param: timeout
 * @param: data to be written to register
 *
 * @return: 0 if successful and 1 if any error*/
MB_ERROR_DESC mbWriteSingleReg(uint8_t SlaveAdd, uint16_t StartAdd, uint16_t Size,
		uint16_t timeout, uint16_t data) ;

// Write  Multiple Coils to the slave
/*
 * @param: slave address
 * @param: Register starting address
 * @param: number of coils to be write
 * @param: timeout
 * @param: data to be written to coils
 *
 * @return: MB_ERROR_DESC error code */

MB_ERROR_DESC mbWriteMulCoil(uint8_t SlaveAdd, uint16_t StartAdd, uint16_t noofCoils,
		uint16_t timeout, uint8_t *data) ;

// Write  Multiple Register to the slave
/*
 * @param: slave address
 * @param: Register starting address
 * @param: number of coils to be write
 * @param: timeout
 * @param: data to be written to coils
 *
 * @return: MB_ERROR_DESC error code */
MB_ERROR_DESC mbWriteMulReg(uint8_t SlaveAdd, uint16_t StartAdd, uint16_t noofRegs,
		uint16_t timeout, uint16_t *data);

#endif
/* INC_MODBUSMASTER_H_ */
