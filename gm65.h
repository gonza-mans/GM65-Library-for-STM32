/*
 * gm65.h
 *
 *  Created on: 16 dic. 2021
 *      Author: Gonzalo Mansilla
 */

#ifndef INC_GM65_H_
#define INC_GM65_H_

#include "stm32f4xx_hal.h"


#define	QR_START				0
#define	QR_SETUP_OFF			1
#define QR_SETUP_OFF_T			11
#define QR_SETUP_OFF_R			12
#define	QR_IDLE					2
#define	QR_SCAN					3
#define QR_SCAN_T				31
#define	QR_SCAN_RC				32
#define	QR_SCAN_RH				33
#define QR_SCAN_RD				34
#define	QR_SCAN_SETUP_OFF		4
#define QR_SCAN_SETUP_OFF_T		41
#define	QR_SCAN_SETUP_OFF_R		42
#define	QR_ABORT				5
#define QR_RECEPTION_UNKNOWN	6

#define QR_LENGTH_COMMAND				9
#define QR_LENGTH_RESPOND_COMMAND		7
#define QR_LENGTH_HEAD					3

#define QR_COMMAND_WAIT					5000
#define QR_SCAN_WAIT					55000




struct gm65{

	uint8_t qrBuffer[256];
	uint8_t qrLengthDecode;
	uint8_t qrStatus;
	uint8_t qrDataReady;

};



void gm65_init( struct gm65 *myQR, UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim); //prepara el modulo para trabajar con el dato
void gm65_uart_TX_handler(struct gm65 *myQR, UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim);
void gm65_uart_RX_handler(struct gm65 *myQR, UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim);
void gm65_Tim_handler(struct gm65 *myQR, UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim);
void gm65_scan(struct gm65 *myQR, UART_HandleTypeDef *huart);
void gm65_abort(struct gm65 *myQR, UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim);

uint8_t qrRespuestaValida(uint8_t *respuesta);
uint8_t qrRespSetupOnCode(uint8_t *code);
uint16_t gm65_crc(uint8_t* ptr, unsigned int len);



#endif /* INC_GM65_H_ */

