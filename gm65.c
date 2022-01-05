/*
 * gm65.c
 *
 *  Created on: 16 dic. 2021
 *      Author: Gonzalo Mansilla
 */

#include "gm65.h"

#include "stm32f4xx_hal.h"

/*
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

*/
uint8_t qrSetupOFF[9]={ 			0x7E, 0,
									0x08,
									0x01,
									0x00, 0x03,
									0x03,
									0x11, 0xA9};
uint8_t qrWriteOK[]={				0x02, 0x00,
									0x00,
									0x01,
									0x00,
									0x33, 0x31};
uint8_t qrScan[]={					0x7E, 0x00,
									0x08,
									0x01,
									0x00, 0x02,
									0x01,
									0xAB, 0xCD};
uint8_t qrSetupOnRespondCode[]={0x51, 0x80, 0x52, 0x30, 0x33, 0x30, 0x32, 0x30, 0x30, 0x2E};

uint8_t buffAux[11];
uint8_t dataLeft;
uint8_t scanPending=0;
uint8_t SetupOnAnalyze = 0;

//CRC-CCITT (XModem)
uint16_t gm65_crc(uint8_t* ptr, unsigned int len)
{
unsigned int crc = 0;
while(len-- != 0)
{
for(unsigned char i = 0x80; i != 0; i >>= 1)
{
crc <<= 1;
if((crc&0x10000) !=0)
crc ^= 0x11021;
if((*ptr&i) != 0)
crc ^= 0x1021;
}
ptr++;
}
return crc;
}

uint8_t qrRespondValid(uint8_t *respond){
	uint8_t flag = 0;
	for(int i=0; i<7; i++){
		if(*(respond+i) != *(qrWriteOK+i)){
			flag = 1;
			break;
		}
	}
	return flag;
}


uint8_t qrRespSetupOnCode(uint8_t *code){
	uint8_t flag = 0;
	for(int i=0; i<10; i++){
		if(*(code+i) != *(qrSetupOnRespondCode+i)){
			flag = 1;
			break;
		}
	}
	return flag;
}



void gm65_init(struct gm65 *myQR, UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim){
	__HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
	__HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
	HAL_UART_Transmit_IT(huart, qrSetupOFF, QR_LENGTH_COMMAND);
	__HAL_TIM_SET_COUNTER(htim, 0);
	myQR->qrStatus =  QR_SETUP_OFF_T;
	myQR->qrDataReady=0;
}

void gm65_uart_TX_handler(struct gm65 *myQR, UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim){
	HAL_TIM_Base_Stop_IT(htim);
	switch (myQR->qrStatus){
	case QR_SETUP_OFF_T:
		HAL_UART_Receive_IT(huart, myQR->qrBuffer, QR_LENGTH_RESPOND_COMMAND);
		__HAL_TIM_SET_AUTORELOAD(htim, QR_COMMAND_WAIT);
		__HAL_TIM_SET_COUNTER(htim, 0);
		HAL_TIM_Base_Start_IT(htim);
		myQR->qrStatus =  QR_SETUP_OFF_R;
		break;
	case QR_SCAN_T:
		HAL_UART_Receive_IT(huart, myQR->qrBuffer, QR_LENGTH_RESPOND_COMMAND);
		__HAL_TIM_SET_AUTORELOAD(htim, QR_COMMAND_WAIT);
		__HAL_TIM_SET_COUNTER(htim, 0);
		HAL_TIM_Base_Start_IT(htim);
		myQR->qrStatus =  QR_SCAN_RC;
		break;
	case QR_SCAN_SETUP_OFF_T:
		HAL_UART_Receive_IT(huart, myQR->qrBuffer, QR_LENGTH_RESPOND_COMMAND);
		__HAL_TIM_SET_AUTORELOAD(htim, QR_COMMAND_WAIT);
		__HAL_TIM_SET_COUNTER(htim, 0);
		HAL_TIM_Base_Start_IT(htim);
		myQR->qrStatus =  QR_SCAN_SETUP_OFF_R;
		break;
	default:
		break;
	}

}

void gm65_uart_RX_handler(struct gm65 *myQR, UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim){
	HAL_TIM_Base_Stop_IT(htim);
	__HAL_TIM_SET_COUNTER(htim,0);
	switch (myQR->qrStatus){
	case QR_SETUP_OFF_R:
		if(qrRespondValid(myQR->qrBuffer) == 1){
			HAL_UART_Transmit_IT(huart, qrSetupOFF, QR_LENGTH_COMMAND);
			myQR->qrStatus = QR_SETUP_OFF_T;

		}
		else{
			myQR->qrStatus =  QR_START;
			HAL_UART_Receive_IT(huart, buffAux, QR_LENGTH_HEAD);
			dataLeft=0;
		}
		break;

	case QR_SCAN_RC:
		if(qrRespondValid(myQR->qrBuffer) == 1){
			HAL_UART_Transmit_IT(huart, qrScan, QR_LENGTH_COMMAND);
			myQR->qrStatus =  QR_SCAN_T;
		}
		else{
			HAL_UART_Receive_IT(huart, myQR->qrBuffer, QR_LENGTH_HEAD);
			__HAL_TIM_SET_AUTORELOAD(htim, QR_SCAN_WAIT);
			HAL_TIM_Base_Start_IT(htim);
			myQR->qrStatus =  QR_SCAN_RH;
		}
		break;
	case QR_SCAN_RH:
		if((myQR->qrBuffer[0]=0x03) && (myQR->qrBuffer[1] == 0x00)){
			myQR->qrLengthDecode = myQR->qrBuffer[2];
			HAL_UART_Receive_IT(huart, myQR->qrBuffer, myQR->qrLengthDecode);
			__HAL_TIM_SET_AUTORELOAD(htim, QR_SCAN_WAIT);
			HAL_TIM_Base_Start_IT(htim);
			myQR->qrStatus =  QR_SCAN_RD;
		}
		else{
			HAL_UART_Transmit_IT(huart, qrScan, QR_LENGTH_COMMAND);
			myQR->qrStatus =  QR_SCAN_T;
		}
		break;
	case QR_SCAN_RD:
		if((myQR->qrLengthDecode == 0x0B) && (qrRespSetupOnCode(myQR->qrBuffer)==0)){
			HAL_UART_Transmit_IT(huart, qrSetupOFF, QR_LENGTH_COMMAND);
			myQR->qrStatus =  QR_SCAN_SETUP_OFF_T;
		}
		else{
			myQR->qrDataReady=1;
			myQR->qrStatus =  QR_START;
		}
		break;
	case QR_SCAN_SETUP_OFF_R:
		if(qrRespondValid(myQR->qrBuffer) == 1){
			HAL_UART_Transmit_IT(huart, qrSetupOFF, QR_LENGTH_COMMAND);
			myQR->qrStatus =  QR_SCAN_SETUP_OFF_T;
		}
		else{
			HAL_UART_Transmit_IT(huart, qrScan, QR_LENGTH_COMMAND);
			myQR->qrStatus =  QR_SCAN_T;
		}
		break;
	case QR_START:
		if((buffAux[0]==0x03) && (buffAux[1]==0x00)){
			dataLeft = buffAux[2];
			if(dataLeft <= 11){
				HAL_UART_Receive_IT(huart, buffAux, dataLeft);
				__HAL_TIM_SET_AUTORELOAD(htim, QR_COMMAND_WAIT);
				HAL_TIM_Base_Start_IT(htim);
				if(dataLeft == 0x0B) SetupOnAnalyze=1;
				dataLeft=0;
			}
			else{
				dataLeft=dataLeft - 11;
				HAL_UART_Receive_IT(huart, buffAux, dataLeft);
				__HAL_TIM_SET_AUTORELOAD(htim, QR_COMMAND_WAIT);
				HAL_TIM_Base_Start_IT(htim);
			}
			myQR->qrStatus= QR_RECEPTION_UNKNOWN;
			break;
	case QR_RECEPTION_UNKNOWN:
		if(dataLeft ==0){
			if((SetupOnAnalyze == 1) && (qrRespSetupOnCode(buffAux)==0)){
				gm65_init(myQR, huart, htim);
				SetupOnAnalyze = 0;
			}
		}
		myQR->qrStatus=QR_START;
		if(scanPending == 1) {
			scanPending = 0;
			gm65_scan(myQR, huart);
		}

		}
	default:
		break;
	}
}


void gm65_Tim_handler(struct gm65 *myQR, UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim){
	HAL_UART_AbortReceive_IT(huart);
	HAL_TIM_Base_Stop_IT(htim);
	__HAL_TIM_SET_COUNTER(htim, 0);
	switch (myQR->qrStatus){
	case QR_SETUP_OFF_R:
		HAL_UART_Transmit_IT(huart, qrSetupOFF, QR_LENGTH_COMMAND);
		myQR->qrStatus =  QR_SETUP_OFF_T;
		break;
	case QR_SCAN_RC:
		HAL_UART_Transmit_IT(huart, qrScan, QR_LENGTH_COMMAND);
		myQR->qrStatus =  QR_SCAN_T;
		break;
	case QR_SCAN_RH:
		HAL_UART_Transmit_IT(huart, qrScan, QR_LENGTH_COMMAND);
		myQR->qrStatus =  QR_SCAN_T;
		break;
	case QR_SCAN_RD:
		HAL_UART_Transmit_IT(huart, qrScan, QR_LENGTH_COMMAND);
		myQR->qrStatus =  QR_SCAN_T;
		break;
	case QR_SCAN_SETUP_OFF_R:
		HAL_UART_Transmit_IT(huart, qrSetupOFF, QR_LENGTH_COMMAND);
		myQR->qrStatus =  QR_SCAN_SETUP_OFF_T;
		break;
	default:
		break;
	}
}

void gm65_scan(struct gm65 *myQR, UART_HandleTypeDef *huart){
	if(myQR->qrStatus == QR_START){
		HAL_UART_AbortReceive_IT(huart);
		myQR->qrStatus=QR_SCAN_T;
		HAL_UART_Transmit_IT(huart, qrScan, QR_LENGTH_COMMAND);
	}
	else if(myQR->qrStatus == QR_RECEPTION_UNKNOWN) scanPending = 1;
}

void gm65_abort(struct gm65 *myQR, UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim){

	HAL_UART_Abort_IT(huart);
	HAL_TIM_Base_Stop_IT(htim);
	__HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
	__HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
	HAL_UART_Transmit_IT(huart, qrSetupOFF, QR_LENGTH_COMMAND);
	__HAL_TIM_SET_COUNTER(htim, 0);
	myQR->qrStatus =  QR_SETUP_OFF_T;
	myQR->qrDataReady=0;
}
