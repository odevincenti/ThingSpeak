/***************************************************************************//**
  @file     App.c
  @brief    Application functions
  @author   Nicolás Magliola
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdint.h>
#include "board.h"
#include "gpio.h"
#include "uart.h"
#include "timer.h"
#include "led.h"
#include "fifo.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/
#define UART_ID		0
#define HEADER_LEN	4
#define DATA_OK		0x81
// #define DATA_OK		'1'
#define DATA_FAIL	0xC1
#define KEEPALIVE_OK	0x82
// #define KEEPALIVE_OK	'2'

#define KEEPALIVE_MS	1000
#define DATA_MS			15500

#define PISO1_IDX		6
#define PISO2_IDX		8
#define PISO3_IDX		10

enum {IS_OK, IS_FAIL, IS_ERR, IS_DATA, IS_KEEPALIVE};
enum {IDLE, KEEPALIVE_SEND, KEEPALIVE_CHECK, DATA_SEND, DATA_CHECK};

#define GET_MSBYTE(x)	((uint8_t) ((x) >> 8) )
#define GET_LSBYTE(x)	((uint8_t) (x) )
#define LSB_MASK		0x0F

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/
void Data_IRQ();
void KeepAlive_IRQ();
void SendData();
uint8_t IsDataOk();
void KeepAlive();
uint8_t IsKeepAliveOk();

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
*******************************************************************************/
static uint8_t keepalive_msg[] = {0xAA, 0x55, 0xC3, 0x3C, 0x01, 0x02};
// static uint8_t keepalive_msg[] = {'A', '5', '3', '5', '1', '2'};
static uint8_t data_msg[] = {0xAA, 0x55, 0xC3, 0x3C, 0x07, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// static uint8_t data_msg[] = {'A', '5', '3', '5', '7', '1', '0', '0', '0', '0', '0', '0'};
static uint16_t piso1 = 10;
static uint16_t piso2 = 20;
static uint16_t piso3 = 30;

static tim_id_t keepalive_timer;
static tim_id_t data_timer;

static uint8_t state = IDLE;

/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

/* Función que se llama 1 vez, al comienzo del programa */
void App_Init (void)
{
    // Inicializo UART
	uart_cfg_t config;
	config.baud_rate = 1200;
	uartInit(0, config);

	// Inicializo LED
	LedInit();
	rgb_t color = {.red = true, .green = false, .blue = true};
	LedRGB(color);

	// Inicializo timers
	timerInit();
	keepalive_timer = timerGetId();
	data_timer = timerGetId();
	timerStart(keepalive_timer, TIMER_MS2TICKS(KEEPALIVE_MS), TIM_MODE_PERIODIC, &KeepAlive_IRQ);
	timerStart(data_timer, TIMER_MS2TICKS(DATA_MS), TIM_MODE_PERIODIC, &Data_IRQ);

	state = DATA_SEND;

}

/* Función que se llama constantemente en un ciclo infinito */
void App_Run (void)
{
	switch (state){
		case KEEPALIVE_SEND:
			KeepAlive();
			state = state == KEEPALIVE_SEND ? KEEPALIVE_CHECK : state;
			break;
		case KEEPALIVE_CHECK:
			if (uartIsTxMsgComplete(UART_ID)){
				if (IsKeepAliveOk() == IS_OK){
					LedOn();
				} else {
					LedOff();
				}
				state = state == KEEPALIVE_CHECK ? IDLE : state;
			}
			break;
		case DATA_SEND:
			SendData();
			state = DATA_CHECK;
			break;
		case DATA_CHECK:
			if (uartIsTxMsgComplete(UART_ID)){
				if (IsDataOk() != IS_OK){
					//state = DATA_SEND;
				} else {
					rgb_t color = {.red = false, .green = true, .blue = false};
					LedRGB(color);
				}
				state = state == DATA_CHECK ? IDLE : state;
			}
			break;
		default:
			break;
	}	
}


/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

void SendData(){

	data_msg[PISO1_IDX] = GET_LSBYTE(piso1);
	data_msg[PISO1_IDX + 1] = GET_MSBYTE(piso1);
	data_msg[PISO2_IDX] = GET_LSBYTE(piso2);
	data_msg[PISO2_IDX + 1] = GET_MSBYTE(piso2);
	data_msg[PISO3_IDX] = GET_LSBYTE(piso3);
	data_msg[PISO3_IDX + 1] = GET_MSBYTE(piso3);
	uartWriteMsg(UART_ID, (uint8_t*) &data_msg[0], sizeof(data_msg));

}

uint8_t IsDataOk(){
	
	uint8_t r = false;
	uint8_t header_rx[HEADER_LEN];
	uint8_t length;
	uint8_t answer;

	uartReadMsg(UART_ID, (uint8_t*) &header_rx[0], 4);
	// uartReadMsg(UART_ID, &length, 1);
	uartReadMsg(UART_ID, &answer, 1);

	if (answer == DATA_OK){
		r = IS_OK;
	} else if (answer == DATA_FAIL){
		r = IS_FAIL;
	} else if (answer == KEEPALIVE_OK){
		r = IS_KEEPALIVE;
	} else {
		r = IS_ERR;
	}

	return r;
}

void KeepAlive_IRQ(){
	state = state == IDLE ? KEEPALIVE_SEND : state;
}

void Data_IRQ(){
	state = DATA_SEND;
}

void KeepAlive(){
	uartWriteMsg(UART_ID, (uint8_t*) &keepalive_msg[0], sizeof(keepalive_msg));
}

uint8_t IsKeepAliveOk(){
	
	uint8_t r = false;
	uint8_t header_rx[HEADER_LEN];
	//uint8_t length;
	uint8_t answer;

	uartReadMsg(UART_ID, (uint8_t*) &header_rx[0], 4);
	//uartReadMsg(UART_ID, &length, 1);
	uartReadMsg(UART_ID, &answer, 1);

	if (answer == KEEPALIVE_OK){
		r = IS_OK;
	} else if (answer == DATA_OK){
		r = IS_DATA;
	} else {
		r = IS_ERR;
	}

	return r;
}

/*******************************************************************************
 ******************************************************************************/
