/***************************************************************************//**
  @file     board.h
  @brief    Board management
  @author   Micho
 ******************************************************************************/

#ifndef _BOARD_H_
#define _BOARD_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "gpio.h"


/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

/***** BOARD defines **********************************************************/

// On Board User LEDs

#define PIN_LED_RED     PORTNUM2PIN(PB, 22)	  // PTB22
#define PIN_LED_GREEN   PORTNUM2PIN(PE, 26)	  // PTE26
#define PIN_LED_BLUE    PORTNUM2PIN(PB, 21)	  // PTB21
#define LED_ACTIVE      LOW

// PINES DIGITALES

// KINETIS
#define PTA0			PORTNUM2PIN(PA, 0)
#define PTA1			PORTNUM2PIN(PA, 1)
#define PTA2			PORTNUM2PIN(PA, 2)
#define PTB2			PORTNUM2PIN(PB, 2)
#define PTB3			PORTNUM2PIN(PB, 3)
#define PTB9			PORTNUM2PIN(PB, 9)
#define PTB10			PORTNUM2PIN(PB, 10)
#define PTB11			PORTNUM2PIN(PB, 11)
#define PTB18			PORTNUM2PIN(PB, 18)
#define PTB19			PORTNUM2PIN(PB, 19)
#define PTB20			PORTNUM2PIN(PB, 20)
#define PTB23			PORTNUM2PIN(PB, 23)
#define PTC0			PORTNUM2PIN(PC, 0)
#define PTC1			PORTNUM2PIN(PC, 1)
#define PTC2			PORTNUM2PIN(PC, 2)
#define PTC3			PORTNUM2PIN(PC, 3)
#define PTC4			PORTNUM2PIN(PC, 4)
#define PTC5			PORTNUM2PIN(PC, 5)
#define PTC7			PORTNUM2PIN(PC, 7)
#define PTC8			PORTNUM2PIN(PC, 8)
#define PTC9			PORTNUM2PIN(PC, 9)
#define PTC10			PORTNUM2PIN(PC, 10)
#define PTC11			PORTNUM2PIN(PC, 11)
#define PTC16			PORTNUM2PIN(PC, 16)
#define PTC17			PORTNUM2PIN(PC, 17)
#define PTD0			PORTNUM2PIN(PD, 0)
#define PTD1			PORTNUM2PIN(PD, 1)
#define PTD2			PORTNUM2PIN(PD, 2)
#define PTD3			PORTNUM2PIN(PD, 3)
#define PTE24			PORTNUM2PIN(PE, 24)
#define PTE25			PORTNUM2PIN(PE, 25)
#define PTE26			PORTNUM2PIN(PE, 26)

/*******************************************************************************
 ******************************************************************************/

#endif // _BOARD_H_
