/******************************************************************************

 @file  board_key.c

 @brief This file contains the interface to the SRF06EB Key Service.

 Group: CMCU, SCS
 Target Device: CC2640R2

 ******************************************************************************
 
 Copyright (c) 2014-2017, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: simplelink_cc2640r2_sdk_1_35_00_33
 Release Date: 2017-05-02 17:08:52
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <stdbool.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include <ti/drivers/pin/PINCC26XX.h>

#ifdef USE_ICALL
#include <icall.h>
#endif
   
#include <inc/hw_ints.h>

#include "util.h"
#include "board.h"
#include "shutdn_button.h"

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void Board_keyChangeHandler(UArg a0);
static void Board_keyCallback(PIN_Handle hPin, PIN_Id pinId);


/*******************************************************************************
 * EXTERNAL VARIABLES
 */

// PIN configuration structure to set all KEY pins as inputs with pullups enabled
PIN_Config ButtonTable[] = {
    Board_BTN0          | PIN_GPIO_OUTPUT_DIS  | PIN_INPUT_EN  |  PIN_PULLUP, 
    //Board_BTN0          | PIN_GPIO_OUTPUT_DIS  | PIN_INPUT_EN  | PIN_PULLUP  | PINCC26XX_WAKEUP_NEGEDGE   , 
    PIN_TERMINATE
};

/*********************************************************************
 * LOCAL VARIABLES
 */

// Value of keys Pressed
static uint8_t keysPressed;

/* Clock used for debounce logic */
Clock_Struct keyChangeClock;
Clock_Handle hkeyChangeClock;

// Pointer to application callback
keysPressedCB_t appKeyChangeHandler = NULL;

/* Pin driver handles */
PIN_Handle hKeyPins;

/* LED pin state */
PIN_State LedPinState;

/* Button pin state */
PIN_State keyPins;

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
/*********************************************************************
 * @fn      Board_initKeys
 *
 * @brief   Enable interrupts for keys on GPIOs.
 *
 * @param   appKeyCB - application key pressed callback
 *
 * @return  none
 */
void Board_initKeys(keysPressedCB_t appKeyCB)
{
  /* Setup button pins with ISR */
  hKeyPins = PIN_open(&keyPins, ButtonTable);
  PIN_registerIntCb(hKeyPins, Board_keyCallback);
  PIN_setConfig(hKeyPins, PIN_BM_IRQ, Board_BTN0 | PIN_IRQ_NEGEDGE);
  //PIN_setConfig(hKeyPins, PIN_BM_IRQ, Board_BTN0 | PIN_IRQ_BOTHEDGES);
  //add
  ///PIN_setConfig(hKeyPins, PINCC26XX_BM_WAKEUP, Board_BTN0 | PINCC26XX_WAKEUP_NEGEDGE);
  
  /* Construct clock for debounce */
  Util_constructClock(&keyChangeClock, Board_keyChangeHandler,
                      KEY_DEBOUNCE_TIMEOUT, 0, false, 0);
  hkeyChangeClock = Clock_handle(&keyChangeClock);
  
  // Set the application callback
  appKeyChangeHandler = appKeyCB;
}

/*********************************************************************
 * @fn      Board_keyCallback
 *
 * @brief   Interrupt handler for Keys
 *
 * @param   none
 *
 * @return  none
 */
static void Board_keyCallback(PIN_Handle hPin, PIN_Id pinId)
{  
  keysPressed = 0;
  
  if ( PIN_getInputValue(Board_BTN0) == 0 )
  {
    keysPressed |= KEY_BTN0;
  }
  
  Clock_start(hkeyChangeClock);
}

/*********************************************************************
 * @fn      Board_keyChangeHandler
 *
 * @brief   Handler for key change
 *
 * @param   UArg a0 - ignored
 *
 * @return  none
 */
static void Board_keyChangeHandler(UArg a0)
{
  
  if (appKeyChangeHandler != NULL)
  {
     PIN_setConfig(hKeyPins, PINCC26XX_BM_WAKEUP, Board_BTN0 | PINCC26XX_WAKEUP_NEGEDGE);
     (*appKeyChangeHandler)(keysPressed);
  }
}
/*********************************************************************
*********************************************************************/
