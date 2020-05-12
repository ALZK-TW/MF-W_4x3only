/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//*****************************************************************************
// Development main.c for MSP430FR2633, MSP430FR2533, MSP430FR2632, and
// MSP430FR2532.
//
// This starter application initializes the CapTIvate touch library
// for the touch panel specified by CAPT_UserConfig.c/.h via a call to
// CAPT_appStart(), which initializes and calibrates all sensors in the
// application, and starts the CapTIvate interval timer.
//
// Then, the capacitive touch interface is driven by calling the CapTIvate
// application handler, CAPT_appHandler().  The application handler manages
// whether the user interface (UI) is running in full active scan mode, or
// in a low-power wake-on-proximity mode.
//
// The CapTIvate application handler will return true if proximity was
// detected on any of the sensors in the application, which is used here
// to control the state of LED2. LED1 is set while the background loop enters
// the handler, and is cleared when the background loop leaves the handler.
//
// \version 1.70.00.03
// Released on July 30, 2018
//
//*****************************************************************************

#include <msp430.h>                      // Generic MSP430 Device Include
#include "driverlib.h"                   // MSPWare Driver Library
#include "captivate.h"                   // CapTIvate Touch Software Library
#include "CAPT_App.h"                    // CapTIvate Application Code
#include "CAPT_BSP.h"                    // CapTIvate EVM Board Support Package

//*****************************************************************************
// Global Variables
//*****************************************************************************
//unsigned char *Pstring = "This is Test ..";
//unsigned char btnArray[12] = {'1','2','3','4','5','6','7','8','9','*','0','#'};
 uint8_t btnStr[6] = {'B','T','N','=',0x00,0x00};

extern tElement BTN00_E00;
extern tElement BTN00_E01;
extern tElement BTN00_E02;
extern tElement BTN00_E03;
extern tElement BTN00_E04;
extern tElement BTN00_E05;
extern tElement BTN00_E06;
extern tElement BTN00_E07;
extern tElement BTN00_E08;
extern tElement BTN00_E09;
extern tElement BTN00_E10;
extern tElement BTN00_E11;



//! \brief Flag
//!
uint8_t btnIdex;
bool bTouch = false;
//#ifndef RX_INPUT
#ifdef TX
uint8_t count = 0;
volatile bool bStart = false;
bool bLedStatus = false;
#endif
#ifdef CLOSE_SLEEP
#if (CAPT_WAKEONPROX_ENABLE==true)
volatile uint16_t g_ui16UISessionTimeoutCtr = 1;
#endif  // CAPT_WAKEONPROX_ENABLE
#endif
//*****************************************************************************
// Function Prototypes
//*****************************************************************************

//! \brief The event handler for the numeric keypad sensor.
//! \param pSensor is a pointer to the calling sensor.
void numericKeypadHandler(tSensor* pSensor);

//! \brief The event handler for the proximity and guard sensor.
//! \param pSensor is a pointer to the calling sensor.
extern void UART_transmitBuffer(const uint8_t *pBuffer, uint16_t ui16Length);
extern void UART_transmitBufferxx(const uint8_t *pBuffer, uint16_t ui16Length);

//*****************************************************************************
// Function Implementations
//*****************************************************************************
bool checkForValidTouch(void)
{
    bool result = true;
#ifdef TX
#ifndef RX_INPUT
#ifndef FLASH_ON
    bool turnoff = false;
#endif
#endif
#endif
    bTouch = false;
    if ((g_uiApp.pSensorList[0]->bSensorTouch == true))
    {
        bTouch = true;
    }

    //
    // If the touch status flag is set, also set LED1.
    //
    if(bTouch == true)
    {
#ifdef TX
//#ifndef RX_INPUT
#ifdef FLASH_ON
        P2OUT &= ~BIT6; // turn off tochled
        bLedStatus = false;
        bStart = true;
        count = 0;
#else
#ifndef RX_INPUT
        if(bLedStatus == true)
        {
            //if(BTN00_E09.bTouch || BTN00_E11.bTouch)
            if(BTN00_E09.bTouch)
            {
               // P1OUT &= ~BIT5; //rx
              //  P2OUT  &= ~BIT6; //tochled
                turnoff = true;
                bLedStatus = false;
                result = false;
            }
        }
        else
        {
            P1OUT |= BIT5; //rx
            P2OUT |= BIT6; //tochled
            bLedStatus = true;
            bStart = false;
        }
#else
        P2OUT |= BIT6; //tochled
#endif
#endif
//#else
//        P2OUT |= BIT6; //tochled
//#endif
#endif
        if(BTN00_E00.bTouch)
            btnIdex ='1';
        if(BTN00_E01.bTouch)
            btnIdex ='2';
        if(BTN00_E02.bTouch)
            btnIdex ='3';
        if(BTN00_E03.bTouch)
            btnIdex ='4';
        if(BTN00_E04.bTouch)
            btnIdex ='5';
        if(BTN00_E05.bTouch)
            btnIdex ='6';
        if(BTN00_E06.bTouch)
            btnIdex ='7';
        if(BTN00_E07.bTouch)
            btnIdex ='8';
        if(BTN00_E08.bTouch)
            btnIdex ='9';
        if(BTN00_E10.bTouch)
            btnIdex ='0';
        if(BTN00_E09.bTouch)
            btnIdex ='*';
        if(BTN00_E11.bTouch)
            btnIdex ='#';

        btnStr[4] = btnIdex;
        UART_transmitBufferxx(&btnStr[4], 1);
    }
    else
    {
        bTouch = false;
#ifdef TX
#ifndef RX_INPUT
#ifndef FLASH_ON
        result = false;
#endif
#endif
#endif
    }
#ifdef TX
#ifndef RX_INPUT
#ifndef FLASH_ON
    if(turnoff)
    {
        P2OUT  &= ~BIT6; //tochled
        P1OUT &= ~BIT5; //rx
    }
#endif
#endif
#endif
    return result;
}

void Func_Callback_init(void)
{
    //
    // Open the I2C Master driver, which the DRV26x driver will use to
    // communicate with the DRV2605L haptic driver IC via I2C.
    // Enable the haptic driver by setting P1.0, which is connected
    // to the DRV2605L ENABLE pin.  Then, load the configuration for the
    // actuator, run an auto-calibration routine, set up for internal trigger
    // mode, and select the linear resonant actuator (LRA) effect library.
    //

    //
    // Associate the capacitive touch callback handlers with sensors.
    //
    MAP_CAPT_registerCallback(
            &BTN00,
            &numericKeypadHandler
        );
}

void numericKeypadHandler(tSensor* pSensor)
{
   // bool result;
    //
    // If the sensor has a new touch, fire a "strong click" effect.
    //
    if((pSensor->bSensorTouch == true) && (pSensor->bSensorPrevTouch == false))
    {
        if(bTouch == false)
            checkForValidTouch();
        /*
        if(result)
        {
            P2OUT |= BIT6; //tochled
            bLedStatus = true;
            bStart = false;
        }
        */
    }
    else if((pSensor->bSensorTouch == false) && (pSensor->bSensorPrevTouch ==true))
    {
        bTouch=false;
#ifdef TX
#ifndef RX_INPUT
#ifndef FLASH_ON
        count = 0;
        bStart = true;
#endif
#endif
#endif
    }

}


void main(void)
{
	//
	// Initialize the MCU
	// BSP_configureMCU() sets up the device IO and clocking
	// The global interrupt enable is set to allow peripherals
	// to wake the MCU.
	//
	WDTCTL = WDTPW | WDTHOLD;
	BSP_configureMCU();
	__bis_SR_register(GIE);
	Func_Callback_init();
	//P2OUT |= BIT6; //tochled

	//
	// Start the CapTIvate application
	//
	CAPT_appStart();

	//
	// Background Loop
	//
	while(1)
	{
		//
		// Run the captivate application handler.
		// Set LED1 while the app handler is running,
		// and set LED2 if proximity is detected
		// on any sensor.
		//
		CAPT_appHandler();
#ifdef TX
//#ifndef RX_INPUT
#ifdef FLASH_ON
		if(bStart && (bLedStatus == false))
		{
		    if(count > 2)
		    {
		        P2OUT  |= BIT6; //tochled
		        bLedStatus = true;
		        bStart = false;
		    }
		    else
		    {
		        count++;
		    }
		}
#else
#ifndef RX_INPUT
		if(bStart)
		{
		    if(count > 120)
		    {
		        P1OUT &= ~BIT5; //rx
		        P2OUT  &= ~BIT6; //tochled
		        bLedStatus = false;
		    }
		    else
		    {
		        count ++;
		    }
		}
#endif
#endif
//#endif
#endif
		//
		// This is a great place to add in any 
		// background application code.
		//
		//__no_operation();

		//
		// End of background loop iteration
		// Go to sleep if there is nothing left to do
		//
		CAPT_appSleep();
		
	} // End background loop
} // End main()
#ifdef TX
#ifdef RX_INPUT
// Port 1 interrupt service routine
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
    /*
    if((P1IN & BIT0) == 0)
    {
        if((P1IN & BIT0) == 0)
        {
       //     SW1Flag = 1;
        }
    }
    */
    bStart = false;
    P2OUT  &= ~BIT6; //tochled
    P1IFG &= ~BIT5;   // Clear P1.0 IFG
#ifdef CLOSE_SLEEP
    if(g_ui16UISessionTimeoutCtr)
        g_ui16UISessionTimeoutCtr = 1;
#endif
}
#endif
#endif
