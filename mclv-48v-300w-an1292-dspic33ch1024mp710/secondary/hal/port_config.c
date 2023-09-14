/*******************************************************************************
  Input / Output Port COnfiguration Routine source File

  File Name:
    port_config.c

  Summary:
    This file includes subroutine for initializing GPIO pins as analog/digital,
    input or output etc. Also to PPS functionality to Remap-able input or output 
    pins

  Description:
    Definitions in the file are for dsPIC33CH512MP508 MC DIM plugged onto
    Motor Control Development board from Microchip
 
*******************************************************************************/
/*******************************************************************************
* Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.
*
* SOFTWARE LICENSE AGREEMENT:
* 
* Microchip Technology Incorporated ("Microchip") retains all ownership and
* intellectual property rights in the code accompanying this message and in all
* derivatives hereto.  You may use this code, and any derivatives created by
* any person or entity by or on your behalf, exclusively with Microchip's
* proprietary products.  Your acceptance and/or use of this code constitutes
* agreement to the terms and conditions of this notice.
*
* CODE ACCOMPANYING THIS MESSAGE IS SUPPLIED BY MICROCHIP "AS IS".  NO
* WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
* TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A
* PARTICULAR PURPOSE APPLY TO THIS CODE, ITS INTERACTION WITH MICROCHIP'S
* PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.
*
* YOU ACKNOWLEDGE AND AGREE THAT, IN NO EVENT, SHALL MICROCHIP BE LIABLE,
* WHETHER IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR BREACH OF
* STATUTORY DUTY),STRICT LIABILITY, INDEMNITY, CONTRIBUTION, OR OTHERWISE,
* FOR ANY INDIRECT, SPECIAL,PUNITIVE, EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL
* LOSS, DAMAGE, FOR COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE CODE,
* HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR
* THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT ALLOWABLE BY LAW,
* MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS CODE,
* SHALL NOT EXCEED THE PRICE YOU PAID DIRECTLY TO MICROCHIP SPECIFICALLY TO
* HAVE THIS CODE DEVELOPED.
*
* You agree that you are solely responsible for testing the code and
* determining its suitability.  Microchip has no obligation to modify, test,
* certify, or support the code.
*
*******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <xc.h>
#include "port_config.h"
#include "userparms.h"

// *****************************************************************************
/* Function:
    SetupGPIOPorts()

  Summary:
    Routine to set-up GPIO ports

  Description:
    Function initializes GPIO pins for input or output ports,analog/digital pins,
    remap the peripheral functions to desires RPx pins.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */

void SetupGPIOPorts(void)
{
    // Reset all PORTx register (all inputs)
    #ifdef TRISA
        TRISA = 0xFFFF;
        LATA  = 0x0000;
    #endif
    #ifdef ANSELA
        ANSELA = 0x0000;
    #endif

    #ifdef TRISB
        TRISB = 0xFFFF;
        LATB  = 0x0000;
    #endif
    #ifdef ANSELB
        ANSELB = 0x0000;
    #endif

    #ifdef TRISC
        TRISC = 0xFFFF;
        LATC  = 0x0000;
    #endif
    #ifdef ANSELC
        ANSELC = 0x0000;
    #endif

    #ifdef TRISD
        TRISD = 0xFFFF;
        LATD  = 0x0000;
    #endif
    #ifdef ANSELD
        ANSELD = 0x0000;
    #endif

    #ifdef TRISE
        TRISE = 0xFFFF;
        LATE  = 0x0000;
    #endif
    #ifdef ANSELE
        ANSELE = 0x0000;
    #endif

    MapGPIOHWFunction();

    return;
}
// *****************************************************************************
/* Function:
    Map_GPIO_HW_Function()

  Summary:
    Routine to setup GPIO pin used as input/output analog/digital etc

  Description:
    Function initializes GPIO pins as input or output port pins,analog/digital 
    pins,remap the peripheral functions to desires RPx pins.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */

void MapGPIOHWFunction(void)
{
    
    /* ANALOG SIGNALS */

    // Configure Port pins for Motor Current Sensing 

    
    // IA_EXT(S1AN0) : DIM #19
    ANSELAbits.ANSELA3 = 1;
    TRISAbits.TRISA3 = 1;    // PIN21 : S1AN0/S1CMP1A/S1PGA1P1/S1RA3

    // IB_EXT(S1AN1) : DIM #27
    ANSELAbits.ANSELA4 = 1;
    TRISAbits.TRISA4 = 1;    // PIN23: S1MCLR3/S1AN1/S1CMP2A/S1PGA2P1/S1PGA3P2/S1RA4
    
    //IBUS_EXT(S1AN13):DIM #35
    ANSELDbits.ANSELD10 = 1;
    TRISDbits.TRISD10 = 1;   //PIN38:S1AN13/S1CMP2B/S1RD10

    // Potentiometer #1 input - used as Speed Reference
    // POT1 : DIM #28
    TRISAbits.TRISA1 = 1;          // PIN18: S1AN15/S1RA1
    ANSELAbits.ANSELA1 = 1;
	
    /*DC Bus Voltage Signals : DIM:039*/
    ANSELCbits.ANSELC0 = 1;
    TRISCbits.TRISC0 = 1;         //PIN15: S1AN10/S1RP48/S1RC0
    
    /* Digital SIGNALS */   
    // DIGITAL INPUT/OUTPUT PINS

    // Inverter Control - PWM Outputs
    // PWM1L : DIM #03  RP59/RC11/S1RP59/S1PWM1L/S1RC11
    // PWM1H : DIM #01  RP58/RC10 S1RP58/S1PWM1H/S1RC10
    // PWM2L : DIM #07  RP53/RC5 S1RP53/S1PWM2L/S1RC5
    // PWM2H : DIM #05  RP52/RC4 S1RP52/S1PWM2H/S1RC4
    // PWM3L : DIM #04  RP67/RD3 S1RP67/S1PWM3L/S1RD3
    // PWM3H : DIM #02  RP68/RD4 S1RP68/S1PWM3H/S1RD4
    TRISCbits.TRISC10 = 0 ;          
    TRISCbits.TRISC11 = 0 ;         
    TRISCbits.TRISC4 = 0 ;          
    TRISCbits.TRISC5 = 0 ;           
    TRISDbits.TRISD4 = 0 ;          
    TRISDbits.TRISD3 = 0 ;         
    
    // Debug LEDs
    // LED1 : DIM #30
    TRISEbits.TRISE2 = 0;          // PIN:17 - RE2 S1RE2
    // LED2 : DIM #32
    TRISEbits.TRISE3 = 0;          // PIN:19 - RE3 S1RE3

    // Push button Switches
    // SW1 : DIM #34
    TRISEbits.TRISE4 = 1;            // PIN:22 RE4 S1RE4
    // SW2 : DIM #36
    TRISEbits.TRISE5 = 1;            // PIN:24 RE5 S1RE5  
	
	/** Diagnostic Interface for MCLV-48V-300W.
        Re-map UART Channels to the device pins connected to the following 
        PIM pins on the Motor Control Development Boards .
        UART_RX : DIM #54 (Input)
        UART_TX : DIM #52 (Output)   */
    _U1RXR = 47;
    _RP46R = 0b000001;
    
    
}
