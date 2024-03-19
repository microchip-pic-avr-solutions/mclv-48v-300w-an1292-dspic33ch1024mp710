/*******************************************************************************
  Input / Output Port COnfiguration Routine source File

  File Name:
    port_config.c

  Summary:
    This file includes subroutine for initializing GPIO pins as analog/digital,
    input or output etc. Also to PPS functionality to Remap-able input or output 
    pins

  Description:
    Definitions in the file are for dsPIC33CH1024MP710 MC DIM plugged onto
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

    #ifdef TRISF
        TRISF = 0xFFFF;
        LATF  = 0x0000;
    #endif
    #ifdef ANSELF
        ANSELF = 0x0000;
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
    TRISAbits.TRISA3 = 1;    // PIN27 : S1AN0/S1CMP1A/S1PGA1P1/S1RA3

    // IB_EXT(S1AN1) : DIM #27
    ANSELAbits.ANSELA4 = 1;
    TRISAbits.TRISA4 = 1;    // PIN30: S1AN1/S1CMP2A/S1PGA2P1/S1PGA3P2/S1RA4
    
    //IBUS_EXT(S1AN12):DIM #35
    ANSELCbits.ANSELC7 = 1;
    TRISCbits.TRISC7 = 1;   //PIN50:S1AN12/S1ANC2/S1ANC3/S1CMP4D/S1CMP5D/S1CMP6D/S1RP55/S1RC7

    // Potentiometer #1 input - used as Speed Reference
    // POT1 : DIM #28
    TRISEbits.TRISE6 = 1;          // PIN45: S1AN19/S1PGA3N2/S1RP86/S1RE6
    ANSELEbits.ANSELE6 = 1;
	
    /*DC Bus Voltage Signals : DIM:039*/
    ANSELCbits.ANSELC0 = 1;
    TRISCbits.TRISC0 = 1;         //PIN17: S1AN10/S1CMP4A/S1RP48/S1RC0
    
    /* Digital SIGNALS */   
    // DIGITAL INPUT/OUTPUT PINS

    // Inverter Control - PWM Outputs
    // PWM1L : DIM #03  PIN83: S1RP59/S1PWML1/S1RC11
    // PWM1H : DIM #01  PIN82: S1RP58S1PWMH1/S1RC10
    // PWM2L : DIM #07  PIN81: S1RP53/S1PWML2/S1RC5
    // PWM2H : DIM #05  PIN79: S1RP52/S1PWMH2/S1RC4
    // PWM3L : DIM #04  PIN85: S1RP67/S1PWML3
    // PWM3H : DIM #02  PIN84: S1RP68/S1PWMH3/S1RD4
    TRISCbits.TRISC10 = 0 ;          
    TRISCbits.TRISC11 = 0 ;         
    TRISCbits.TRISC4 = 0 ;          
    TRISCbits.TRISC5 = 0 ;           
    TRISDbits.TRISD4 = 0 ;          
    TRISDbits.TRISD3 = 0 ;         
    
    // Debug LEDs
    // LED1 : DIM #30
    TRISEbits.TRISE15 = 0;          // PIN:99 S1RP95/S1RE15
    // LED2 : DIM #32
    TRISEbits.TRISE14 = 0;          // PIN:97 S1RP94/S1RPO54/S1RE14

    // Push button Switches
    // SW1 : DIM #34
    TRISBbits.TRISB14 = 1;            // PIN:1 S1RP46/S1RB14
    // SW2 : DIM #36
    TRISEbits.TRISE0 = 1;            // PIN:2 S1RP80/S1RE0  
	
	/** Diagnostic Interface for MCLV-48V-300W.
        Re-map UART Channels to the device pins connected to the following 
        DIM pins on the Motor Control Development Boards .
        UART_RX : DIM #54 (Input) - PIN:73 S1RP110/S1RF14
        UART_TX : DIM #52 (Output) - PIN:74 S1RP109/S1RF13   */
    _U1RXR = 109;
    _RP110R = 0b000001;
    
    
}
