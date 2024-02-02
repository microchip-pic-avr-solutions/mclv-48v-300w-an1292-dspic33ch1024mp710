/*******************************************************************************
  Oscillator Configuration Routine Header File

  File Name:
    clock.h

  Summary:
    This header file lists Clock Configuration related functions and definitions

  Description:
    Definitions in the file are for dsPIC33CH1024MP710 DIM plugged onto
    Motor Control Development board from Microchip

*******************************************************************************/
/*******************************************************************************
Copyright (c) 2016 released Microchip Technology Inc. All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
#ifndef _CLOCK_H
#define _CLOCK_H

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif
// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <xc.h>
#include <stdint.h>
// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************
// OSCILLATOR Related Definitions
// Instruction cycle frequency (Hz)
// OSCILLATOR Related Definitions
// Oscillator frequency (Hz) - 170,000,000 Hz
#define FOSC                    170000000UL
// Oscillator frequency (MHz) - 170MHz
#define FOSC_MHZ                170U     
// Instruction cycle frequency (Hz) - 85,000,000 Hz
#define FCY                     85000000UL
// Instruction cycle frequency (MHz) - 85 MHz
#define FCY_MHZ                 85U      
// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines
// *****************************************************************************
// *****************************************************************************
void InitOscillator(void);
void EnableREFCLKOutput(uint16_t);        
        
#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif      // end of CLOCK_H

