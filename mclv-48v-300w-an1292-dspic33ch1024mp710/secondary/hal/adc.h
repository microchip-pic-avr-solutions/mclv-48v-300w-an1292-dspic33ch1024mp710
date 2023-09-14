/*******************************************************************************
  SecondaryCore Shared ADC Configuration Routine Header File

  File Name:
    adc.h

  Summary:
    This header file lists ADC Configuration related functions and definitions

  Description:
    Definitions in the file are for dsPIC33CH512MP508 MC PIM plugged onto
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
#ifndef _ADC_H
#define _ADC_H

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
#include "userparms.h"
// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************
// ADC MODULE Related Definitions
#define ADCBUF_INV_A_IPHASE1    (int16_t)(-ADCBUF0)   
#define ADCBUF_INV_A_IPHASE2    (int16_t)(-ADCBUF1)
#define ADCBUF_INV_A_IBUS       ADCBUF13        
        
#define ADCBUF_SPEED_REF_A      ADCBUF15
#define ADCBUF_VBUS_A           ADCBUF10
#ifdef SINGLE_SHUNT
#define EnableADCInterrupt()   _ADCAN13IE = 1
#define DisableADCInterrupt()  _ADCAN13IE = 0
#define ClearADCIF()           _ADCAN13IF = 0
#define ClearADCIF_ReadADCBUF() ADCBUF13 
        
#define _ADCInterrupt _ADCAN13Interrupt        
#else
#define EnableADCInterrupt()   _ADCAN15IE = 1
#define DisableADCInterrupt()  _ADCAN15IE = 0
#define ClearADCIF()           _ADCAN15IF = 0
#define ClearADCIF_ReadADCBUF() ADCBUF15

#define _ADCInterrupt _ADCAN15Interrupt
#endif
        
// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines
// *****************************************************************************
// *****************************************************************************
void InitializeADCs(void);

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif      // end of ADC_H

