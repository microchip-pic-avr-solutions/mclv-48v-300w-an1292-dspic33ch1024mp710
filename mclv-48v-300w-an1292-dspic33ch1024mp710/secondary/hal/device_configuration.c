/*******************************************************************************
  Source File with Device Configuration bit settings

  File Name:
    device_configuration.c

  Summary:
    This file includes configuration fuse settings to configure device operation
    of Main and Secondary core 

  Description:
    Definitions in the file are for dsPIC33CH1024MP712 DIM plugged onto
    Motor Control Development Boards from Microchip
*******************************************************************************/
/*******************************************************************************
Copyright (c) 2016 released Microchip Technology Inc. All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip micro controller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sub-license terms in the accompanying license agreement).

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

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <xc.h>
#include <stdint.h>

// DSPIC33CH1024MP712S1 Configuration Bit Settings

// 'C' source line config statements

// FS1SEC
#pragma config BWRP = OFF               // Boot Segment Write-Protect bit (Boot Segment may be written)
#pragma config BSS = DISABLED           // Boot Segment Code-Protect Level bits (No Protection (other than BWRP))
#pragma config BSEN = OFF               // Boot Segment Control bit (No Boot Segment)
#pragma config GWRP = OFF               // General Segment Write-Protect bit (General Segment may be written)
#pragma config GSS = DISABLED           // General Segment Code-Protect Level bits (No Protection (other than GWRP))
#pragma config CWRP = OFF               // Configuration Segment Write-Protect bit (Configuration Segment may be written)
#pragma config CSS = DISABLED           // Configuration Segment Code-Protect Level bits (No Protection (other than CWRP))
#pragma config AIVTDIS = OFF            // Alternate Interrupt Vector Table bit (Disabled AIVT)

// FS1BSLIM
#pragma config BSLIM = 0x1FFF           // Boot Segment Flash Page Address Limit bits (Enter Hexadecimal value)

// FS1SIGN

// FS1OSCSEL
#pragma config FNOSC = FRC              // Oscillator Source Selection (Internal Fast RC (FRC) Oscillator with postscaler)
#pragma config IESO = OFF               // Two-speed Oscillator Start-up Enable bit (Start up device with FRC, then switch to user-selected oscillator source)

// FS1OSC
#pragma config OSCIOFNC = ON            // Secondary core OSC2 Pin Function bit (OSC2 is clock output)
#pragma config FCKSM = CSECMD           // Clock Switching Mode bits (Both Clock switching and Fail-safe Clock Monitor are disabled)
#pragma config PLLKEN = S1PLLKEN_ON     // S1PLLKEN (S1PLLKEN_ON)

// FS1WDT
#pragma config RWDTPS = PS1048576       // Run Mode Watchdog Timer Post Scaler select bits (1:1048576)
#pragma config RCLKSEL = LPRC           // Watchdog Timer Clock Select bits (Always use LPRC)
#pragma config WINDIS = ON              // Watchdog Timer Window Enable bit (Watchdog Timer operates in Non-Window mode)
#pragma config WDTWIN = WIN25           // Watchdog Timer Window Select bits (WDT Window is 25% of WDT period)
#pragma config SWDTPS = PS1048576       // Sleep Mode Watchdog Timer Post Scaler select bits (1:1048576)
#pragma config FWDTEN = ON              // Watchdog Timer Enable bit (WDT enabled in hardware)

// FS1POR
#pragma config BISTDIS = DISABLED       // Secondary core BIST on reset disable bit (Secondary core BIST on reset feature disabled)

// FS1ICD
#pragma config ICS = PGD3               // ICD Communication Channel Select bits (Communicate on PGC1 and PGD1)
#pragma config ISOLAT = ON              // Isolate the Secondary core core subsystem from the Main core subsystem during Debug (The Secondary core can operate (in debug mode) even if the SLVEN bit in the MSI is zero.)
#pragma config NOBTSWP = OFF            // BOOTSWP Instruction Enable/Disable bit (BOOTSWP instruction is disabled)

// FS1DEVOPT
#pragma config ALTI2C1 = OFF            // Alternate I2C1 Pin bit (I2C1 mapped to SDA1/SCL1 pins)
#pragma config ALTI2C2 = OFF            // Alternate I2C2 Pin bit (I2C2 mapped to SDA2/SCL2 pins)
#pragma config SMBEN = SMBUS            // SM Bus Enable (SMBus input threshold is enabled)
#pragma config SPI1PIN = PPS            // S1 SPI1 Pin Select bit (Secondary core SPI1 uses I/O remap (PPS) pins)
#pragma config SSRE = ON                // Secondary core Secondary core Reset Enable (Secondary core generated resets will reset the Secondary core Enable Bit in the MSI module)
#pragma config MSRE = ON                // Main Secondary core Reset Enable (The Main core software oriented RESET events (RESET Op-Code, Watchdog timeout, TRAP reset, illegalInstruction) will also cause the Secondary core subsystem to reset.)

// FALTREG
#pragma config CTXT1 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 1 bits (Not Assigned)
#pragma config CTXT2 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 2 bits (Not Assigned)
#pragma config CTXT3 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 3 bits (Not Assigned)
#pragma config CTXT4 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 4 bits (Not Assigned)

// FS1DMTIVTL
#pragma config DMTIVTL = 0xFFFF         // Secondary core Dead Man Timer Interval low word (Enter Hexadecimal value)

// FS1DMTIVTH
#pragma config DMTIVTH = 0xFFFF         // Secondary core Dead Man Timer Interval high word (Enter Hexadecimal value)

// FS1DMTCNTL
#pragma config DMTCNTL = 0xFFFF         // Secondary core DMT instruction count time-out value low word (Enter Hexadecimal value)

// FS1DMTCNTH
#pragma config DMTCNTH = 0xFFFF         // Secondary core DMT instruction count time-out value high word (Enter Hexadecimal value)

// FS1DMT
#pragma config DMTDIS = OFF             // Secondary core Dead Man Timer Disable bit (Secondary core Dead Man Timer is Disabled and can be enabled by software)

// FS1BTSEQ
#pragma config BSEQ = 0xFFF             // Relative value defining which partition will be active after devie Reset; the partition containing a lower boot number will be active. (Enter Hexadecimal value)
#pragma config IBSEQ = 0xFFF            // The one's complement of BSEQ; must be calculated by the user and written during device programming. (Enter Hexadecimal value)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.