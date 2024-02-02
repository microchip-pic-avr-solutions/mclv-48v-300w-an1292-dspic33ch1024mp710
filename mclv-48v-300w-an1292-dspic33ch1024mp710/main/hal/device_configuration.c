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

// DSPIC33CH1024MP712 Configuration Bit Settings

// 'C' source line config statements

// FSEC
#pragma config BWRP = OFF               // Boot Segment Write-Protect bit (Boot Segment may be written)
#pragma config BSS = DISABLED           // Boot Segment Code-Protect Level bits (No Protection (other than BWRP))
#pragma config BSEN = OFF               // Boot Segment Control bit (No Boot Segment)
#pragma config GWRP = OFF               // General Segment Write-Protect bit (General Segment may be written)
#pragma config GSS = DISABLED           // General Segment Code-Protect Level bits (No Protection (other than GWRP))
#pragma config CWRP = OFF               // Configuration Segment Write-Protect bit (Configuration Segment may be written)
#pragma config CSS = DISABLED           // Configuration Segment Code-Protect Level bits (No Protection (other than CWRP))
#pragma config AIVTDIS = OFF            // Alternate Interrupt Vector Table bit (Disabled AIVT)

// FBSLIM
#pragma config BSLIM = 0x1FFF           // Boot Segment Flash Page Address Limit bits (Enter Hexadecimal value)

// FSIGN

// FOSCSEL
#pragma config FNOSC = FRC              // Oscillator Source Selection (Internal Fast RC (FRC))
#pragma config IESO = OFF               // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)

// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Mode Select bits (Primary Oscillator disabled)
#pragma config OSCIOFNC = ON            // OSC2 Pin Function bit (OSC2 is general purpose digital I/O pin)
#pragma config FCKSM = CSECMD           // Clock Switching Mode bits (Clock switching is enabled,Fail-safe Clock Monitor is disabled)
#pragma config PLLKEN = PLLKEN_ON       // PLL Lock Status Control (PLL lock signal will be used to disable PLL clock output if lock is lost)
#pragma config XTCFG = G0               // XT Config (4-8 MHz crystals)
#pragma config XTBST = DISABLE          // XT Boost (Default kick-start)

// FWDT
#pragma config RWDTPS = PS1048576       // Run Mode Watchdog Timer Post Scaler select bits (1:1048576)
#pragma config RCLKSEL = LPRC           // Watchdog Timer Clock Select bits (Always use LPRC)
#pragma config WINDIS = ON              // Watchdog Timer Window Enable bit (Watchdog Timer operates in Non-Window mode)
#pragma config WDTWIN = WIN25           // Watchdog Timer Window Select bits (WDT Window is 25% of WDT period)
#pragma config SWDTPS = PS1048576       // Sleep Mode Watchdog Timer Post Scaler select bits (1:1048576)
#pragma config FWDTEN = ON_SW           // Watchdog Timer Enable bit (WDT controlled via SW, use WDTCON.ON bit)

// FPOR
#pragma config BISTDIS = DISABLED       // Memory BIST Feature Disable (mBIST on reset feature disabled)

// FICD
#pragma config ICS = PGD3               // ICD Communication Channel Select bits (Communicate on PGC3 and PGD3)
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)
#pragma config NOBTSWP = OFF            // BOOTSWP instruction disable bit (BOOTSWP instruction is disabled)

// FDMTIVTL
#pragma config DMTIVTL = 0x0            // Dead Man Timer Interval low word (Enter Hexadecimal value)

// FDMTIVTH
#pragma config DMTIVTH = 0x0            // Dead Man Timer Interval high word (Enter Hexadecimal value)

// FDMTCNTL
#pragma config DMTCNTL = 0x0            // Lower 16 bits of 32 bit DMT instruction count time-out value (0-0xFFFF) (Enter Hexadecimal value)

// FDMTCNTH
#pragma config DMTCNTH = 0x0            // Upper 16 bits of 32 bit DMT instruction count time-out value (0-0xFFFF) (Enter Hexadecimal value)

// FDMT
#pragma config DMTDIS = OFF             // Dead Man Timer Disable bit (Dead Man Timer is Disabled and can be enabled by software)

// FDEVOPT
#pragma config ALTI2C1 = OFF            // Alternate I2C1 Pin bit (I2C1 mapped to SDA1/SCL1 pins)
#pragma config ALTI2C2 = OFF            // Alternate I2C2 Pin bit (I2C2 mapped to SDA2/SCL2 pins)
#pragma config ALTI2C3 = OFF            // Alternate I2C3 Pin bit (I2C3 mapped to SDA3/SCL3 pins)
#pragma config SMBEN = STANDARD         // SM Bus Enable (Standard I2C input threshold operation)
#pragma config SPI2PIN = PPS            // SPI2 Pin Select bit (SPI2 uses I/O remap (PPS) pins)

// FALTREG
#pragma config CTXT1 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 1 bits (Not Assigned)
#pragma config CTXT2 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 2 bits (Not Assigned)
#pragma config CTXT3 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 3 bits (Not Assigned)
#pragma config CTXT4 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 4 bits (Not Assigned)

// FMBXM
#pragma config MBXM0 = S2M              // Mailbox 0 data direction (Mailbox register configured for Main core data read (Secondary core to Main core data transfer))
#pragma config MBXM1 = S2M              // Mailbox 1 data direction (Mailbox register configured for Main core data read (Secondary core to Main core data transfer))
#pragma config MBXM2 = S2M              // Mailbox 2 data direction (Mailbox register configured for Main core data read (Secondary core to Main core data transfer))
#pragma config MBXM3 = S2M              // Mailbox 3 data direction (Mailbox register configured for Main core data read (Secondary core to Main core data transfer))
#pragma config MBXM4 = S2M              // Mailbox 4 data direction (Mailbox register configured for Main core data read (Secondary core to Main core data transfer))
#pragma config MBXM5 = S2M              // Mailbox 5 data direction (Mailbox register configured for Main core data read (Secondary core to Main core data transfer))
#pragma config MBXM6 = S2M              // Mailbox 6 data direction (Mailbox register configured for Main core data read (Secondary core to Main core data transfer))
#pragma config MBXM7 = S2M              // Mailbox 7 data direction (Mailbox register configured for Main core data read (Secondary core to Main core data transfer))
#pragma config MBXM8 = S2M              // Mailbox 8 data direction (Mailbox register configured for Main core data read (Secondary core to Main core data transfer))
#pragma config MBXM9 = S2M              // Mailbox 9 data direction (Mailbox register configured for Main core data read (Secondary core to Main core data transfer))
#pragma config MBXM10 = S2M             // Mailbox 10 data direction (Mailbox register configured for Main core data read (Secondary core to Main core data transfer))
#pragma config MBXM11 = S2M             // Mailbox 11 data direction (Mailbox register configured for Main core data read (Secondary core to Main core data transfer))
#pragma config MBXM12 = S2M             // Mailbox 12 data direction (Mailbox register configured for Main core data read (Secondary core to Main core data transfer))
#pragma config MBXM13 = S2M             // Mailbox 13 data direction (Mailbox register configured for Main core data read (Secondary core to Main core data transfer))
#pragma config MBXM14 = S2M             // Mailbox 14 data direction (Mailbox register configured for Main core data read (Secondary core to Main core data transfer))
#pragma config MBXM15 = S2M             // Mailbox 15 data direction (Mailbox register configured for Main core data read (Secondary core to Main core data transfer))

// FMBXHS1
#pragma config MBXHSA = MBX15           // Mailbox handshake protocol block A register assignment (MSIxMBXD15 assigned to mailbox handshake protocol block A)
#pragma config MBXHSB = MBX15           // Mailbox handshake protocol block B register assignment (MSIxMBXD15 assigned to mailbox handshake protocol block B)
#pragma config MBXHSC = MBX15           // Mailbox handshake protocol block C register assignment (MSIxMBXD15 assigned to mailbox handshake protocol block C)
#pragma config MBXHSD = MBX15           // Mailbox handshake protocol block D register assignment (MSIxMBXD15 assigned to mailbox handshake protocol block D)

// FMBXHS2
#pragma config MBXHSE = MBX15           // Mailbox handshake protocol block E register assignment (MSIxMBXD15 assigned to mailbox handshake protocol block E)
#pragma config MBXHSF = MBX15           // Mailbox handshake protocol block F register assignment (MSIxMBXD15 assigned to mailbox handshake protocol block F)
#pragma config MBXHSG = MBX15           // Mailbox handshake protocol block G register assignment (MSIxMBXD15 assigned to mailbox handshake protocol block G)
#pragma config MBXHSH = MBX15           // Mailbox handshake protocol block H register assignment (MSIxMBXD15 assigned to mailbox handshake protocol block H)

// FMBXHSEN
#pragma config HSAEN = OFF              // Mailbox A data flow control protocol block enable (Mailbox data flow control handshake protocol block disabled.)
#pragma config HSBEN = OFF              // Mailbox B data flow control protocol block enable (Mailbox data flow control handshake protocol block disabled.)
#pragma config HSCEN = ON               // Mailbox C data flow control protocol block enable (Mailbox data flow control handshake protocol block enabled)
#pragma config HSDEN = ON               // Mailbox D data flow control protocol block enable (Mailbox data flow control handshake protocol block enabled)
#pragma config HSEEN = ON               // Mailbox E data flow control protocol block enable (Mailbox data flow control handshake protocol block enabled)
#pragma config HSFEN = ON               // Mailbox F data flow control protocol block enable (Mailbox data flow control handshake protocol block enabled)
#pragma config HSGEN = ON               // Mailbox G data flow control protocol block enable (Mailbox data flow control handshake protocol block enabled)
#pragma config HSHEN = ON               // Mailbox H data flow control protocol block enable (Mailbox data flow control handshake protocol block enabled)

// FCFGPRA0
#pragma config CPRA0 = MAIN             // Pin RA0 Ownership Bits (Main core core owns pin.)
#pragma config CPRA1 = MAIN             // Pin RA1 Ownership Bits (Main core core owns pin.)
#pragma config CPRA2 = MAIN             // Pin RA2 Ownership Bits (Main core core owns pin.)
#pragma config CPRA3 = SEC1             // Pin RA3 Ownership Bits (Main core core owns pin.)
#pragma config CPRA4 = SEC1             // Pin RA4 Ownership Bits (Main core core owns pin.)

// FCFGPRB0
#pragma config CPRB0 = MAIN             // Pin RB0 Ownership Bits (Main core core owns pin.)
#pragma config CPRB1 = MAIN             // Pin RB1 Ownership Bits (Main core core owns pin.)
#pragma config CPRB2 = SEC1             // Pin RB2 Ownership Bits (Main core core owns pin.)
#pragma config CPRB3 = MAIN             // Pin RB3 Ownership Bits (Main core core owns pin.)
#pragma config CPRB4 = MAIN             // Pin RB4 Ownership Bits (Main core core owns pin.)
#pragma config CPRB5 = MAIN             // Pin RB5 Ownership Bits (Main core core owns pin.)
#pragma config CPRB6 = MAIN             // Pin RB6 Ownership Bits (Main core core owns pin.)
#pragma config CPRB7 = SEC1             // Pin RB7 Ownership Bits (Main core core owns pin.)
#pragma config CPRB8 = SEC1             // Pin RB8 Ownership Bits (Main core core owns pin.)
#pragma config CPRB9 = SEC1             // Pin RB9 Ownership Bits (Main core core owns pin.)
#pragma config CPRB10 = MAIN            // Pin RB10 Ownership Bits (Main core core owns pin.)
#pragma config CPRB11 = MAIN            // Pin RB11 Ownership Bits (Main core core owns pin.)
#pragma config CPRB12 = MAIN            // Pin RB12 Ownership Bits (Main core core owns pin.)
#pragma config CPRB13 = MAIN            // Pin RB13 Ownership Bits (Main core core owns pin.)
#pragma config CPRB14 = SEC1            // Pin RB14 Ownership Bits (Main core core owns pin.)
#pragma config CPRB15 = MAIN            // Pin RB15 Ownership Bits (Main core core owns pin.)

// FCFGPRC0
#pragma config CPRC0 = SEC1             // Pin RC0 Ownership Bits (Main core core owns pin.)
#pragma config CPRC1 = MAIN             // Pin RC1 Ownership Bits (Main core core owns pin.)
#pragma config CPRC2 = MAIN             // Pin RC2 Ownership Bits (Main core core owns pin.)
#pragma config CPRC3 = MAIN             // Pin RC3 Ownership Bits (Main core core owns pin.)
#pragma config CPRC4 = SEC1             // Pin RC4 Ownership Bits (Main core core owns pin.)
#pragma config CPRC5 = SEC1             // Pin RC5 Ownership Bits (Main core core owns pin.)
#pragma config CPRC6 = MAIN             // Pin RC6 Ownership Bits (Main core core owns pin.)
#pragma config CPRC7 = SEC1             // Pin RC7 Ownership Bits (Main core core owns pin.)
#pragma config CPRC8 = MAIN             // Pin RC8 Ownership Bits (Main core core owns pin.)
#pragma config CPRC9 = MAIN             // Pin RC9 Ownership Bits (Main core core owns pin.)
#pragma config CPRC10 = SEC1            // Pin RC10 Ownership Bits (Main core core owns pin.)
#pragma config CPRC11 = SEC1            // Pin RC11 Ownership Bits (Main core core owns pin.)
#pragma config CPRC12 = MAIN            // Pin RC12 Ownership Bits (Main core core owns pin.)
#pragma config CPRC13 = MAIN            // Pin RC13 Ownership Bits (Main core core owns pin.)
#pragma config CPRC14 = MAIN            // Pin RC14 Ownership Bits (Main core core owns pin.)
#pragma config CPRC15 = MAIN            // Pin RC15 Ownership Bits (Main core core owns pin.)

// FCFGPRD0
#pragma config CPRD0 = MAIN             // Pin RD0 Ownership Bits (Main core core owns pin.)
#pragma config CPRD1 = MAIN             // Pin RD1 Ownership Bits (Main core core owns pin.)
#pragma config CPRD2 = MAIN             // Pin RD2 Ownership Bits (Main core core owns pin.)
#pragma config CPRD3 = SEC1             // Pin RD3 Ownership Bits (Main core core owns pin.)
#pragma config CPRD4 = SEC1             // Pin RD4 Ownership Bits (Main core core owns pin.)
#pragma config CPRD5 = MAIN             // Pin RD5 Ownership Bits (Main core core owns pin.)
#pragma config CPRD6 = MAIN             // Pin RD6 Ownership Bits (Main core core owns pin.)
#pragma config CPRD7 = MAIN             // Pin RD7 Ownership Bits (Main core core owns pin.)
#pragma config CPRD8 = MAIN             // Pin RD8 Ownership Bits (Main core core owns pin.)
#pragma config CPRD9 = SEC1             // Pin RD9 Ownership Bits (Main core core owns pin.)
#pragma config CPRD10 = SEC1            // Pin RD10 Ownership Bits (Main core core owns pin.)
#pragma config CPRD11 = MAIN            // Pin RD11 Ownership Bits (Main core core owns pin.)
#pragma config CPRD12 = MAIN            // Pin RD12 Ownership Bits (Main core core owns pin.)
#pragma config CPRD13 = MAIN            // Pin RD13 Ownership Bits (Main core core owns pin.)
#pragma config CPRD14 = MAIN            // Pin RD14 Ownership Bits (Main core core owns pin.)
#pragma config CPRD15 = MAIN            // Pin RD15 Ownership Bits (Main core core owns pin.)

// FCFGPRE0
#pragma config CPRE0 = SEC1             // Pin RE0 Ownership Bits (Main core core owns pin.)
#pragma config CPRE1 = MAIN             // Pin RE1 Ownership Bits (Main core core owns pin.)
#pragma config CPRE2 = MAIN             // Pin RE2 Ownership Bits (Main core core owns pin.)
#pragma config CPRE3 = MAIN             // Pin RE3 Ownership Bits (Main core core owns pin.)
#pragma config CPRE4 = MAIN             // Pin RE4 Ownership Bits (Main core core owns pin.)
#pragma config CPRE5 = MAIN             // Pin RE5 Ownership Bits (Main core core owns pin.)
#pragma config CPRE6 = SEC1             // Pin RE6 Ownership Bits (Main core core owns pin.)
#pragma config CPRE7 = MAIN             // Pin RE7 Ownership Bits (Main core core owns pin.)
#pragma config CPRE8 = MAIN             // Pin RE8 Ownership Bits (Main core core owns pin.)
#pragma config CPRE9 = MAIN             // Pin RE9 Ownership Bits (Main core core owns pin.)
#pragma config CPRE10 = MAIN            // Pin RE10 Ownership Bits (Main core core owns pin.)
#pragma config CPRE11 = MAIN            // Pin RE11 Ownership Bits (Main core core owns pin.)
#pragma config CPRE12 = MAIN            // Pin RE12 Ownership Bits (Main core core owns pin.)
#pragma config CPRE13 = MAIN            // Pin RE13 Ownership Bits (Main core core owns pin.)
#pragma config CPRE14 = SEC1              // Pin RE14 Ownership Bits (Main core core owns pin.)
#pragma config CPRE15 = SEC1            // Pin RE15 Ownership Bits (Main core core owns pin.)

// FCFGPRF0
#pragma config CPRF0 = MAIN             // Pin RF0 Ownership Bits (Main core core owns pin.)
#pragma config CPRF1 = MAIN             // Pin RF1 Ownership Bits (Main core core owns pin.)
#pragma config CPRF2 = MAIN             // Pin RF2 Ownership Bits (Main core core owns pin.)
#pragma config CPRF3 = MAIN             // Pin RF3 Ownership Bits (Main core core owns pin.)
#pragma config CPRF4 = MAIN             // Pin RF4 Ownership Bits (Main core core owns pin.)
#pragma config CPRF5 = MAIN             // Pin RF5 Ownership Bits (Main core core owns pin.)
#pragma config CPRF6 = MAIN             // Pin RF6 Ownership Bits (Main core core owns pin.)
#pragma config CPRF7 = MAIN             // Pin RF7 Ownership Bits (Main core core owns pin.)
#pragma config CPRF8 = MAIN             // Pin RF8 Ownership Bits (Main core core owns pin.)
#pragma config CPRF9 = MAIN             // Pin RF9 Ownership Bits (Main core core owns pin.)
#pragma config CPRF10 = MAIN            // Pin RF10 Ownership Bits (Main core core owns pin.)
#pragma config CPRF11 = MAIN            // Pin RF11 Ownership Bits (Main core core owns pin.)
#pragma config CPRF12 = MAIN            // Pin RF12 Ownership Bits (Main core core owns pin.)
#pragma config CPRF13 = SEC1            // Pin RF13 Ownership Bits (Main core core owns pin.)
#pragma config CPRF14 = SEC1            // Pin RF14 Ownership Bits (Main core core owns pin.)
#pragma config CPRF15 = MAIN            // Pin RF15 Ownership Bits (Main core core owns pin.)

// FBTSEQ
#pragma config BSEQ = 0xFFF             // Relative value defining which partition will be active after devie Reset; the partition containing a lower boot number will be active. (Enter Hexadecimal value)
#pragma config IBSEQ = 0xFFF            // The one's complement of BSEQ; must be calculated by the user and written during device programming. (Enter Hexadecimal value)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.