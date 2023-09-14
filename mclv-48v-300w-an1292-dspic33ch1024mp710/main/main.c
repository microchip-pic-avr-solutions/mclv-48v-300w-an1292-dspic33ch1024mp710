#include <xc.h>
#include <libpic30.h>
#include "clock.h"
#include "port_config.h"

// *****************************************************************************
/* Function:
   main()

  Summary:
    main() function

  Description:
    program entry point, calls the system initialization function group 
    containing the buttons polling loop

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
int main(void) 
{
    /* Disable Watch Dog Timer */
    RCONbits.SWDTEN = 0;
    
    MSI1CONbits.MTSIRQ = 0;

    /* Routine to start SecondaryCore */
    _start_secondary();

    /* Clock Configuration */
    InitOscillator(); 
    SetupGPIOPorts();
    
    DIM_LED1 = 1;
    while(1)
    {
        
    }

}
