![image](images/microchip.jpg) 
## Sensorless FOC using PLL Estimator for PMSM : MCLV-48V-300W and dsPIC33CH1024MP710 Motor Control DIM


## 1. INTRODUCTION
This document describes the setup requirements for driving a Permanent Magnet Synchronous Motor (PMSM) using Sensorless Field Oriented Control (FOC) and PLL Estimator Algorithms on the hardware platform MCLV-48V-300W Inverter Board and dsPIC33CH1024MP710 Motor Control Dual In-line Module (DIM).

For details about PLL estimator refer to Microchip application note [AN1292](https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ApplicationNotes/ApplicationNotes/01292A.pdf) “Sensorless Field Oriented Control (FOC) for a Permanent Magnet Synchronous Motor (PMSM) Using a PLL Estimator and Field Weakening (FW)”

</br>

## 2. SUGGESTED DEMONSTRATION REQUIREMENTS

### 2.1 Motor Control Application Firmware Required for the Demonstration

To clone or download this application firmware on GitHub, 
- Navigate to the [main page of this repository](https://github.com/microchip-pic-avr-solutions/mclv-48v-300w-an1292-dspic33ch1024mp710) and 
- On the tab **<> Code**, above the list of files in the right-hand corner, click Code, then from the menu, click **Download ZIP** or copy the repository URL to **clone.**
> **Note:**</br>
>In this document, hereinafter this firmware package is referred as **firmware.**
### 2.2 Software Tools Used for Testing the firmware

- MPLAB® X IDE **v6.20** 
- DFP: **dsPIC33CH-MP_DFP v1.15.378**
- MPLAB® XC-DSC Compiler **v3.00**
- MPLAB® X IDE Plugin: **X2C-Scope v1.6.6** 
> **Note:** </br>
>The software used for testing the firmware prior to release is listed above. It is recommended to use the version listed above or later versions for building the firmware.

### 2.3 Hardware Tools Required for the Demonstration
- MCLV-48V-300W Inverter Board [(EV18H47A)](https://www.microchip.com/en-us/development-tool/ev18h47a)
- dsPIC33CH1024MP710 Motor Control Dual In-line Module (EV44A73A)
- 24V Power Supply [(AC002013)](https://www.microchipdirect.com/dev-tools/AC002013)
- 24V 3-Phase Brushless DC Motor [(AC300020)](https://www.microchip.com/en-us/development-tool/AC300020)

> **Note:** </br>
> All items listed under the section Hardware Tools Required for the Demonstration are available at [microchip DIRECT](https://www.microchipdirect.com/)

</br>

## 3. HARDWARE SETUP
This section describes hardware setup required for the demonstration.

1. Motor currents are amplified on the MCLV-48V-300W Inverter Board. The firmware and DIM are configured to sample and convert external amplifier outputs to measure the motor currents needed to implement FOC. 

2. Insert the **dsPIC33CH1024MP710 Motor Control DIM** into the DIM Interface **connector J8** on the MCLV-48V-300W Inverter Board. Make sure the DIM is placed correctly and oriented before going ahead.
-----**Image Change**----
     <p align="left" >
     <img  src="images/dimconnected.PNG"></p>


3. Connect the 3-phase wires from the motor to PHC, PHB, and PHA of the **connector J4**(no specific order), provided on the MCLV-48V-300W Inverter Board.

     <p align="left" >
      <img  src="images/motorconnection.png"></p>

4. Plug the 24V power supply to **connector J1** on the MCLV-48V-300W Inverter Board. Alternatively, the Inverter Board can also be powered through connector J3.
      <p align="left">
      <img  src="images/mclvpower.png"></p>
 

 5. The board has an onboard programmer **PICkit™ On Board (PKoBv4)** , which can be used for programming or debugging the microcontroller or dsPIC DSC on the DIM. To use the onboard programmer, connect a micro-USB cable between the Host PC and **connector J16** on the MCLV-48V-300W Inverter Board.
      <p align="left">
     <img  src="images/mclvpkob4.png"></p>

 6. Alternatively, connect the Microchip programmer/debugger MPLAB® PICkit™ 4 In-Circuit Debugger between the Host PC used for programming the device and the **ICSP header J9** on the MCLV-48V-300W Inverter Board (as shown). Ensure that PICkit 4 is oriented correctly before proceeding.
      <p align="left">
       <img  src="images/mclvprogramming.PNG"></p>
 
 </br>

## 4. SOFTWARE SETUP AND RUN
### 4.1 Setup: MPLAB X IDE and MPLAB XC-DSC Compiler
Install **MPLAB X IDE** and **MPLAB XC-DSC Compiler** versions that support the device **dsPIC33CH1024MP710** and **PKOBv4.** The MPLAB X IDE, MPLAB XC-DSC Compiler, and X2C-Scope plug-in used for testing the firmware are mentioned in the [Motor Control Application Firmware Required for the Demonstration](#21-motor-control-application-firmware-required-for-the-demonstration) section. 

To get help on  

- MPLAB X IDE installation, refer [link](https://microchipdeveloper.com/xwiki/bin/view/software-tools/x/install-guide/)
- MPLAB XC-DSC Compiler installation steps, refer [link](https://microchipdeveloper.com/xwiki/bin/view/software-tools/xc-dsc/install/)

If MPLAB IDE v8 or earlier is already installed on your computer, then run the MPLAB driver switcher (Installed when MPLAB®X IDE is installed) to switch from MPLAB IDE v8 drivers to MPLAB X IDE drivers. If you have Windows 8 or 10, you must run the MPLAB driver switcher in **Administrator Mode**. To run the Device Driver Switcher GUI application as administrator, right-click on the executable (or desktop icon) and select **Run as Administrator**. For more details, refer to the MPLAB X IDE help topic **“Before You Begin: Install the USB Device Drivers (For Hardware Tools): USB Driver Installation for Windows Operating Systems.”**

### 4.2 Setup: X2C-SCOPE
X2C-Scope is an MPLAB X IDE plugin that allows developers to interact with an application while it runs. X2C-Scope enables you to read, write, and plot global variables (for motor control) in real-time. It communicates with the target using the UART. To use X2C-Scope, the plugin must be installed. To set up and use X2C-Scope, refer to the instructions provided on the [web page](https://x2cscope.github.io/docs/MPLABX_Plugin.html).

## 5.  BASIC DEMONSTRATION
### 5.1 Firmware Description
The firmware version needed for the demonstration is mentioned in the section [Motor Control Application Firmware Required for the Demonstration](#21-motor-control-application-firmware-required-for-the-demonstration) section. This firmware is implemented to work on Microchip’s dual-core 16-bit Digital signal controller (dsPIC® DSC) **dsPIC33CH1024MP710**. There are two independent dsPIC DSC cores called **Main Core** and **Secondary Core** in the device. For more information, see the **dsPIC33CH1024MP710 Family datasheet (DS50003069C)**.

In MPLAB X IDE, the code for two cores is developed as separate projects with the follow-ing device selections.
- Device selection in Main Project (code for **Main Core**) is **dsPIC33CH1024MP710**
- Device selection in Secondary Project (code for **Secondary Core**) is **dsPIC33CH1024MP710S1**

Hence the firmware used in this demonstration consists of two MPLAB X projects, **main.X** (**Main Project**) and **pmsm.X** (**Secondary Project**). User must program each project independently before running the demo application.The detailed steps are outline in the section [5.2 Basic Demonstration](#52-basic-demonstration)

In this this application demo firmware, the project main.X for the Main core configures the secondary core pins and enable the Secondary Core. The second project pmsm.X for the Secondary autonomously run the Motor Control Demo application after programming. This Motor Control Demo application uses a push button to start or stop the motor and a potentiometer to vary the speed of the motor.

The function of the Main Core is defined by the Main Project **main.X**, they are:
- To set device configuration bits applicable for both Main and Secondary cores (Configuration bits for Main and Secondary cores exist in Main core). Note that the configuration bits decide the I/O port ownership between Main Core and Secondary Core. 
- Configure Main Core Oscillator Subsystem to generate clocks needed to operate Core and its peripherals. In the firmware, Main is configured to run at 85MHz.
- The main core enables the Secondary core by invoking <code>_start_secondary()</code> rountine in the XC-DSC library (<code>libpic30.h</code>).

The function of the Secondary Core (as defined in the Secondary Project **pmsm.X**) is:
- To configure Secondary Core Oscillator Subsystem to generate clocks needed to operate core and its peripherals. In the firmware, the Secondary core is configured to operate at 75MHz.
- To configure I/O ports and Secondary Core peripherals (such as PWM Generators PG1, PG2 and PG3, ADC Cores, UART1) required to function the firmware.
- To execute the Motor Control Demo Application based on the Microchip Application note AN1292. 

The firmware directory structure is shown below: **--Image Change--**
<p align="left">
<img  src="images/firmwarestructure.png"></p>



This Motor Control Demo Application configures and uses peripherals like PWM, ADC, UART, etc. For more details, refer to Microchip Application note **AN1292, “Sensorless Field Oriented Control (FOC) for a Permanent Magnet Synchronous Motor (PMSM) Using a PLL Estimator and Field Weakening (FW),”** available on the [Microchip website.]((https://www.microchip.com/).)

> **Note:**</br>
> The project may not build correctly in Windows OS if the Maximum path length of any source file in the project is more than 260 characters. In case the absolute path exceeds or nears the maximum length, do any (or both) of the following:
> - Shorten the directory name containing the firmware used in this demonstration. If you renamed the directory, consider the new name while reading the instructions provided in the upcoming sections of the document.
> - Place firmware in a location such that the total path length of each file included in the projects does not exceed the Maximum Path length specified. </br>
> Refer to MPLAB X IDE help topic **“Path, File, and Folder Name Restrictions”** for details. 

### 5.2 Basic Demonstration
Follow the below instructions, step by step, to set up and run the motor control demo application:

1. Start **MPLAB X IDE** and open the main project **main.X (File > Open Project)** with device selection **dsPIC33CH1024MP710.**  
    <p align="left">
       <img  src="images/idedeviceselection.png"></p>
  

2. Set the project **main.X** as the main project by right clicking on the project name and selecting **Set as Main Project** as shown. The project **main.X** will then appear in **bold.**
    <p align="left">
     <img  src="images/ideprojectsetup.png"></p>

3. Right-click on the main project **main.X** and select **Properties** to open its **Project Properties** Dialog. Click the **Conf:[default]** category to reveal the general project configuration information. The development tools used for testing the firmware are listed in section [2.2 Software Tools Used for Testing the firmware.](#22-software-tools-used-for-testing-the-firmware).

     In the **Conf:[default]** category window: 
     - Ensure the selected **Device** is **dsPIC33CH1024MP710.**
     - Select the **Connected Hardware Tool** to be used for programming and debugging. 
     - Select the specific Device Family Pack (DFP) from the available list of **Packs.** In this case, **dsPIC33CH-MP_DFP v1.15.378** is selected. 
     - Select the specific **Compiler Toolchain** from the available list of **XC-DSC** compilers. 
     In this case, **XC-DSC(v3.00)** is selected.
     - After selecting Hardware Tool and Compiler Toolchain, Device Pack, click the button **Apply**

     Please ensure that the selected MPLAB® XC-DSC Compiler and Device Pack support the device configured in the firmware

     <p align="left">
     <img  src="images/projectpropertiessettings.png"></p>

4. To build the main project (in this case, **main.X**) and program the device dsPIC33CH1024MP710, click **Make and Program Device Main project** on the toolbar.

     Upon this, MPLAB X IDE begin executing the following activities in order:
     - Builds Main Project **main.X** and 
     - Programs Main flash memory of dsPIC33CH1024MP710 with code generated when building the Main Project. 

    <p align="left">
    <img  src="images/deviceprogramming.png"></p>


	
5. Open the secondary project **pmsm.X (File > Open Project)** with device selection **dsPIC33CH1024MP710S1.**  
    <p align="left">
       <img  src="images/idesecondaryopen.png"></p>

6. Set the project **pmsm.X** as the main project by right clicking on the project name and selecting **Set as Main Project** as shown. The project **pmsm.X** will then appear in **bold.**
    <p align="left">
     <img  src="images/idesecondaryprojectsetup.png"></p>

7. Open <code>**userparms.h** </code> (**pmsm.X > Header Files**) in the project **pmsm.X.**  
     - Ensure that the macros <code>**TUNING</code>, <code>OPEN_LOOP_FUNCTIONING</code>, <code>TORQUE_MODE</code>, <code>SINGLE_SHUNT</code>** and <code>**INTERNAL_OPAMP_CONFIG**</code> is not defined in the header file<code> **userparms.h.**</code>
          <p align="left"><img  src="images/configParam.png"></p>
        <p align="left"><img  src="images/externalopampconfig.png"></p> 

     > **Note:**</br>
     >The motor phase currents can be reconstructed from the DC Bus current by appropriately sampling it during the PWM switching period, called a single-shunt reconstruction algorithm. The firmware can be configured to demonstrate **the single shunt reconstruction algorithm** by defining the macro <code>**SINGLE_SHUNT**</code> in the header file <code>**userparms.h**</code> 
     >For additional information, refer to Microchip application note **AN1299, “Single-Shunt Three-Phase Current Reconstruction Algorithm for Sensorless FOC of a PMSM.”**
     >By default, the firmware uses phase currents measured across the phase shunt resistors on two of the half-bridges of the three-phase inverter (**‘dual shunt configuration’**) to implement FOC.

8. Right click on the associated Secondary Project **pmsm.X** and select **Properties** to open its **Project Properties** Dialog. Click the **Conf: [default]** category to reveal the general project configuration information.

     In the **Conf: [default]** category window: 
     - Select the specific Compiler Toolchain from the available list of compilers. Please ensure MPLAB XC-DSC Compiler supports the device **dsPIC33CH1024MP710S1**.
     - After selecting Compiler Toolchain, click the button **Apply**.

     This step is required to build the Secondary Project with a specific compiler version.

9. Ensure that the checkbox **Load symbols when programming or building for production (slows process)** is checked under the **Loading** category of the **Project Properties** window of **Secondary** project **pmsm.X**       
        
      <p align="left">
      <img  src="images/loadvariables.png"></p>

    Also, go to **Tools > Options** and
           
      <p align="left">
      <img  src="images/tools_options.png"></p>
      
    ensure in the  **Embedded > Generic Settings** tab **ELF debug session symbol load methodology (MIPS/ARM)** is selected as **Pre-procesed (Legacy)** from the drop down.
           
      <p align="left">
      <img  src="images/embedded_legacy.png"></p>

10. To build the secondary project (in this case, **pmsm.X**) and program the device dsPIC33CH1024MP710S1, click **Make and Program Device Main project** on the toolbar.

     Upon this, MPLAB X IDE begin executing the following activities in order:
     - Builds Secondary Project **pmsm.X** 
     - Programs Main flash memory of dsPIC33CH1024MP710S1 with code generated when building the Secondary Project. 

    <p align="left">
    <img  src="images/deviceprogrammingsecondary.png"></p>

     > **Note:**</br>
     > In this firmware configuration, the Main Core and the Secondary Core are programmed separately. </p>

10. If the device is successfully programmed, **LD2 (LED1)** will be turned **ON**, indicating that the dsPIC® DSC is enabled.
    <p align="left">
     <img  src="images/led.png"></p>


11. Run or stop the motor by pressing the push button **SW1.** The motor should start spinning smoothly in one direction in the nominal speed range. Ensure that the motor is spinning smoothly without any vibration. The LED **LD3(LED2)** is turned **ON** to show the button is pressed to start the motor.
     <p align="left">
     <img  src="images/pushbuttons.png"></p>
 

12. The motor speed can be varied using the potentiometer **(POT1).**
    <p align="left">
    <img  src="images/potentiometer.png"></p>
 
13. Press the push button **SW2** to enter the extended speed range (<code>NOMINAL_SPEED_RPM</code> to <code>MAXIMUM_SPEED_RPM</code>).
Press the push button **SW2** again to revert the speed of the motor to its nominal speed range (<code>END_SPEED_RPM</code> to <code>NOMINAL_SPEED_RPM</code>).</p>

     <p align="left">
     <img  src="images/stopButton.png"></p> 

14. Press the push button **SW1** to stop the motor.


>**Note:**</br>
>The macros <code>END_SPEED_RPM</code>, <code>NOMINAL_SPEED_RPM</code>, and <code>MAXIMUM_SPEED_RPM</code> are specified in the header file <code>**userparms.h**</code> included in the project **pmsm.X.** The macros <code>NOMINAL_SPEED_RPM</code> and <code>MAXIMUM_SPEED_RPM</code> are defined as per the Motor manufacturer’s specifications. Exceeding manufacture specifications may damage the motor or the board or both.

## 5.3  Data visualization through X2C-Scope Plug-in of MPLAB X

X2C-Scope is a third-party plug-in in MPLAB X, which helps in real-time diagnostics. The application firmware comes with the initialization needed to interface the controller with the host PC to enable data visualization through the X2C-Scope plug-in. Ensure the X2C-Scope plug-in is installed. For more information on how to set up a plug-in, refer to either the [Microchip Developer Help page](https://microchipdeveloper.com/mplabx:tools-plugins-available) or the [web page.](https://x2cscope.github.io/docs/MPLABX_Plugin.html)
 
1. To establish serial communication with the host PC, connect a micro-USB cable between the host PC and **connector J16** on the MCLV-48V-300W Inverter Board. This interface is also used for programming.


2. Ensure the application is configured and running as described under section [5.2 Basic Demonstration](#52-basic-demonstration) by following steps 1 through 14.

3. Build the secondary project **pmsm.X**. To do that, right click on the project **pmsm.X** and select **Clean and Build**.
      <p align="left">
       <img  src="images/x2cscopecleanbuild.png"></p>

4. Open the **X2C-Scope** window by selecting **Tools>Embedded>X2CScope.**
      <p align="left">
       <img  src="images/x2cselection.png"></p>
 

5. **In the X2C-Scope Configuration** window, open the **Connection Setup** tab and click **Select Project.** This opens the drop-down menu **Select Project** with a list of opened projects. Select the specific project **pmsm** from the list of projects and click **OK.**
    <p align="left">
    <img  src="images/x2cprojectselection.png"></p>

6. To configure and establish the serial communication for **X2C-Scope**, open the **X2CScope Configuration** window, click on the **Connection Setup** tab and:
     - Set **Baudrate** as **115200**, which is configured in the application firmware. 
     - Click on the **Refresh** button to refresh and update the list of the available Serial COM ports connected to the Host PC. 
     - Select the specific **Serial port** detected when interfaced with the MCLV-48V-300W Inverter Board. The **Serial port** depends on the system settings

    <p align="left">
     <img  src="images/x2cconnectionsetup.png"></p>
 

7. Once the **Serial port** is detected, click on **Disconnected** and turn to **Connected**, to establish serial communication between the Host PC and the board.
     <p align="left">
    <img  src="images/x2cconnectionbutton.png"></p>


8. Open the **Project Setup** tab in the **X2CScope Configuration** window and,
     - Set **Scope Sampletime** as the interval at which <code>X2CScopeUpdate()</code> is called. In this application, it is every <code>50µs.</code> 
     - Then, click **Set Values** to save the configuration.

      <p align="left">
      <img  src="images/x2cprojectsetup.png"></p>


9.	Click on **Open Scope View** (in the **Data Views** tab of the **X2CScope Configuration** Window); this opens **Scope Window.**
     <p align="left">
      <img  src="images/x2cdataview.png"></p>
    	     
10. In the **Scope Window**, select the variables that must be watched. To do this, click on the **Source** against each channel, and a window **Select Variables** opens on the screen. From the available list, the required variable can be chosen. Ensure checkboxes **Enable** and **Visible** are checked for the variables to be plotted.
To view data plots continuously, uncheck **Single-shot.** When **Single-shot** is checked, it captures the data once and stops. The **Sample time factor** value multiplied by **Sample time** decides the time difference between any two consecutive data points on the plot.
    <p align="left">
    <img  src="images/x2cdatapointselection.png"></p>

11.	Click on **SAMPLE**, then the X2C-Scope window plots variables in real-time, which updates automatically.
     <p align="left">
     <img  src="images/x2csample.png"></p>
 

12.	Click on **ABORT** to stop.
     <p align="left">
     <img  src="images/x2cabort.png"></p>
 
 ## 6. REFERENCES:
For additional information, refer following documents or links.
1. AN1292 Application Note “[Sensorless Field Oriented Control (FOC) for a Permanent Magnet Synchronous Motor (PMSM) Using a PLL Estimator and Field Weakening (FW)](https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ApplicationNotes/ApplicationNotes/01292A.pdf)”
2. AN1299 Application Note “[Single-Shunt Three-Phase Current Reconstruction Algorithm for Sensorless FOC of a PMSM](http://ww1.microchip.com/downloads/en/appnotes/01299a.pdf)”
3. AN2721 "[Getting Started with Dual Core](https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ApplicationNotes/ApplicationNotes/AN2721-Getting-Started-with-Dual-Core-DS00002721A.pdf)" 
4. MCLV-48V-300W Inverter Board User’s Guide [(DS50003297)](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU16/ProductDocuments/UserGuides/Motor-Control-Low-Voltage-48V-300W-Inverter-Board-Users-Guide-DS50003297.pdf) 
5. dsPIC33CH1024MP710 Motor Control Dual In-Line Module (DIM) Information Sheet (DS50003069C)
6. dsPIC33CH1024MP710 Family datasheet [(DS70005371D)](https://ww1.microchip.com/downloads/en/DeviceDoc/dsPIC33CH1024MP710-Family-Data-Sheet-DS70005371D.pdf)
7. [Family Reference manuals (FRM) of dsPIC33CH1024MP710 family](https://www.microchip.com/en-us/product/dsPIC33CH1024MP710#document-table)
8. MPLAB® X IDE User’s Guide [(DS50002027)](https://ww1.microchip.com/downloads/en/DeviceDoc/50002027E.pdf) or [MPLAB® X IDE help](https://microchipdeveloper.com/xwiki/bin/view/software-tools/x/)
9. [MPLAB® X IDE installation](https://microchipdeveloper.com/xwiki/bin/view/software-tools/x/install-guide/)
10. [MPLAB® XC-DSC Compiler installation](https://microchipdeveloper.com/xwiki/bin/view/software-tools/xc-dsc/install/)
11. [Installation and setup of X2Cscope plugin for MPLAB X](https://x2cscope.github.io/docs/MPLABX_Plugin.html)
