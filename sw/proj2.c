
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "xparameters.h"
#include "xintc.h"
#include "xtmrctr.h"
#include "xgpio.h"
#include "mb_interface.h"
#include "platform.h"
#include "Nexys4IO.h"
#include "PMod544IOR2.h"
#include "pwm_tmrctr.h"
#include "microblaze_sleep.h"

#define DUTY_CYCLE_INCREMENTS 5
#define PWM_FREQ_MSK          0x07
#define PWM_FREQ_005KHZ       5000

#define MIN(a, b)  ( ((a) <= (b)) ? (a) : (b) )
#define MAX(a, b)  ( ((a) >= (b)) ? (a) : (b) )

XIntc    instIntrCtrl;               // Interrupt Controller instance
XTmrCtr  instPWMTimer;                 // PWM timer instance
XGpio    instGPIODebug;           // GPIO instance

XStatus  init_axi_devices(void);
void     errorExit(void);
void     init_welcom(void);
int      getFreqFromSwitches(u16 switches);
void     update_lcd(int freq, int dutycycle, u32 linenum);
void     FIT_Handler(void);


int main () {
   XStatus status;
   // VARS : to hold user input stuff
   int      encoderCurn = 0x0000;
   int      encoderPrev = 0x1000;   
   int      PWMDutyGen = 50;
   bool     PWMGenUpdateFlag = false;
   u32      PWMFreqGenRead;
   u32      PWMDutyGenRead;

   // Initializations
   init_platform();
   status = init_axi_devices();
   if (status != XST_SUCCESS) errorExit();
   init_welcom();

   
   while(1) {
      PWMGenUpdateFlag = false;

      // CHECK : Updates on ENCODER
      PMDIO_ROT_readRotcnt(&encoderCurn);
      if (encoderCurn != encoderPrev) {

         // SET seven-seg with encoder value
         NX4IO_SSEG_putU16Hex(SSEGLO, encoderCurn);

         PWMDutyGen = MAX(0, MIN(encoderCurn, 99));
         encoderPrev = encoderCurn;
         PWMGenUpdateFlag = true;
      }

      // SET : Update PWM Generator
      if (PWMGenUpdateFlag) {

         // set the new PWM parameters - PWM_SetParams stops the timer
         status = PWM_SetParams(&instPWMTimer, PWM_FREQ_005KHZ, PWMDutyGen);
         if (status != XST_SUCCESS) errorExit();

         // GET : read the PWM parameters back from the PWM generator
         PWM_GetParams(&instPWMTimer, &PWMFreqGenRead, &PWMDutyGenRead);
         update_lcd(PWMFreqGenRead, PWMDutyGenRead, 1);
         PWM_Start(&instPWMTimer);
      }

   }
   
}


void errorExit(void) {
   PMDIO_LCD_setcursor(1,0);
   PMDIO_LCD_wrstring("****** ERROR *******");
   PMDIO_LCD_setcursor(2,0);
   PMDIO_LCD_wrstring("INIT FAILED- EXITING");
   exit(XST_FAILURE);
}


XStatus init_axi_devices(void) {
   int status;
   
   // INITIALIZE : Nexys4IO
   status = NX4IO_initialize(XPAR_NEXYS4IO_0_S00_AXI_BASEADDR);
   if (status != XST_SUCCESS) return XST_FAILURE;
   NX4IO_setLEDs(0x0000FFFF);

   // INITIALIZE : PMOD544IO
   status = PMDIO_initialize(XPAR_PMOD544IOR2_0_S00_AXI_BASEADDR);
   if (status != XST_SUCCESS) return XST_FAILURE;
   // SET : Encoder val = 0, increment val = DUTY_CYCLE_CHANGE
   NX4IO_setLEDs(0x0000FFFE);
   PMDIO_ROT_init(DUTY_CYCLE_INCREMENTS, true);
   PMDIO_ROT_clear();
   NX4IO_setLEDs(0x0000FFFD);
   
   // This is currently crashing the program =/
   // No GPIO for now
   // INITIALIZE : Degug signal GPIO (1X 4-bit)
   
   /*status = XGpio_Initialize(&instGPIODebug,  XPAR_AXI_GPIO_0_BASEADDR);*/
   /*if (status != XST_SUCCESS) return XST_FAILURE;*/
   /*NX4IO_setLEDs(0x0000FFFC);*/
   /*// SET : Direction of output ports */
   /*XGpio_SetDataDirection(&instGPIODebug, 0, 0xFFFFFFF0);*/
   /*NX4IO_setLEDs(0x0000FFFB);*/

   // INITIALIZE : PWM timer/counter
   status = PWM_Initialize(&instPWMTimer, XPAR_AXI_TIMER_0_DEVICE_ID, false, XPAR_CPU_CORE_CLOCK_FREQ_HZ);
   if (status != XST_SUCCESS) return XST_FAILURE;
   NX4IO_setLEDs(0x0000FFFA);
   
   // INITIALIZE : Interrupt controller
   status = XIntc_Initialize(&instIntrCtrl, XPAR_MICROBLAZE_0_AXI_INTC_DEVICE_ID);
   if (status != XST_SUCCESS) return XST_FAILURE;
   NX4IO_setLEDs(0x0000FFF9);

   // SET : GPIO handler to interrupt vector
   status = XIntc_Connect(&instIntrCtrl, XPAR_MICROBLAZE_0_AXI_INTC_FIT_TIMER_0_INTERRUPT_INTR,
                         (XInterruptHandler)FIT_Handler,
                         (void *)0);
   if (status != XST_SUCCESS) return XST_FAILURE;
   NX4IO_setLEDs(0x0000FFF8);
 
   // SET : Interrupt controller enabled for all devices
   status = XIntc_Start(&instIntrCtrl, XIN_REAL_MODE);
   if (status != XST_SUCCESS) return XST_FAILURE;
   NX4IO_setLEDs(0x0000FFF7);

   // SET : Interrupts enabled
   XIntc_Enable(&instIntrCtrl, XPAR_MICROBLAZE_0_AXI_INTC_FIT_TIMER_0_INTERRUPT_INTR);
   NX4IO_setLEDs(0x0000FFF6);

   // SET : RGB LED duty cycles and disable output
   NX4IO_RGBLED_setDutyCycle(RGB1, 64, 0, 64);
   NX4IO_RGBLED_setChnlEn(RGB1, false, false, false);
   NX4IO_setLEDs(0x0000FFF5);

   return XST_SUCCESS;
}

void init_welcom(void) {
   // display the greeting   
   PMDIO_LCD_setcursor(1,0);
   PMDIO_LCD_wrstring(" Project 01     ");
   PMDIO_LCD_setcursor(2,0);
   PMDIO_LCD_wrstring(" by Chad Sutfin ");
   NX4IO_setLEDs(0x0000FFFF);
   MB_Sleep(2000);
   NX4IO_setLEDs(0x00000000);

   // write the static text to the display
   PMDIO_LCD_clrd();
   PMDIO_LCD_setcursor(1,0);
   PMDIO_LCD_wrstring("G|FR:    DCY:  %");
   PMDIO_LCD_setcursor(2,0);
   PMDIO_LCD_wrstring("M|FR:    DCY:  %");
   
   // turn off the LEDs and clear the seven segment display
   NX4IO_setLEDs(0x00000000);
   NX410_SSEG_setAllDigits(SSEGLO, CC_BLANK, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);
   NX410_SSEG_setAllDigits(SSEGHI, CC_BLANK, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);
}






void update_lcd(int freq, int dutycycle, u32 linenum) {
   PMDIO_LCD_setcursor(linenum, 5);
   PMDIO_LCD_wrstring("    ");
   PMDIO_LCD_setcursor(linenum, 5);
   if (freq < 1000) {  // display Hz if frequency < 1Khz
      PMDIO_LCD_putnum(freq, 10);
   }
   else if (freq < 1000000) {  // display frequency in KHz
      PMDIO_LCD_putnum((freq / 1000), 10);
      PMDIO_LCD_wrstring("K");
   }
   else {  // display frequency in KHz
      PMDIO_LCD_putnum((freq / 1000000), 10);
      PMDIO_LCD_wrstring("M");
   }
   PMDIO_LCD_setcursor(linenum, 13);
   PMDIO_LCD_wrstring("  %");
   PMDIO_LCD_setcursor(linenum, 13);
   PMDIO_LCD_putnum(dutycycle, 10);
}


void FIT_Handler(void) {
   
}
