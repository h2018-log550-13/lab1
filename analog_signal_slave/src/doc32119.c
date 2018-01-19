/*This file is prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief Getting Started main.c for 32-bit AVR UC3 A0/A1 devices.
 *
 * AVR32 Getting started C file.
 *
 * - Compiler:           IAR EWAVR32 and GNU GCC for AVR UC3
 * - Supported devices:  UC3 A0/A1 series
 * - AppNote:
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support and FAQ: http://support.atmel.no/
 *
 *****************************************************************************/

/* Copyright (c) 2009 Atmel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an Atmel
 * AVR product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 */
#include <avr32/io.h>
#include "intc.h"


//--------------------------------------------------------------------------
// Timer counter used in this example
#define TC_CHANNEL    0

/*! \brief Enables interrupts globally.
 */
#if __GNUC__
  #define Enable_global_interrupt()         ({__asm__ __volatile__ ("csrf\t%0" :  : "i" (AVR32_SR_GM_OFFSET));})
#elif __ICCAVR32__
  #define Enable_global_interrupt()         (__enable_interrupt())
#endif

// Timer tick - used in the TC interrupt
volatile unsigned int tc_tick=0;
// To specify we have to toggle each second
volatile static int print_sec = 1;

/*! 
 * \brief TC interrupt handler. Increments tc_tick variable.
 *
 */
__attribute__((__interrupt__)) static void tc_irq( void )
{
  // Increment the ms seconds counter
  tc_tick++;

  // clear the interrupt flag of TC channel 0
  AVR32_TC.channel[0].sr;

  // specify that an interrupt has been raised
  print_sec = 1;
}

/*! 
 * \brief GPIO interrupt handler. 
 * Use GPIO88 (connected to EVK1100 push button 0) to toggle PB22
 * 
 */
__attribute__((__interrupt__)) static void gpio_irq( void )
{
  // GPIO88 (PX16) is connected to push button 0. 
  // GPIO88 bit control can be found in gpio port 2 (88/32=>2), bit 24 (88%32=24).
  AVR32_GPIO.port[2].ifrc = 1<<24;  
 
  // Toggle the I/O line GPIO54 (PB22)
  // GPIO54 bit control can be found in gpio port 1 (54/32=>1), bit 22 (54%32=22).
  AVR32_GPIO.port[1].ovrt  = 1 << (22 & 0x1F); 
}

/*! 
 * \brief GPIO init. 
 * Init GPIO(51) (PB19), GPIO54 (PB22) as output.
 * Init GPIO88 (PX16) as interrupt (pin level change)
 */
void gpio_init(void)
{
  // Init GPIO51 (PB19)	
  AVR32_GPIO.port[1].oders = 1 << (19 & 0x1F); // The GPIO output driver is enabled for that pin.
  AVR32_GPIO.port[1].gpers = 1 << (19 & 0x1F); // The GPIO module controls that pin.
	
  // Init GPIO54 (PB22)	
  AVR32_GPIO.port[1].oders = 1 << (22 & 0x1F); // The GPIO output driver is enabled for that pin.
  AVR32_GPIO.port[1].gpers = 1 << (22 & 0x1F); // The GPIO module controls that pin.

  // Init GPIO88 (PX16) as interrupt (pin level change)
  // GPIO88 bit control can be found in gpio port 2 (88/32=>2), bit 24 (88%32=24).
  // Enable the glitch filter on GPIO88.
  AVR32_GPIO.port[2].gfers = 1 << (24 & 0x1F);

  // Configure the edge detector on pin change on GPIO88. We use imr0c (c like 'clear')
  // to clear the specified bit.
  AVR32_GPIO.port[2].imr0c = 1 << (24 & 0x1F);
  AVR32_GPIO.port[2].imr1c = 1 << (24 & 0x1F);

  // Enable interrupt on GPIO88
  AVR32_GPIO.port[2].iers = 1 << (24 & 0x1F);
}


/*! 
 * \brief TC channel 0 init to get an interrupt every 1ms.
 */
void tc_init(void)
{
  // Init Waveform operating mode.
  AVR32_TC.channel[0].cmr =       AVR32_TC_WAVSEL_UP_AUTO << AVR32_TC_WAVSEL_OFFSET | // Waveform selection: Up mode with automatic trigger(reset) on RC compare.
                                  AVR32_TC_EEVTEDG_NO_EDGE << AVR32_TC_EEVTEDG_OFFSET | // External event edge selection.
                                  AVR32_TC_TCCLKS_TIMER_DIV3_CLOCK << AVR32_TC_TCCLKS_OFFSET; // Internal source clock 3 - connected to PBA/8

  // Enable the interrupt on Timer RC compare of channel 0.
  AVR32_TC.channel[0].ier = 1 << AVR32_TC_CPCS_OFFSET;
  
  // Configure RC compare.
  // The 16-bit timer/counter channel will cycle from 0x0000 to RC. RC is initialized
  // to (PBA/8)/ 1000, so that an interrupt will be triggered every 1 ms.
  // TC counter is 16-bits, so counting second is not possible. We configure it to count ms.
  //  (1/(FPBA/8)) * RC = 1000 Hz => RC = (FPBA/8) / 1000 = 1500 to get an interrupt every 1ms
  AVR32_TC.channel[0].rc=(12000000/8)/1000;
  
  // Enable, reset and start the selected timer/counter channel 0.
  AVR32_TC.channel[0].ccr = AVR32_TC_SWTRG_MASK | AVR32_TC_CLKEN_MASK;
}

#if (defined __GNUC__)
  #define Set_system_register(sysreg, value)  __builtin_mtsr(sysreg, value)
#elif (defined __ICCAVR32__)
  #define Set_system_register(sysreg, value)  __set_system_register(sysreg, value)
#endif
extern void _evba; // defined in exception.S

static void INTC_init_evba(void)
{
  Set_system_register(AVR32_EVBA, &_evba );

}

/*! \brief Main function. Execution starts here.
 * Toggle EVK1100 LED5 every 1second. When pushbutton 0 is pressed, toggle EVK1100 LED6.
 */
int main(void)
{
  // Configure oscillator 0 in crystal mode
  AVR32_PM.oscctrl0=AVR32_PM_OSCCTRL0_MODE_CRYSTAL_G3<<AVR32_PM_OSCCTRL0_MODE_OFFSET | 3<<AVR32_PM_OSCCTRL0_STARTUP_OFFSET;
  
  // Enable oscillator 0
  AVR32_PM.mcctrl |= AVR32_PM_MCCTRL_OSC0EN_MASK;

  // wait oscillator 0 ready
  while (!(AVR32_PM.poscsr & AVR32_PM_POSCSR_OSC0RDY_MASK));
  
  // Switch main clock from internal RC oscillator to oscillator 0.
  // on EVK1100, a 12MHz is connected to OSC0.
  AVR32_PM.mcctrl |=  AVR32_PM_MCCTRL_MCSEL_OSC0;


  // As we are using a local interrupt driver intc.c, we need to setup the EVBA (exception vector base address)
  INTC_init_evba();

  // Initialize interrupt vectors.
  INTC_init_interrupts();
  
  // Register TC interrupt for INT0 level to the interrupt controller
  // tc_irq is the interrupt handler to register.
  // AVR32_TC_IRQ0 is the IRQ of the interrupt handler to register.
  // INT0 is the interrupt priority level to assign to the group of this IRQ.
  INTC_register_interrupt(&tc_irq, AVR32_TC_IRQ0, AVR32_INTC_INT0);

  // Register GPIO interrupt for INT1 level.
  // In every port there are four interrupt lines connected to the interrupt 
  // controller. Every eigth interrupts in the port are ored together to form an
  // interrupt line.
  // 
  // That is why we use the formula "AVR32_GPIO_IRQ_0 + (88/8)"
  // to register the GPIO88; AVR32_GPIO_IRQ_0 is used as the base interrupt line 
  // and we add '(88/8)' to register the corresponding interrupt
  // line.
  INTC_register_interrupt(&gpio_irq, (AVR32_GPIO_IRQ_0+88/8), AVR32_INTC_INT1);
     
  // Init all I/Os   
  gpio_init();
  
  // init TC channel
  tc_init();
  
  // Enable global interrupts
  Enable_global_interrupt(); 
    
  while(1){
  	// each 1sec, toggle the EVK1100 LED5
    if ((print_sec) && (!(tc_tick%1000)))
    {
      AVR32_GPIO.port[1].ovrt  = 1 << (19 & 0x1F); // Toggle the GPIO51 (PB19)	
      print_sec = 0;
    }
  }

  return 0;
}