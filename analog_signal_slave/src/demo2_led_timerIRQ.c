/*=============================================================================*/
/* demo2_led_timerIRQ.c                                                        */
/*                                                                             */
/* Clignotement des LED1-LED2, avec un delai IRQ D'UN TIMER-COUNTER.           */
/* Demonstration inspiree de: tc_example3 du FRAMEWORK ATMEL.                  */
/*=============================================================================*/
/*=============================================================================*/
/* Composant FRAMEWORK a ajouter:  (GPIO et INTC deja inclu par defaut)        */
/*     TC - Timer counter    (driver)                                          */
/*     PM - Power Manager    (driver)                                          */
/*=============================================================================*/

/* Le UC3A possede 3 TIMER-COUNTER identique, identifie par channel 0-1-2.     */
/* Ces TC sont de 16bits, donc il compte des cycles de 0x000 a RC choisi       */
/* Lorsque le compte atteint RC, il genere une interrupt qui toggle un LED.    */
/* Le TC compte a la vitesse FPBA/32 choisie (source TIMER_CLOCK4)             */

#include <asf.h>
#include "compiler.h"

#  define TC_CHANNEL                  0
#  define EXAMPLE_TC                  (&AVR32_TC)
#  define EXAMPLE_TC_IRQ_GROUP        AVR32_TC_IRQ_GROUP
#  define EXAMPLE_TC_IRQ              AVR32_TC_IRQ0
#  define FPBA                        FOSC0          // FOSC0 est a 12Mhz
#  define FALSE                       0

__attribute__((__interrupt__))

static void tc_irq(void)
{
  // La lecture du registre SR efface le fanion de l'interruption.
  tc_read_sr(EXAMPLE_TC, TC_CHANNEL);

  // Toggle le premier et le second LED.
  gpio_tgl_gpio_pin(LED0_GPIO);
  gpio_tgl_gpio_pin(LED1_GPIO);
}

int main(void)
{
  U32 i;

  volatile avr32_tc_t *tc = EXAMPLE_TC;

  // Configuration du peripherique TC
  static const tc_waveform_opt_t WAVEFORM_OPT =
  {
    .channel  = TC_CHANNEL,                        // Channel selection.

    .bswtrg   = TC_EVT_EFFECT_NOOP,                // Software trigger effect on TIOB.
    .beevt    = TC_EVT_EFFECT_NOOP,                // External event effect on TIOB.
    .bcpc     = TC_EVT_EFFECT_NOOP,                // RC compare effect on TIOB.
    .bcpb     = TC_EVT_EFFECT_NOOP,                // RB compare effect on TIOB.

    .aswtrg   = TC_EVT_EFFECT_NOOP,                // Software trigger effect on TIOA.
    .aeevt    = TC_EVT_EFFECT_NOOP,                // External event effect on TIOA.
    .acpc     = TC_EVT_EFFECT_NOOP,                // RC compare effect on TIOA: toggle.
    .acpa     = TC_EVT_EFFECT_NOOP,                // RA compare effect on TIOA: toggle 
    .wavsel   = TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER,// Waveform selection: Up mode with automatic trigger(reset) on RC compare.
    .enetrg   = FALSE,                             // External event trigger enable.
    .eevt     = 0,                                 // External event selection.
    .eevtedg  = TC_SEL_NO_EDGE,                    // External event edge selection.
    .cpcdis   = FALSE,                             // Counter disable when RC compare.
    .cpcstop  = FALSE,                             // Counter clock stopped with RC compare.

    .burst    = FALSE,                             // Burst signal selection.
    .clki     = FALSE,                             // Clock inversion.
    .tcclks   = TC_CLOCK_SOURCE_TC4                // Internal source clock 3, connected to fPBA / 8.
  };

  static const tc_interrupt_t TC_INTERRUPT =
  {
    .etrgs = 0,
    .ldrbs = 0,
    .ldras = 0,
    .cpcs  = 1,
    .cpbs  = 0,
    .cpas  = 0,
    .lovrs = 0,
    .covfs = 0
  };

  /*! \brief Main function:
   *  - Configure the CPU to run at 12MHz
   *  - Register the TC interrupt (GCC only)
   *  - Configure, enable the CPCS (RC compare match) interrupt, and start a TC channel in waveform mode
   *  - In an infinite loop, do nothing
   */

  /* Au reset, le microcontroleur opere sur un crystal interne a 115200Hz. */
  /* Nous allons le configurer pour utiliser un crystal externe, FOSC0, a 12Mhz. */
  pcl_switch_to_osc(PCL_OSC0, FOSC0, OSC0_STARTUP);

  Disable_global_interrupt(); // Desactive les interrupts le temps de la config
  INTC_init_interrupts();     // Initialise les vecteurs d'interrupt

  // Enregistrement de la nouvelle IRQ du TIMER au Interrupt Controller .
  INTC_register_interrupt(&tc_irq, EXAMPLE_TC_IRQ, AVR32_INTC_INT0);
  Enable_global_interrupt();  // Active les interrupts

  tc_init_waveform(tc, &WAVEFORM_OPT);     // Initialize the timer/counter waveform.

  // Placons le niveau RC a atteindre pour declencher de l'IRQ.
  // Attention, RC est un 16-bits, valeur max 65535

  // We want: (1/(fPBA/32)) * RC = 0.100 s, donc RC = (fPBA/32) / 10  to get an interrupt every 100 ms.
  tc_write_rc(tc, TC_CHANNEL, (FPBA / 32) / 10); // Set RC value.

  tc_configure_interrupts(tc, TC_CHANNEL, &TC_INTERRUPT);

  // Start the timer/counter.
  tc_start(tc, TC_CHANNEL);                    // And start the timer/counter.

  while(1) // Boucle inifinie a vide !
  {
   i++;
  }
}