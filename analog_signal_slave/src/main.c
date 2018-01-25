/*=============================================================================*/
/* demo3_usartIRQ.c                                                            */
/*                                                                             */
/* Demonstration des 2 interruptions du USART.                                 */
/*    Utilisez le cable seriel USART1                                          */
/*    Sur PC, utilisez PUTTY ou TERATERMINAL, mode seriel, 57600bauds N-1-8-F  */
/*=============================================================================*/
/* Composant FRAMEWORK a ajouter:  (GPIO et INTC deja inclu par defaut)        */
/*     PM Power Manager    (driver)                                            */
/*     USART               (driver)                                            */
/*     USART debug Strings (service)                                           */
/*=============================================================================*/

#include <asf.h>
#include "compiler.h"    // Definitions utile: de U8, S8, U16, S16, U32, S32, U32, U62, F32

#define TC_LED_CHANNEL      0
#define TC_LED              (&AVR32_TC)
#define TC_LED_IRQ          AVR32_TC_IRQ0
#define LED_INTRPT_PRIO     AVR32_INTC_INT0    // LED interrupt priority

#define USART_INTRPT_PRIO   AVR32_INTC_INT1    // USART interrupt priority

#define FPBA                FOSC0              // a 12Mhz
#define FALSE               0

U32 char_recu;
U8 compteur;

/**
 * 8bits pour conserver l'etat des led clignotant
 * 
 * bit 1: le cycle actuel du clignotement (0 = off, 1 = on)
 * bit 2: en acquisition (0 = faux, 1 = vrai)
*/
U8 led_status;
#define LED_STATUS_STATE       1
#define LED_STATUS_ACQ         2
#define LED_SET_STATE(led) if(led_status & LED_STATUS_STATE) { gpio_clr_gpio_pin(led); } else { gpio_set_gpio_pin(led); }

__attribute__((__interrupt__))
static void led_interrupt_handler(void)
{
	// Reset l'interrupt?
	tc_read_sr(TC_LED, TC_LED_CHANNEL);
	
	// Flip le premier bit (XOR)
	led_status ^= LED_STATUS_STATE;
	
	// Allume ou eteint les led selon le premier bit de led_status
	LED_SET_STATE(LED0_GPIO);
	if(led_status & LED_STATUS_ACQ)
	{
		LED_SET_STATE(LED1_GPIO);
	}
	else
	{
		gpio_set_gpio_pin(LED1_GPIO);
	}
}

//==================================================================================
// USART interrupt handler.
//  2 sources peuvent lancer cette interruption
//      bit RXRDY : Ce bit se leve sur RECEPTION D'UN CARACTERE (provenant du PC),
//                  et redescend lorqu'on lit le caractere recu (acces lecture dans RHR)
//      bit TXRDY : Ce bit se leve lorsqu'un transmission (vers le PC se termine,
//                  et demeure lever tant que le transmetteur est disponible.
//                  Si on lance une trasmission, celui-ci descend le temps de transmettre.
//                  Attention, lorsque le transmetteur ne transmet pas, ce bit est toujours a 1,
//                  donc il va toujours relancer l'interruption si vous oubliez le bit TXRDY du IER.

__attribute__((__interrupt__))
static void usart_int_handler(void)
{
	// Si cette interruption est lancee par une reception (bit RXRDY=1)
	if (AVR32_USART1.csr & (AVR32_USART_CSR_RXRDY_MASK))
	{
		//Lire le char recu dans registre RHR, et le stocker dans un 32bit
		char_recu = (AVR32_USART1.rhr & AVR32_USART_RHR_RXCHR_MASK);
		//Eliminer la source de l'IRQ, bit RXRDY (automatiquement mis a zero a la lecture de RHR)
		compteur=3;
		//Retransmettre ce caractere vers le PC, si transmetteur disponible, renvoi un echo
		if (AVR32_USART1.csr & (AVR32_USART_CSR_TXRDY_MASK))
		{
			AVR32_USART1.thr = (char_recu) & AVR32_USART_THR_TXCHR_MASK; // on renvoi le char
			// Activer la source d'interrution du UART en fin de transmission (TXRDY)
			AVR32_USART1.ier = AVR32_USART_IER_TXRDY_MASK;
		}
		
		// DEBUG: On active ou desactive le clignotement du led1
		// eventuellement, ceci sera activer lorsquón se met en mode acquisition (voir la spec)
		led_status ^= LED_STATUS_ACQ;
	}
	else  // Donc cette l'interruption est lancee par une fin de transmission, bit TXRDY=1
	{
		if(compteur>0)
		{
			//Retransmettre un caractere modifie vers le PC
			AVR32_USART1.thr = (++char_recu) & AVR32_USART_THR_TXCHR_MASK;
			// Eliminer la source de l'IRQ, bit TXRDY est automatiquement mis a zero en remplissant THR
			compteur--;
		}
		else
		{
			// On arrete les transferts
			// Impossible d'eliminer la source de l'IRQ sans remplir THR, parce que TXRDY est read-only.
			// On doit donc desactiver la source d'interrution du UART en fin de transmission (TXRDY).
			AVR32_USART1.idr = AVR32_USART_IDR_TXRDY_MASK;
		}
	}
}

//==================================================================================
int main(void)
{
	// Necessaire a la config des leds
	volatile avr32_tc_t *tc_led = TC_LED;
	static const tc_waveform_opt_t tc_led_waveform_opt =
	{
		.channel  = TC_LED_CHANNEL,                    // Channel selection.

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
	static const tc_interrupt_t tc_led_interrupt_config =
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


	/*******************************************/
	/** Initialisation des variables globales **/
	/*******************************************/
	
	static const gpio_map_t USART_GPIO_MAP =
	{
		{AVR32_USART1_RXD_0_0_PIN, AVR32_USART1_RXD_0_0_FUNCTION},
		{AVR32_USART1_TXD_0_0_PIN, AVR32_USART1_TXD_0_0_FUNCTION}
	};
	// initialisation des variables globales;
	compteur=0;
	led_status=0;
	
	// Preparatif pour l'enregistrement des interrupt handler du INTC.
	INTC_init_interrupts();
	
	// Desactive les interruptions pendant la configuration.
	Disable_global_interrupt();
	
	/***************************/
	/** Configuration des LED **/
	/***************************/
	
	// On initialise le crystal externe sur le channel 0
	pcl_switch_to_osc(PCL_OSC0, FOSC0, OSC0_STARTUP);
	// On setup le interrupt handler
	INTC_register_interrupt(&led_interrupt_handler, TC_LED_IRQ, LED_INTRPT_PRIO);
	// Autoriser les interruptions.
	Enable_global_interrupt();
	// Initialise le timer
	tc_init_waveform(tc_led, &tc_led_waveform_opt);
	// Configure la fréquence du timer
	// 1 interruption tous les .1s TODO: wtf this calculation means?
	tc_write_rc(tc_led, TC_LED_CHANNEL, (FPBA /32 ) / 10);
	tc_configure_interrupts(tc_led, TC_LED_CHANNEL, &tc_led_interrupt_config);
	// Finalement, on demarre le timer
	tc_start(tc_led, TC_LED_CHANNEL);
	
	/****************************/
	/**	Configuration de USART **/
	/****************************/

	// Au boot, 115kHz, on doit passer au crystal FOSC0=12MHz avec le PM
	pm_switch_to_osc0(&AVR32_PM, FOSC0, OSC0_STARTUP);

	// Assigner les pins du GPIO a etre utiliser par le USART1.
	gpio_enable_module(USART_GPIO_MAP,sizeof(USART_GPIO_MAP) / sizeof(USART_GPIO_MAP[0]));

	// Initialise le USART1 en mode seriel RS232, a 57600 BAUDS, a FOSC0=12MHz.
	init_dbg_rs232(FOSC0);


	// Enregister le USART interrupt handler au INTC, level INT1
	INTC_register_interrupt(&usart_int_handler, AVR32_USART1_IRQ, USART_INTRPT_PRIO);

	print_dbg("Taper un caractere sur le clavier du PC avec TeraTerminal...\r\n\n");

	// Activer la source d'interrution du UART en reception (RXRDY)
	AVR32_USART1.ier = AVR32_USART_IER_RXRDY_MASK;


	while(1)
	{
		// Rien a faire, tout ce fait par interuption !
	}
}
