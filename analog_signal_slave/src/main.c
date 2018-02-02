//TODO: header
#include <asf.h>
#include "compiler.h"    // Definitions utile: de U8, S8, U16, S16, U32, S32, U32, U62, F32

#define TC_INTERVAL_CHANNEL      0
#define TC_INTERVAL              (&AVR32_TC)
#define TC_INTERVAL_IRQ          AVR32_TC_IRQ0
#define INTERVAL_INTRPT_PRIO     AVR32_INTC_INT1    // LED interrupt priority
// On veut une interruption a 2000Hz
#define TC_LED_FREQ              750

#define SAMPLE_RATE_TGL_BTN      GPIO_PUSH_BUTTON_0
#define SAMPLE_RATE_TGL_IRQ      AVR32_GPIO_IRQ_0
#define SAMPLE_RATE_TGL_PRIO     AVR32_INTC_INT0

#define USART_INTRPT_PRIO   AVR32_INTC_INT2    // USART interrupt priority

#define FPBA                FOSC0              // a 12MMz = 12000000Hz
#define FALSE               0

U32 char_recu;
U8 compteur;

//PArtie pour Potentiomètre et Lumière Test

/* Connection of the light sensor */
#  define EXAMPLE_ADC_LIGHT_CHANNEL           2
#  define EXAMPLE_ADC_LIGHT_PIN               AVR32_ADC_AD_2_PIN
#  define EXAMPLE_ADC_LIGHT_FUNCTION          AVR32_ADC_AD_2_FUNCTION

/* Connection of the potentiometer */
#  define EXAMPLE_ADC_POTENTIOMETER_CHANNEL   1
#  define EXAMPLE_ADC_POTENTIOMETER_PIN       AVR32_ADC_AD_1_PIN
#  define EXAMPLE_ADC_POTENTIOMETER_FUNCTION  AVR32_ADC_AD_1_FUNCTION

/** GPIO pin/adc-function map. */
const gpio_map_t ADC_GPIO_MAP = {
	#if defined(EXAMPLE_ADC_LIGHT_CHANNEL)
	{EXAMPLE_ADC_LIGHT_PIN, EXAMPLE_ADC_LIGHT_FUNCTION},
	#endif
	#if defined(EXAMPLE_ADC_POTENTIOMETER_CHANNEL)
	{EXAMPLE_ADC_POTENTIOMETER_PIN, EXAMPLE_ADC_POTENTIOMETER_FUNCTION}
	#endif
};
#if defined(EXAMPLE_ADC_LIGHT_CHANNEL)
signed short adc_value_light = -1;
#endif
#if defined(EXAMPLE_ADC_POTENTIOMETER_CHANNEL)
signed short adc_value_pot   = -1;
#endif
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * 8bits pour conserver l'etat du programme
 * bit 1: le cycle actuel du clignotement (0 = off, 1 = on)
 * bit 2: en acquisition (0 = faux, 1 = vrai)
 * bit 3: taux d'échantillonage (0=1000, 1=2000)
*/
U8 status;
U8 led_timer_counter;
#define STATUS_LED_POWER_STATE    1
#define STATUS_INTERVAL_STATE     2
#define STATUS_IN_ACQ             4
#define STATUS_SAMPLE_RATE        8

// Allume ou eteint un led selon STATUS_LED_POWER_STATE
#define LED_SET_STATE(led) if(status & STATUS_LED_POWER_STATE) { gpio_clr_gpio_pin(led); } else { gpio_set_gpio_pin(led); }

__attribute__((__interrupt__))
static void interval_interrupt_handler(void)
{
	// Reset l'interrupt
	tc_read_sr(TC_INTERVAL, TC_INTERVAL_CHANNEL);
	
	status ^= STATUS_INTERVAL_STATE;
	// Puisque cette fonction est execute 2000 fois par secondes, on skip 250 cycles avant de mofifier les led (4Hz)
	if(++led_timer_counter >= 250) {
		led_timer_counter=0;
			// Flip le premier bit (XOR)
			status ^= STATUS_LED_POWER_STATE;
			
			// Allume ou eteint les led selon le premier bit de led_status
			LED_SET_STATE(LED0_GPIO);
			if(status & STATUS_IN_ACQ)
			{
				LED_SET_STATE(LED1_GPIO);
			}
			else
			{
				// on eteint le led
				gpio_set_gpio_pin(LED1_GPIO);
			}
	}
	
	// On exécute quand le flag STATUS_SAMPLE_RATE est vrai (2000 fois par seconde)
	// ou lorsque le flag STATUS_INTERVAL_STATE est vrai (1 cyce sur 2, effectivement 1000 fois par seconde)
	if((status & STATUS_SAMPLE_RATE || (status & STATUS_INTERVAL_STATE))&& (status&STATUS_IN_ACQ)) {
		// Code pour l'echantillonage ici
		//tests (Potentiometre et lumiere)
		//if(status&STATUS_IN_ACQ)
		//{
			adc_start(&AVR32_ADC);
			#if defined(EXAMPLE_ADC_LIGHT_CHANNEL)
			/* Get value for the light adc channel */
			adc_value_light = adc_get_value(&AVR32_ADC,
			EXAMPLE_ADC_LIGHT_CHANNEL);
			
			/* Display value to user */
			print_dbg("HEX Value for Channel light : 0x");
			print_dbg_hex(adc_value_light);
			print_dbg("\r\n");
			#endif

			#if defined(EXAMPLE_ADC_POTENTIOMETER_CHANNEL)
			/* Get value for the potentiometer adc channel */
			adc_value_pot = adc_get_value(&AVR32_ADC,
			EXAMPLE_ADC_POTENTIOMETER_CHANNEL);
			
			/* Display value to user */
			print_dbg("HEX Value for Channel pot : 0x");
			print_dbg_hex(adc_value_pot);
			print_dbg("\r\n");
			#endif
		//}
	}

}

__attribute__((__interrupt__))
static void toggle_sample_rate_handler(void)
{
	// reset l'interrupt
	gpio_clear_pin_interrupt_flag(SAMPLE_RATE_TGL_BTN);
	status ^= STATUS_SAMPLE_RATE;
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
		if(char_recu == 0x00000073 && !(status & STATUS_IN_ACQ)) //0x00000073="s" et on check si le mode acquisition n'est pas activé
		{
		    status ^= STATUS_IN_ACQ;
		}
		if(char_recu == 0x00000078 && (status & STATUS_IN_ACQ))//0x00000078="x" et on check si le mode acquisition est activé
		{
			status ^= STATUS_IN_ACQ;
		}
		
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
	volatile avr32_tc_t *tc_led = TC_INTERVAL;
	static const tc_waveform_opt_t tc_led_waveform_opt =
	{
		.channel  = TC_INTERVAL_CHANNEL,                    // Channel selection.

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
	status=0;
	
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
	INTC_register_interrupt(&interval_interrupt_handler, TC_INTERVAL_IRQ, INTERVAL_INTRPT_PRIO);
	// Autoriser les interruptions.
	Enable_global_interrupt();
	// Initialise le timer
	tc_init_waveform(tc_led, &tc_led_waveform_opt);
	// Configure la fréquence du timer
	tc_write_rc(tc_led, TC_INTERVAL_CHANNEL, TC_LED_FREQ);
	tc_configure_interrupts(tc_led, TC_INTERVAL_CHANNEL, &tc_led_interrupt_config);
	// Finalement, on demarre le timer
	tc_start(tc_led, TC_INTERVAL_CHANNEL);
	
	/*****************************/
	/**	Configuration du bouton **/
	/*****************************/
	gpio_enable_gpio_pin(SAMPLE_RATE_TGL_BTN);
	INTC_register_interrupt(&toggle_sample_rate_handler, SAMPLE_RATE_TGL_IRQ + (SAMPLE_RATE_TGL_BTN/8), SAMPLE_RATE_TGL_PRIO);
	gpio_enable_pin_interrupt(SAMPLE_RATE_TGL_BTN, GPIO_FALLING_EDGE);
	
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

	print_dbg("Pret\r\n\n");

	// Activer la source d'interrution du UART en reception (RXRDY)
	AVR32_USART1.ier = AVR32_USART_IER_RXRDY_MASK;

	/* Assign and enable GPIO pins to the ADC function. */
	gpio_enable_module(ADC_GPIO_MAP, sizeof(ADC_GPIO_MAP) /
	sizeof(ADC_GPIO_MAP[0]));

	//Enable ADC Channel
	#if defined(EXAMPLE_ADC_LIGHT_CHANNEL)
	adc_enable(&AVR32_ADC, EXAMPLE_ADC_LIGHT_CHANNEL);
	#endif
	#if defined(EXAMPLE_ADC_POTENTIOMETER_CHANNEL)
	adc_enable(&AVR32_ADC, EXAMPLE_ADC_POTENTIOMETER_CHANNEL);
	#endif

	while(1)
	{
		// Rien a faire, tout ce fait par interuption 
	}
}
