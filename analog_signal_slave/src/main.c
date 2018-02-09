//TODO: header
#include <asf.h>
#include "compiler.h"    // Definitions utile: de U8, S8, U16, S16, U32, S32, U32, U62, F32

#define LED_ACTIVE                  LED0_GPIO
#define LED_ACQ                     LED1_GPIO
#define LED_ADC_OVERFLOW            LED2_GPIO
#define LED_USART_OVERFLOW          LED3_GPIO

#define TC_INTERVAL                 (&AVR32_TC)

#define TC_LED_CHANNEL              0
#define TC_LED_IRQ                  AVR32_TC_IRQ0
#define LED_INTRPT_PRIO             AVR32_INTC_INT2    // LED interrupt priority
// On veut 8 interruptions par seconde (1000/8=125ms)
// Comme la valeur maximale est 65535, on configure une frequence de 32Hz et on ignore 1 interruption sur 8
#define TC_LED_FREQ                 46875              // solve(1/(fpba/32)*x=0.125, x) => x=46875

#define TC_SAMPLE_CHANNEL           1
#define TC_SAMPLE_IRQ               AVR32_TC_IRQ1
#define SAMPLE_INTRPT_PRIO          AVR32_INTC_INT1
// On veut 2000 interruptions par seconde (1000/2000=0.5ms)
#define TC_SAMPLE_FREQ              188                // solve(1/(fpba/32)*x=0.0005, x) => x=187.5

#define SAMPLE_RATE_TGL_BTN         GPIO_PUSH_BUTTON_0
#define SAMPLE_RATE_TGL_IRQ         AVR32_GPIO_IRQ_0
#define SAMPLE_RATE_TGL_PRIO        AVR32_INTC_INT0

#define USART_INTRPT_PRIO           AVR32_INTC_INT0    // USART interrupt priority
#define ACQ_START_CHAR              0x00000073         // 0x00000073 = 's'
#define ACQ_STOP_CHAR               0x00000078         // 0x00000078 = 'x'
#define USART_TX_VAL_MASK           0x000000FE         // Les 7 premiers bits
#define USART_TX_VAL_LIGHT_ID       1
#define USART_TX_VAL_POT_ID         0
#define USART_TX_SET_VAL(val,id) \
AVR32_USART1.thr = ((val >> 2 & USART_TX_VAL_MASK) | id) & AVR32_USART_THR_TXCHR_MASK; AVR32_USART1.ier = AVR32_USART_IER_TXRDY_MASK;

/* Connection of the light sensor */
#define ADC_LIGHT_CHANNEL           2
#define ADC_LIGHT_PIN               AVR32_ADC_AD_2_PIN
#define ADC_LIGHT_FUNCTION          AVR32_ADC_AD_2_FUNCTION

/* Connection of the potentiometer */
#define ADC_POTENTIOMETER_CHANNEL   1
#define ADC_POTENTIOMETER_PIN       AVR32_ADC_AD_1_PIN
#define ADC_POTENTIOMETER_FUNCTION  AVR32_ADC_AD_1_FUNCTION
   
#define FPBA                        FOSC0              // a 12MMz = 12000000Hz
#define FALSE                       0
#define TRUE                        1

/**
 * 8bits pour conserver l'etat du programme
 * bit 1: le cycle actuel du clignotement (0 = off, 1 = on)
 * bit 2: en acquisition (0 = faux, 1 = vrai)
 * bit 3: taux d'échantillonage (0=1000, 1=2000)
*/
U8 status = 0;
#define STATUS_LED_POWER_STATE    1
#define STATUS_INTERVAL_STATE     2
#define STATUS_IN_ACQ             4
#define STATUS_SAMPLE_RATE        8
#define STATUS_TX_LIGHT_READY    16
#define STATUS_TX_POT_READY      32

// Allume ou eteint un led selon STATUS_LED_POWER_STATE
#define LED_SET_STATE(led) if(status & STATUS_LED_POWER_STATE) { gpio_clr_gpio_pin(led); } else { gpio_set_gpio_pin(led); }

__attribute__((__interrupt__))
static void interval_led_handler(void)
{
	// Reset l'interrupt
	tc_read_sr(TC_INTERVAL, TC_LED_CHANNEL);
	
	status ^= STATUS_INTERVAL_STATE;
	// Flip le premier bit (XOR)
	status ^= STATUS_LED_POWER_STATE;
		
	// Allume ou eteint les led selon le premier bit de led_status
	LED_SET_STATE(LED_ACTIVE);
	if(status & STATUS_IN_ACQ)
	{
		LED_SET_STATE(LED_ACQ);
	}
	else
	{
		// on eteint le led
		gpio_set_gpio_pin(LED_ACQ);
	}
}

volatile short adc_value_light = -1;
volatile short adc_value_pot   = -1;
__attribute__((__interrupt__))
static void interval_sample_handler(void)
{
	// Reset l'interrupt
	tc_read_sr(TC_INTERVAL, TC_SAMPLE_CHANNEL);
	
	// On exécute quand le flag STATUS_SAMPLE_RATE est vrai (2000 fois par seconde)
	// ou lorsque le flag STATUS_INTERVAL_STATE est vrai (1 cyce sur 2, effectivement 1000 fois par seconde)
	if((status & STATUS_SAMPLE_RATE || (status & STATUS_INTERVAL_STATE)) && (status & STATUS_IN_ACQ)) {
		// On recupere la donne de l'ADC
		adc_start(&AVR32_ADC);
		adc_value_light = adc_get_value(&AVR32_ADC, ADC_LIGHT_CHANNEL);
		adc_value_pot = adc_get_value(&AVR32_ADC, ADC_POTENTIOMETER_CHANNEL);
		
		// On transmet si le transmetteur est disponible et les precedante donnee on ete transmise
		if ((AVR32_USART1.csr & (AVR32_USART_CSR_TXRDY_MASK))
		 && (status & (STATUS_TX_LIGHT_READY | STATUS_TX_POT_READY)))
		{
			status |= (STATUS_TX_LIGHT_READY | STATUS_TX_POT_READY);
			
			// On transmet les donnees du capteur de lumiere
			USART_TX_SET_VAL(adc_value_light, USART_TX_VAL_LIGHT_ID);
			status ^= STATUS_TX_LIGHT_READY;
		}
		else 
		{
			// On allume le temoin pour le depassement de l'USART
			gpio_clr_gpio_pin(LED_USART_OVERFLOW);
		}
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

volatile U32 char_recu = 0;
__attribute__((__interrupt__))
static void usart_int_handler(void)
{
	// Si cette interruption est lancee par une reception (bit RXRDY=1)
	if (AVR32_USART1.csr & (AVR32_USART_CSR_RXRDY_MASK))
	{
		//Lire le char recu dans registre RHR, et le stocker dans un 32bit
		char_recu = (AVR32_USART1.rhr & AVR32_USART_RHR_RXCHR_MASK);
		
		// On active ou désactive le clignotement du led selon le charactere recus
		if(char_recu == ACQ_START_CHAR && !(status & STATUS_IN_ACQ)) 
		{
		    status ^= STATUS_IN_ACQ;
		}
		if(char_recu == ACQ_STOP_CHAR && (status & STATUS_IN_ACQ))
		{
			status ^= STATUS_IN_ACQ;
		}
		
	}
	else  // Donc cette l'interruption est lancee par une fin de transmission, bit TXRDY=1
	{
		if(status & STATUS_TX_POT_READY)
		{
			// On transmet les donnees pour le potentiometre
			USART_TX_SET_VAL(adc_value_pot, USART_TX_VAL_POT_ID);
			status ^= STATUS_TX_POT_READY;
		}
		else if(!(status & STATUS_IN_ACQ))
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
	volatile avr32_tc_t *tc_sample = TC_INTERVAL;
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
	static const tc_waveform_opt_t tc_sample_waveform_opt =
	{
		.channel  = TC_SAMPLE_CHANNEL,                 // Channel selection.

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
	static const tc_interrupt_t tc_interrupt_config =
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

	static const gpio_map_t USART_GPIO_MAP =
	{
		{AVR32_USART1_RXD_0_0_PIN, AVR32_USART1_RXD_0_0_FUNCTION},
		{AVR32_USART1_TXD_0_0_PIN, AVR32_USART1_TXD_0_0_FUNCTION}
	};
	
	static const gpio_map_t ADC_GPIO_MAP =
	{
		{ADC_LIGHT_PIN, ADC_LIGHT_FUNCTION},
		{ADC_POTENTIOMETER_PIN, ADC_POTENTIOMETER_FUNCTION}
	};
	
	// Preparatif pour l'enregistrement des interrupt handler du INTC.
	INTC_init_interrupts();
	// Desactive les interruptions pendant la configuration.
	Disable_global_interrupt();
	
	/***************************/
	/** Configuration des LED **/
	/***************************/

	// On initialise le crystal externe sur le channel 0
	pcl_switch_to_osc(PCL_OSC0, FPBA, OSC0_STARTUP);
	// On setup le interrupt handler
	INTC_register_interrupt(&interval_led_handler, TC_LED_IRQ, LED_INTRPT_PRIO);
	// Initialise le timer
	tc_init_waveform(tc_led, &tc_led_waveform_opt);
	// Configure la fréquence du timer
	tc_write_rc(tc_led, TC_LED_CHANNEL, TC_LED_FREQ);
	tc_configure_interrupts(tc_led, TC_LED_CHANNEL, &tc_interrupt_config);
	// Finalement, on demarre le timer
	tc_start(tc_led, TC_LED_CHANNEL);
	
	/***************************************/
	/** Configuration de l'echantillonage **/
	/***************************************/

	// Essentiellement les meme etapes que pour les led
	INTC_register_interrupt(&interval_sample_handler, TC_SAMPLE_IRQ, SAMPLE_INTRPT_PRIO);
	tc_init_waveform(tc_sample, &tc_sample_waveform_opt);
	tc_write_rc(tc_sample, TC_SAMPLE_CHANNEL, TC_SAMPLE_FREQ);
	tc_configure_interrupts(tc_sample, TC_SAMPLE_CHANNEL, &tc_interrupt_config);
	tc_start(tc_sample, TC_SAMPLE_CHANNEL);
	
	// Active les pins requisent pour l'ADC
	gpio_enable_module(ADC_GPIO_MAP, sizeof(ADC_GPIO_MAP) / sizeof(ADC_GPIO_MAP[0]));
	// Active l'ADC
	adc_enable(&AVR32_ADC, ADC_LIGHT_CHANNEL);
	adc_enable(&AVR32_ADC, ADC_POTENTIOMETER_CHANNEL);

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

	// Activer la source d'interrution du UART en reception (RXRDY)
	AVR32_USART1.ier = AVR32_USART_IER_RXRDY_MASK;

	// Autoriser les interruptions.
	Enable_global_interrupt();

	while(1) {} // Rien a faire, tout ce fait par interuption 
}
