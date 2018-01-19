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

U32 char_recu;       // Variables globales unsigned 32bits
U8 compteur;         // Variables globales unsigned 8bits
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
	static const gpio_map_t USART_GPIO_MAP =
	{
		{AVR32_USART1_RXD_0_0_PIN, AVR32_USART1_RXD_0_0_FUNCTION},
		{AVR32_USART1_TXD_0_0_PIN, AVR32_USART1_TXD_0_0_FUNCTION}
	};
	// initialisation des variables globales;
	compteur=0;

	// Au boot, 115kHz, on doit passer au crystal FOSC0=12MHz avec le PM
	pm_switch_to_osc0(&AVR32_PM, FOSC0, OSC0_STARTUP);

	// Assigner les pins du GPIO a etre utiliser par le USART1.
	gpio_enable_module(USART_GPIO_MAP,sizeof(USART_GPIO_MAP) / sizeof(USART_GPIO_MAP[0]));

	// Initialise le USART1 en mode seriel RS232, a 57600 BAUDS, a FOSC0=12MHz.
	init_dbg_rs232(FOSC0);

	// Desactive les interruptions pendant la configuration.
	Disable_global_interrupt();

	// Preparatif pour l'enregistrement des interrupt handler du INTC.
	INTC_init_interrupts();

	// Enregister le USART interrupt handler au INTC, level INT0
	INTC_register_interrupt(&usart_int_handler, AVR32_USART1_IRQ, AVR32_INTC_INT0);

	print_dbg("Taper un caractere sur le clavier du PC avec TeraTerminal...\n\n");

	// Activer la source d'interrution du UART en reception (RXRDY)
	AVR32_USART1.ier = AVR32_USART_IER_RXRDY_MASK;

	// Autoriser les interruptions.
	Enable_global_interrupt();

	while(1)
	{
		// Rien a faire, tout ce fait par interuption !
	}
}
