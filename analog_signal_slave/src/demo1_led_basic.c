/*=============================================================================*/
/* demo1_led_basic.c                                                           */
/*                                                                             */
/* Roulement des LED1-LED2-LED3-LED4, avec un delai par boucle FOR a vide.     */
/* Demonstration inspiree de: gpio_peripheral_bus_example du FRAMEWORK AVR32.  */
/*=============================================================================*/
/*=============================================================================*/
/* Composant FRAMEWORK a ajouter:  (GPIO et INTC deja inclu par defaut)        */
/*     Aucun                                                                   */
/*=============================================================================*/

#include <asf.h>   
#include "compiler.h" // Definitions utile: de U8, S8, U16, S16, U32, S32, U32, U62, F32

/* Au reset, le microcontroleur opere sur un crystal interne a 115200Hz.
 * Il est possible de changer, en execution, pour le crystal OSC0 a 12Mhz. 
 * grace a un service du Power Manager (PM)  */

int main(void)
{
  U8 u8LedMap=0x01; // Unsigned int 8bits
  volatile U32 i, k;            // Unsigned int 32bits, volatile IMPORTANT !

  while (1)  // Boucle infinie
  {
       LED_Display(u8LedMap);      // Envoi chaque bits sur chaque pins du GPIO correspondant aux LEDs
       u8LedMap = (u8LedMap << 1); // Rotation des bits vers la gauche
       u8LedMap = u8LedMap & 0x0F; // Masquage des 4 bits superieurs
       if(u8LedMap==0)
    	   u8LedMap=0x01;

    // Attente de quelques msec pour visualiser les LEDS
       for (i = 0; i < 3000; i=i+1)
       {
    	   k++; // Boucle a vide, attente active...
       }
  }
}