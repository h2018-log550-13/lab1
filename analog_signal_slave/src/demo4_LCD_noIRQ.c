/*=============================================================================*/
/* demo4_LCD_noIRQ.c                                                           */
/*                                                                             */
/* Affichage de 2 chaines de caracteres sur le LCD (DIP204) via le             */
/* peripherique de communication seriel SPI.                                   */
/*                                                                             */
/* Version originale, FRAMEWORK AVR32, Component-DIP204 demo                   */
/*=============================================================================*/
/*=============================================================================*/
/* Composant FRAMEWORK a ajouter:  (GPIO et INTC deja inclu par defaut)        */
/*     TC - Timer counter                (driver)                              */
/*     PM - Power Manager                (driver)                              */
/*     SPI - Serial Peripheral Interface (driver)                              */
/*     PWM - Pulse Width Modulation      (driver)                              */
/*     CPU Cycle counter                 (driver)                              */
/*     LCD DISPLAY DIP204                (component)                           */
/*     Delays Routines                   (services)                            */
/*=============================================================================*/

//CETTE DEMO EST INSTABLE.......
#include <asf.h>
#include "compiler.h"

#define   TRUE   1

int main(void)
{
  static const gpio_map_t DIP204_SPI_GPIO_MAP =
  {
    {DIP204_SPI_SCK_PIN,  DIP204_SPI_SCK_FUNCTION },  // SPI Clock.
    {DIP204_SPI_MISO_PIN, DIP204_SPI_MISO_FUNCTION},  // MISO.
    {DIP204_SPI_MOSI_PIN, DIP204_SPI_MOSI_FUNCTION},  // MOSI.
    {DIP204_SPI_NPCS_PIN, DIP204_SPI_NPCS_FUNCTION}   // Chip Select NPCS.
  };

  // Switch the CPU main clock to oscillator 0
  pm_switch_to_osc0(&AVR32_PM, FOSC0, OSC0_STARTUP);

  // add the spi options driver structure for the LCD DIP204
  spi_options_t spiOptions =
  {
    .reg          = DIP204_SPI_NPCS,
    .baudrate     = 1000000,
    .bits         = 8,
    .spck_delay   = 0,
    .trans_delay  = 0,
    .stay_act     = 1,
    .spi_mode     = 0,
    .modfdis      = 1
  };
  
  // Assign I/Os to SPI
  gpio_enable_module(DIP204_SPI_GPIO_MAP,
                     sizeof(DIP204_SPI_GPIO_MAP) / sizeof(DIP204_SPI_GPIO_MAP[0]));

  // Initialize as master
  spi_initMaster(DIP204_SPI, &spiOptions);

  // Set selection mode: variable_ps, pcs_decode, delay
  spi_selectionMode(DIP204_SPI, 0, 0, 0);

  // Enable SPI
  spi_enable(DIP204_SPI);

  // setup chip registers
  spi_setupChipReg(DIP204_SPI, &spiOptions, FOSC0);

  // initialize delay driver
  delay_init( FOSC0 );

  // initialize LCD
  dip204_init(backlight_PWM, TRUE);

  // Display default message.
  dip204_set_cursor_position(5,1);
  dip204_write_string("## LOG550 ##");
  //delay_ms(10);
  //dip204_set_cursor_position(5,2);
  dip204_write_string("UC3A0512");
  //dip204_set_cursor_position(2,3);
  dip204_write_string("Atmel UC3A series");
  dip204_hide_cursor();

  /* do a loop */
  while (1)
  {

  }
}
