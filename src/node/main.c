#include <stdint.h>
#include <avr/io.h>


/* bano */

#include "bano/src/node/bano_node.h"


/* uart */

#define CONFIG_UART 0
#if (CONFIG_UART == 1)
#include "nrf/src/uart.c"
#endif


/* led, portb[1] */

#define CONFIG_LED 0

#if (CONFIG_LED == 1)

static void led_setup(void)
{
#define LED_DDR DDRB
#define LED_PORT PORTB
#define LED_MASK (1 << 1)

  LED_DDR |= LED_MASK;
}

static void led_set_high(void)
{
  LED_PORT |= LED_MASK;
}

static void led_set_low(void)
{
  LED_PORT &= ~LED_MASK;
}

#endif /* CONFIG_LED */


/* reed sensor, pind[3], pcint19 */

#define REED_DDR DDRD
#define REED_PIN PIND
#define REED_MASK (1 << 3)

#if 0 /* unused, done by bano */
static void reed_setup(void)
{
  /* pcint2 vector, pcint19 flag */
#define REED_PCMSK PCMSK2
#define REED_PCICR_MASK (1 << 2)

  /* disable pullup */
#define REED_PORT PORTD
  REED_PORT &= ~REED_MASK;
  REED_DDR &= ~REED_MASK;

  /* enable interrupt on change pcint19 */
  PCICR |= REED_PCICR_MASK;
  REED_PCMSK |= REED_MASK;
}
#endif /* unused */

static uint8_t reed_is_high(void)
{
  /* normally closed */
  /* switch opened when field applied */
  /* external pull down resistor */
  /* high means door opened */
  
  return REED_PIN & REED_MASK;
}


/* bano handlers */

uint8_t bano_get_handler(uint16_t key, uint32_t* val)
{
  return BANO_FLAG_ERR;
}

uint8_t bano_set_handler(uint16_t key, uint32_t val)
{
  return BANO_FLAG_ERR;
}

void bano_timer_handler(void)
{
}

void bano_pcint_handler(void)
{
  const uint8_t x = reed_is_high();

#if ((CONFIG_LED == 1) || (CONFIG_UART == 1))
  if (x)
  {
#if (CONFIG_LED == 1)
    led_set_high();
#endif

#if (CONFIG_UART == 1)
    static uint8_t i = 0;
    if (((i++) & 0xf) == 0)
      uart_write((const uint8_t*)"\r\n", 2);
    uart_write((const uint8_t*)"x", 1);
#endif
  }
  else
  {
#if (CONFIG_LED == 1)
    led_set_low();
#endif
  }
#endif

  bano_send_set(0x2a, x);
}


/* main */

int main(void)
{
  bano_info_t info = bano_info_default;

#if (CONFIG_LED == 1)
  led_setup();
#endif

#if (CONFIG_UART == 1)
  uart_setup();
  uart_write((const uint8_t*)"go\r\n", 4);
#endif

  info.wake_mask = BANO_WAKE_PCINT;
  info.pcint_mask = 1UL << 19UL;

  info.disable_mask = BANO_DISABLE_ALL;
#if (CONFIG_UART == 1)
  info.disable_mask &= ~BANO_DISABLE_USART;
#endif

  bano_init(&info);
  bano_loop();
  bano_fini();

  return 0;
}
