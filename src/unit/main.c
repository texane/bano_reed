#include <stdint.h>
#include <stddef.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/sleep.h>


#define CONFIG_UART 1
#define CONFIG_NRF905 1
#define CLK_PRESCAL (256UL)


/* uart */

#if (CONFIG_UART == 1)
#include "/home/texane/repo/nrf/src/uart.c"
#endif


/* nrf905 */

#if (CONFIG_NRF905 == 1)
#include "/home/texane/repo/nrf/src/nrf905.c"
#endif


/* led */

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


/* reed */

static volatile uint8_t reed_is_pcint = 0;

static void reed_setup(void)
{
#define REED_DDR DDRD
#define REED_PIN PIND
#define REED_PORT PORTD
#define REED_MASK (1 << 3)

  /* pcint2 vector, pcint19 flag */
#define REED_PCMSK PCMSK2
#define REED_PCICR_MASK (1 << 2)

  /* disable pullup */
  REED_PORT &= ~REED_MASK;
  REED_DDR &= ~REED_MASK;

  /* enable interrupt on change pcint19 */
  PCICR |= REED_PCICR_MASK;
  REED_PCMSK |= REED_MASK;
}

static uint8_t reed_is_high(void)
{
  /* normally closed */
  /* switch opened when field applied */
  /* external pull down resistor */
  /* high means door opened */
  
  return REED_PIN & REED_MASK;
}

ISR(PCINT2_vect)
{
  /* do nothing, processed by sequential */
  reed_is_pcint = 1;
}


/* system wide setup */

static inline void clk_set_prescal(uint8_t x)
{
  /* pow2, max 8 = 256 */

  CLKPR = 1 << 7;
  CLKPR = x << 0;

  /* wait for 4 - 2 cycles */
  __asm__ __volatile__ ("nop\n");
  __asm__ __volatile__ ("nop\n");
}

static inline void clk_set_prescal_max(void)
{
  clk_set_prescal(8);
}

static void sys_setup(void)
{
  /* leave with interrupts disabled */
  cli();

  set_sleep_mode(SLEEP_MODE_IDLE);

  clk_set_prescal_max();

#if (CONFIG_UART == 0)
  UCSR0B = 0;
#endif
  wdt_reset();
  MCUSR &= ~(1 << WDRF);
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  WDTCSR = 0x00;
  ACSR = 1 << 7;
  ADMUX = 0;
  ADCSRA = 0;

#if (CONFIG_UART == 0)
  PRR = 0xff & ~(1 << 2);
#else
  PRR = 0xff & ~((1 << 2) | (1 << 1));
#endif
}


/* main */

static void wait(void)
{
  sei();

  while (1)
  {
    sleep_disable();

    if (reed_is_pcint)
    {
      reed_is_pcint = 0;
      return ;
    }

    /* the following procedure is used to not miss interrupts */
    /* disable interrupts, check if something available */
    /* otherwise, enable interrupt and sleep (sei, sleep) */
    /* the later ensures now interrupt is missed */

    sleep_enable();
    sleep_bod_disable();

    cli();

    if (reed_is_pcint)
    {
      reed_is_pcint = 0;
      return ;
    }

    /* warning: keep the 2 instructions in the same block */
    /* atomic, no int schedule between sei and sleep_cpu */
    sei();
    sleep_cpu();
  }
}

int main(void)
{
  sys_setup();

  led_setup();
  reed_setup();

#if (CONFIG_UART == 1)
  uint8_t i = 0;
  uart_setup();
  uart_write((const uint8_t*)"go\r\n", 4);
#endif

#if (CONFIG_NRF905 == 1)
  uint8_t tx_addr[4] = { 0xaa, 0xab, 0xac, 0xad };
  uint8_t rx_addr[4] = { 0xa1, 0xa2, 0xa3, 0xa4 };

  /* setup spi first */
  spi_setup_master();
  spi_set_sck_freq(SPI_SCK_FREQ_FOSC2);

  nrf905_setup();
  nrf905_set_tx_addr(tx_addr, 4);
  nrf905_set_rx_addr(rx_addr, 4);
  nrf905_set_payload_width(2);
  nrf905_commit_config();
#endif

  while (1)
  {
#if (CONFIG_UART == 1)
    uart_write((const uint8_t*)">", 1);
#endif
    
    wait();

    if (reed_is_high())
    {
      led_set_high();
#if (CONFIG_UART == 1)
      if (((i++) & 0xf) == 0)
	uart_write((const uint8_t*)"\r\n", 2);
      uart_write((const uint8_t*)"x", 1);
#endif
    }
    else
    {
      led_set_low();
    }
  }

  return 0;
}
