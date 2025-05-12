#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <serial.h>
#include <stdio.h>
#include <stdlib.h>

volatile uint32_t millis{};

void delay_ms(uint32_t ms) {
  uint32_t tm{millis + ms};
  while (millis < tm)
    ;
}

void error(const char str[]) {
  serial_puts(USART1, str);
  while (true)
    ;
}

int main() {
  rcc_clock_setup_in_hse_8mhz_out_72mhz();

  serial_begin(USART1, BAUD115K2);

  rcc_periph_clock_enable(RCC_GPIOB);
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN,
                GPIO0);
  gpio_set(GPIOB, GPIO0);
  rcc_periph_clock_enable(RCC_TIM2);
  timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
  timer_set_prescaler(TIM2, 71); // 72Mhz / 72 = 1Mhz => T = 1µs
  timer_set_period(TIM2, 0xFFFF);
  systick_set_frequency(1000, rcc_ahb_frequency);
  systick_interrupt_enable();
  systick_counter_enable();
  uint32_t now{};
  uint32_t tm{};
  uint32_t diff{};
  uint8_t dht_bytes[5]{};
  while (true) {
    serial_puts(USART1, "\r\nLectura:\r\n");
    timer_disable_counter(TIM2);
    timer_set_counter(TIM2, 0);
    timer_enable_counter(TIM2);
    gpio_clear(GPIOB, GPIO0);
    delay_ms(18);
    gpio_set(GPIOB, GPIO0);
    tm = timer_get_counter(TIM2);
    while (gpio_get(GPIOB, GPIO0)) // mientras esté en 1
      ;
    now = timer_get_counter(TIM2);
    diff = now - tm;
    tm = now;
    if (diff > 40)
      error("no contestó\n");
    while (!gpio_get(GPIOB, GPIO0)) // mientras esté en 0
      ;
    now = timer_get_counter(TIM2);
    diff = now - tm;
    tm = now;
    if (diff > 90)
      error("handshake error\n");
    while (gpio_get(GPIOB, GPIO0)) // mientras esté en 1
      ;
    now = timer_get_counter(TIM2);
    diff = now - tm;
    tm = now;
    if (diff > 90)
      error("handshake error\n");

    uint16_t bitcount{0};
    while (bitcount < 40) {
      while (!gpio_get(GPIOB, GPIO0)) // mientras esté en 0
        ;
      now = timer_get_counter(TIM2);
      diff = now - tm;
      tm = now;
      if (diff > 70)
        error("comm error en LOW\n");
      while (gpio_get(GPIOB, GPIO0)) // mientras esté en 1
        ;
      now = timer_get_counter(TIM2);
      diff = now - tm;
      tm = now;
      int actual_byte{bitcount / 8};
      int actual_bit{bitcount % 8};
      if (diff < 30) {
        dht_bytes[actual_byte] &= ~(1 << (7 - actual_bit));
      } else if (diff < 80) {
        dht_bytes[actual_byte] |= 1 << (7 - actual_bit);
      } else
        error("comm error en HIGH\n");
      bitcount++;
    }
    uint8_t chksum{static_cast<uint8_t>(dht_bytes[0] + dht_bytes[1] +
                                        dht_bytes[2] + dht_bytes[3])};
    if (chksum != dht_bytes[4])
      error("CHKSUM error\n");
    char str[80]{};
    uint16_t hum{(dht_bytes[0] * 256 + dht_bytes[1])};
    uint16_t tmp{(dht_bytes[2] * 256 + dht_bytes[3])};
    sprintf(str,
            "Hum: %d.%d%%\n"
            "Tmp: %d.%d°C\n",
            hum / 10, hum % 10, tmp / 10, tmp % 10); // dht22
    serial_puts(USART1, str);

    delay_ms(1000 * 5);
  }
}

void sys_tick_handler() { millis++; }
