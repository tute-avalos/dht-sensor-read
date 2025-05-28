#include <cstdio>
#include <cstdlib>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <serial.h>

volatile uint32_t millis{};
volatile uint32_t ttick{}; // para manejar las diferencias de tiempo en la lectura

// Estado de la comunicación con el DHT22
enum dht_comm_state {
  STAND_BY,          // no inició la comunicación
  INIT_COMM_LOW,     // El micro pone la linea en LOW
  INIT_COMM_HIGH,    // El micro pone la linea en HIGH luego de los 18ms
  WAIT_FOR_RESPONSE, // Se espera la respuesta del DHT22
  HANDSHAKE,         // Se maneja el HANDSHAKE (80ms LOW y 80ms HIGH)
  READING_DATA,      // Se leen los 40 bits y verificamos el CHKSUM
  DATA_READY,        // Los datos ya se leyeron y están disponibles
  ERROR              // Ocurrió un error
};
// Estado de error (qué fallo?)
enum dht_error_state {
  NO_ERROR,
  ERROR_NO_RESPONSE,
  ERROR_TIMEOUT,
  ERROR_NO_HANDSHAKE,
  ERROR_READING,
  ERROR_CHKSUM
};
// Datos del DHT22/11
struct dht_data_t {
  uint16_t hum;
  uint16_t tmp;
};
// Variables que manejan los estados y datos del DHT22
volatile dht_comm_state dht_state{STAND_BY};
volatile dht_error_state dht_error{NO_ERROR};
volatile uint8_t dht_bytes[5]{};
volatile dht_data_t dht_data{0, 0};

// Muestra los errores por el puerto serie:
void error(const char str[]) { serial_puts(USART1, str); }

// Función para comenzar la comunicación
void dht_begin_comm() {
  if (dht_state == STAND_BY) {
    dht_state = INIT_COMM_LOW;
  }
}

// Rutina de interrupción del timer
void tim2_isr() {
  timer_clear_flag(TIM2, TIM_SR_UIF);
  if (dht_state == STAND_BY || dht_state == DATA_READY) return;
  if (dht_state == INIT_COMM_LOW) {
    gpio_clear(GPIOB, GPIO0);
    dht_state = INIT_COMM_HIGH;
  } else if (dht_state == INIT_COMM_HIGH) {
    gpio_set(GPIOB, GPIO0);
    ttick = timer_get_counter(TIM2);
    exti_enable_request(EXTI0);
    dht_state = WAIT_FOR_RESPONSE;
  } else {
    dht_state = ERROR;
    dht_error = ERROR_TIMEOUT;
  }
}
// Rutina de interrupción por flancos del EXTI0 (PB0 data pin del DHT22)
void exti0_isr() {
  static uint32_t now{};
  static uint16_t bitcount{0};
  exti_reset_request(EXTI0);
  switch (dht_state) {
  case WAIT_FOR_RESPONSE:
    if (!gpio_get(GPIOB, GPIO0)) { // si está en 0 flanco descendente:
      now = timer_get_counter(TIM2);
      dht_state = ((now - ttick) < 40) ? HANDSHAKE : ERROR;
      ttick = now;
      if (dht_state) dht_error = ERROR_NO_RESPONSE;
    }
    break;
  case HANDSHAKE:
    now = timer_get_counter(TIM2);
    if (gpio_get(GPIOB, GPIO0)) { // si está en 1 flanco ascendente:
      if ((now - ttick) > 90) {
        dht_state = ERROR;
        dht_error = ERROR_NO_HANDSHAKE;
      }
      ttick = now;
    } else { // flanco descendente:
      if ((now - ttick) > 90) {
        dht_state = ERROR;
        dht_error = ERROR_NO_HANDSHAKE;
      }
      ttick = now;
      dht_state = READING_DATA;
      bitcount = 0;
    }
    break;
  case READING_DATA:
    now = timer_get_counter(TIM2);
    if (gpio_get(GPIOB, GPIO0)) {
      if ((now - ttick) > 70) {
        dht_state = ERROR;
        dht_error = ERROR_READING;
      }
      ttick = now;
    } else {
      uint32_t diff = now - ttick;
      ttick = now;
      int actual_byte{bitcount / 8};
      int actual_bit{bitcount % 8};
      if (diff < 30) {
        dht_bytes[actual_byte] &= ~(1 << (7 - actual_bit));
      } else if (diff < 80) {
        dht_bytes[actual_byte] |= 1 << (7 - actual_bit);
      } else {
        dht_state = ERROR;
        dht_error = ERROR_READING;
      }
      bitcount++;
      if (dht_state != ERROR && bitcount == 40) {
        uint8_t chksum{static_cast<uint8_t>(dht_bytes[0] + dht_bytes[1] + dht_bytes[2] + dht_bytes[3])};
        if (chksum == dht_bytes[4]) {
          dht_data.hum = static_cast<uint16_t>(dht_bytes[0] * 256 + dht_bytes[1]);
          dht_data.tmp = static_cast<uint16_t>(dht_bytes[2] * 256 + dht_bytes[3]);
          dht_state = DATA_READY;
        } else {
          dht_state = ERROR;
          dht_error = ERROR_CHKSUM;
        }
      }
    }
    break;
  default: break;
  }
  if (dht_state == ERROR) exti_disable_request(EXTI0);
}

void delay_ms(uint32_t ms) {
  uint32_t tm{millis + ms};
  while (millis < tm);
}

int main() {
  rcc_clock_setup_in_hse_8mhz_out_72mhz();

  rcc_periph_clock_enable(RCC_AFIO);
  gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, 0); // JTAG off ‐ SW on

  serial_begin(USART1, BAUD115K2);
  nvic_set_priority(NVIC_USART1_IRQ, 3); // Se le baja la prioridad al puerto serie

  // Configuración del data pin del DHT22
  rcc_periph_clock_enable(RCC_GPIOB);
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, GPIO0);
  gpio_set(GPIOB, GPIO0);

  // Configuración del Timer2 cada 18ms (tiempo para empezar la comunicación del DHT22)
  rcc_periph_clock_enable(RCC_TIM2);
  timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
  // Cuenta ascendente:
  timer_set_prescaler(TIM2, 71);        // 72Mhz / 72 = 1Mhz => T = 1µs
  timer_set_period(TIM2, 17999);        // a los 18ms
  nvic_set_priority(NVIC_TIM2_IRQ, 1);  // Se le da más prioridad que al puerto serie
  timer_enable_irq(TIM2, TIM_DIER_UIE); // interrupción por desborde del timer
  nvic_enable_irq(NVIC_TIM2_IRQ);       // habilitación de la interrupción
  timer_enable_counter(TIM2);           // Arranca el timer

  // Configuración de la interrupción del pin de datos del DHT22
  exti_select_source(EXTI0, GPIOB);           // pin PB0 (dht22 data)
  exti_set_trigger(EXTI0, EXTI_TRIGGER_BOTH); // Interrupción en ambos flancos
  nvic_set_priority(NVIC_EXTI0_IRQ, 0);       // La mayor prioridad
  nvic_enable_irq(NVIC_EXTI0_IRQ);            // Se habilita la interrupción en el NVIC
  // No se habilitó la interrupción del EXTI0 aún, eso se hará cuando se inicie la comunicación.

  // Systick para delays bloqueantes de 1ms
  systick_set_frequency(1000, rcc_ahb_frequency);
  systick_interrupt_enable();
  systick_counter_enable();

  dht_begin_comm(); // inicia una conversión del DHT22
  while (true) {
    // Se espera a que haya terminado la conversión:
    if (dht_state == DATA_READY) {
      char str[80]{};
      sprintf(str,
              "Hum: %d.%d%%\n"
              "Tmp: %d.%d°C\n",
              dht_data.hum / 10, dht_data.hum % 10, dht_data.tmp / 10,
              dht_data.tmp % 10); // dht22
      // Una vez leído el dato, se vuelve al estado STAND_BY
      dht_state = STAND_BY;
      serial_puts(USART1, str);
      // Se pide otra conversión en 5 segundos:
      delay_ms(1000 * 5);
      dht_begin_comm();
    } else if (dht_state == ERROR) {
      // Se informa si hubo un error en la conversión:
      switch (dht_error) {
      case ERROR_NO_RESPONSE: serial_puts(USART1, "No respondió\n"); break;
      case ERROR_NO_HANDSHAKE: serial_puts(USART1, "Error en el Handshake\n"); break;
      case ERROR_READING: serial_puts(USART1, "Error leyendo bits\n"); break;
      case ERROR_CHKSUM: serial_puts(USART1, "Error de CHKSUM\n"); break;
      case ERROR_TIMEOUT: serial_puts(USART1, "Time out\n"); break;
      case NO_ERROR: serial_puts(USART1, "Ud no debería estar aquí\n"); break;
      }
      // Una vez informado el error, se "reinicia" la comunicación con dispositivo:
      dht_error = NO_ERROR;
      dht_state = STAND_BY;
      delay_ms(1000 * 5);
      dht_begin_comm();
    }
  }
}

void sys_tick_handler() { millis++; }
