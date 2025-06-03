#include <cstdio>
#include <cstdlib>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <usbserial.h>

volatile uint32_t millis{};
volatile uint32_t ttick{}; // para manejar las diferencias de tiempo en la lectura

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
volatile dht_error_state dht_error{NO_ERROR};
volatile dht_data_t dht_data{0, 0};

// Muestra los errores por el puerto serie:
void error(const char str[]) { usb_serial_puts(str); }

void dht_stand_by(uint32_t diff __unused) {}
void dht_data_ready(uint32_t diff __unused);
void dht_init_comm(uint32_t diff __unused);
void dht_errorf(uint32_t diff __unused);
void dht_wait_for_response(uint32_t diff);
void dht_handshake(uint32_t diff);
void dht_reading_data(uint32_t diff);

// puntero a función de prototipo void func(uint32_t var)
void (*dht_state_function)(uint32_t){dht_stand_by};

void dht_data_ready(uint32_t diff __unused) {
  char str[80]{};
  sprintf(str,
          "Hum: %d.%d%%\n"
          "Tmp: %d.%d°C\n",
          dht_data.hum / 10, dht_data.hum % 10, dht_data.tmp / 10,
          dht_data.tmp % 10); // dht22
  // Una vez leído el dato, se vuelve al estado STAND_BY
  dht_state_function = dht_stand_by;
  usb_serial_puts(str);
}

void dht_init_comm(uint32_t diff __unused) {
  static bool first_time{true};
  if (first_time) {
    gpio_clear(GPIOB, GPIO0);
    first_time = false;
  } else {
    gpio_set(GPIOB, GPIO0);
    ttick = timer_get_counter(TIM2);
    exti_enable_request(EXTI0);
    dht_state_function = dht_wait_for_response;
    first_time = true;
  }
}

void dht_wait_for_response(uint32_t diff) {
  if (!gpio_get(GPIOB, GPIO0)) { // si está en 0 flanco descendente:
    dht_state_function = (diff < 40) ? dht_handshake : dht_errorf;
  }
}

void dht_handshake(uint32_t diff) {
  if (gpio_get(GPIOB, GPIO0)) { // si está en 1 flanco ascendente:
    if (diff > 90) {
      dht_state_function = dht_errorf;
      dht_error = ERROR_NO_HANDSHAKE;
    }
  } else { // flanco descendente:
    if (diff > 90) {
      dht_state_function = dht_errorf;
      dht_error = ERROR_NO_HANDSHAKE;
    } else {
      dht_state_function = dht_reading_data;
    }
  }
}

void dht_reading_data(uint32_t diff) {
  static uint8_t dht_bytes[5]{};
  static int bitcount{0};
  if (gpio_get(GPIOB, GPIO0)) {
    if (diff > 70) {
      dht_state_function = dht_errorf;
      dht_error = ERROR_READING;
      bitcount = 0;
    }
  } else {
    int actual_byte{bitcount / 8};
    int actual_bit{bitcount % 8};
    if (diff < 30) {
      dht_bytes[actual_byte] &= ~(1 << (7 - actual_bit));
    } else if (diff < 80) {
      dht_bytes[actual_byte] |= 1 << (7 - actual_bit);
    } else {
      dht_state_function = dht_errorf;
      dht_error = ERROR_READING;
      bitcount = 0;
    }
    bitcount++;
    if (dht_state_function != dht_errorf && bitcount == 40) {
      uint8_t chksum{static_cast<uint8_t>(dht_bytes[0] + dht_bytes[1] + dht_bytes[2] + dht_bytes[3])};
      if (chksum == dht_bytes[4]) {
        dht_data.hum = static_cast<uint16_t>(dht_bytes[0] * 256 + dht_bytes[1]);
        dht_data.tmp = static_cast<uint16_t>(dht_bytes[2] * 256 + dht_bytes[3]);
        dht_state_function = dht_data_ready;
      } else {
        dht_state_function = dht_errorf;
        dht_error = ERROR_CHKSUM;
      }
      exti_disable_request(EXTI0);
      bitcount = 0;
    }
  }
}

void dht_errorf(uint32_t diff __unused) {
  // Se informa si hubo un error en la conversión:
  switch (dht_error) {
  case ERROR_NO_RESPONSE: error("No respondió\n"); break;
  case ERROR_NO_HANDSHAKE: error("Error en el Handshake\n"); break;
  case ERROR_READING: error("Error leyendo bits\n"); break;
  case ERROR_CHKSUM: error("Error de CHKSUM\n"); break;
  case ERROR_TIMEOUT: error("Time out\n"); break;
  case NO_ERROR: error("Ud no debería estar aquí\n"); break;
  }
  // Una vez informado el error, se "reinicia" la comunicación con dispositivo:
  dht_error = NO_ERROR;
  dht_state_function = dht_stand_by;
}
// Función para comenzar la comunicación
void dht_begin_comm() {
  if (dht_state_function == dht_stand_by) dht_state_function = dht_init_comm;
}


void delay_ms(uint32_t ms) {
  uint32_t tm{millis + ms};
  while (millis < tm);
}

int main() {
  rcc_clock_setup_in_hse_8mhz_out_72mhz();
  
  rcc_periph_clock_enable(RCC_AFIO);
  gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, 0); // JTAG off ‐ SW on

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
  bool start_conversion{true};
  uint32_t last_conversion_tick{0};
  usb_serial_begin();
  while (true) {
    usb_serial_poll();
    // Se espera a que haya terminado la conversión:
    if (start_conversion && (millis - last_conversion_tick) >= 5000) {
      usb_serial_puts("Iniciando conversión...\n");
      start_conversion = false;
      dht_begin_comm(); // inicia una nueva conversión del DHT22
    }
    if (dht_state_function == dht_data_ready) {
      dht_state_function(0);
      start_conversion = true; // se habilita la próxima conversión
      last_conversion_tick = millis;
    } else if (dht_state_function == dht_errorf) {
      dht_state_function(0);
      start_conversion = true; // se habilita la próxima conversión
      last_conversion_tick = millis;
    }
  }
}

void sys_tick_handler() { millis++; }
// Rutina de interrupción del timer
void tim2_isr() {
  timer_clear_flag(TIM2, TIM_SR_UIF);
  if (dht_state_function == dht_stand_by || dht_state_function == dht_data_ready) return;
  if (dht_state_function == dht_init_comm) {
    dht_state_function(0);
  } else {
    dht_state_function = dht_errorf;
    dht_error = ERROR_TIMEOUT;
  }
}
// Rutina de interrupción por flancos del EXTI0 (PB0 data pin del DHT22)
void exti0_isr() {
  exti_reset_request(EXTI0);
  uint32_t now{timer_get_counter(TIM2)};
  dht_state_function(now - ttick);
  ttick = now;
  if (dht_state_function == dht_errorf) exti_disable_request(EXTI0);
}
