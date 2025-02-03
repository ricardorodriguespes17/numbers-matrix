#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "pico/bootrom.h"
#include "pico/stdlib.h"

// arquivo .pio
#include "pio_matrix.pio.h"

// número de LEDs
#define NUM_PIXELS 25
// pino de saída
#define OUT_PIN 7
#define BUTTON_PIN_A 5
#define BUTTON_PIN_B 6
// define o numero do pino do led vermelho
#define RED_LED_PIN 13
// tempo de espera entre as piscadas do led vermelho
#define INTERVAL_DURATION_RED_LED 5000

void define_all_components();
void make_number();
void turn_off_red_led();
void turn_on_red_led();
static void gpio_irq_handler(uint gpio, uint32_t events);
uint32_t matrix_rgb(double b, double r, double g);
void draw_in_matrix(double* draw);

// configurações da PIO
PIO pio = pio0;
bool ok;
uint16_t i;
double r = 0.0, b = 0.0, g = 0.0;
uint offset;
uint sm;

static volatile uint current_number = 0;
static volatile uint32_t last_time = 0;

// vetor para criar imagem na matriz de led - 1
double matrix_draws[10][NUM_PIXELS] = {
    {
        // numero 0
        0.3, 0.3, 0.3, 0.3, 0.3,  //
        0.3, 0.0, 0.0, 0.0, 0.3,  //
        0.3, 0.0, 0.0, 0.0, 0.3,  //
        0.3, 0.0, 0.0, 0.0, 0.3,  //
        0.3, 0.3, 0.3, 0.3, 0.3   //
    },
    {
        // numero 1
        0.0, 0.0, 0.3, 0.0, 0.0,  //
        0.0, 0.0, 0.3, 0.3, 0.0,  //
        0.0, 0.0, 0.3, 0.0, 0.0,  //
        0.0, 0.0, 0.3, 0.0, 0.0,  //
        0.0, 0.0, 0.3, 0.0, 0.0   //
    },
    {
        // numero 2
        0.3, 0.3, 0.3, 0.3, 0.3,  //
        0.3, 0.0, 0.0, 0.0, 0.0,  //
        0.3, 0.3, 0.3, 0.3, 0.3,  //
        0.0, 0.0, 0.0, 0.0, 0.3,  //
        0.3, 0.3, 0.3, 0.3, 0.3   //
    },
    {
        // numero 3
        0.3, 0.3, 0.3, 0.3, 0.3,  //
        0.3, 0.0, 0.0, 0.0, 0.0,  //
        0.3, 0.3, 0.3, 0.3, 0.3,  //
        0.3, 0.0, 0.0, 0.0, 0.0,  //
        0.3, 0.3, 0.3, 0.3, 0.3   //
    },
    {
        // numero 4
        0.3, 0.0, 0.0, 0.0, 0.3,  //
        0.3, 0.0, 0.0, 0.0, 0.3,  //
        0.3, 0.3, 0.3, 0.3, 0.3,  //
        0.3, 0.0, 0.0, 0.0, 0.0,  //
        0.0, 0.0, 0.0, 0.0, 0.3   //
    },
    {
        // numero 5
        0.3, 0.3, 0.3, 0.3, 0.3,  //
        0.0, 0.0, 0.0, 0.0, 0.3,  //
        0.3, 0.3, 0.3, 0.3, 0.3,  //
        0.3, 0.0, 0.0, 0.0, 0.0,  //
        0.3, 0.3, 0.3, 0.3, 0.3   //
    },
    {
        // numero 6
        0.3, 0.3, 0.3, 0.3, 0.3,  //
        0.0, 0.0, 0.0, 0.0, 0.3,  //
        0.3, 0.3, 0.3, 0.3, 0.3,  //
        0.3, 0.0, 0.0, 0.0, 0.3,  //
        0.3, 0.3, 0.3, 0.3, 0.3   //
    },
    {
        // numero 7
        0.3, 0.3, 0.3, 0.3, 0.3,  //
        0.3, 0.0, 0.0, 0.0, 0.0,  //
        0.0, 0.0, 0.0, 0.0, 0.3,  //
        0.3, 0.0, 0.0, 0.0, 0.0,  //
        0.0, 0.0, 0.0, 0.0, 0.3   //
    },
    {
        // numero 8
        0.3, 0.3, 0.3, 0.3, 0.3,  //
        0.3, 0.0, 0.0, 0.0, 0.3,  //
        0.3, 0.3, 0.3, 0.3, 0.3,  //
        0.3, 0.0, 0.0, 0.0, 0.3,  //
        0.3, 0.3, 0.3, 0.3, 0.3   //
    },
    {
        // numero 9
        0.3, 0.3, 0.3, 0.3, 0.3,  //
        0.3, 0.0, 0.0, 0.0, 0.3,  //
        0.3, 0.3, 0.3, 0.3, 0.3,  //
        0.3, 0.0, 0.0, 0.0, 0.0,  //
        0.3, 0.3, 0.3, 0.3, 0.3   //
    },
};

// função principal
int main() {
  define_all_components();

  // coloca a frequência de clock para 128 MHz, facilitando a divisão pelo clock
  ok = set_sys_clock_khz(128000, false);
  // configurações da PIO
  uint offset = pio_add_program(pio, &pio_matrix_program);
  uint sm = pio_claim_unused_sm(pio, true);
  pio_matrix_program_init(pio, sm, offset, OUT_PIN);

  // interrupção da gpio habilitada
  gpio_set_irq_enabled_with_callback(BUTTON_PIN_A, GPIO_IRQ_EDGE_FALL, 1,
                                     &gpio_irq_handler);
  gpio_set_irq_enabled_with_callback(BUTTON_PIN_B, GPIO_IRQ_EDGE_FALL, 1,
                                     &gpio_irq_handler);

  turn_on_red_led();

  while (true) {
    draw_number();
    sleep_ms(100);
  }
}

void turn_on_red_led() {
  gpio_put(RED_LED_PIN, 1);
  add_alarm_in_ms(INTERVAL_DURATION_RED_LED, turn_off_red_led, NULL, false);
}

void turn_off_red_led() {
  gpio_put(RED_LED_PIN, 0);
  add_alarm_in_ms(INTERVAL_DURATION_RED_LED, turn_on_red_led, NULL, false);
}

void draw_number() { draw_in_matrix(matrix_draws[current_number]); }

// rotina da interrupção
static void gpio_irq_handler(uint gpio, uint32_t events) {
  uint32_t current_time = to_us_since_boot(get_absolute_time());

  if (current_time - last_time > 200000) {
    last_time = current_time;
    if (gpio == BUTTON_PIN_A) {
      current_number = (current_number + 1) %
                       10;  // Incrementa o número e mantém no intervalo 0-9
    } else if (gpio == BUTTON_PIN_B) {
      current_number = (current_number - 1 + 10) %
                       10;  // Decrementa o número e mantém no intervalo 0-9
    }
  }
}

// define os pinos e os tipos dos componentes utilizados
void define_all_components() {
  // inicializar o led vermelho - GPIO13
  gpio_init(RED_LED_PIN);
  gpio_set_dir(RED_LED_PIN, GPIO_OUT);

  // inicializar o botão de interrupção - GPIO5
  gpio_init(BUTTON_PIN_A);
  gpio_set_dir(BUTTON_PIN_A, GPIO_IN);
  gpio_pull_up(BUTTON_PIN_A);

  // inicializar o botão de interrupção - GPIO6
  gpio_init(BUTTON_PIN_B);
  gpio_set_dir(BUTTON_PIN_B, GPIO_IN);
  gpio_pull_up(BUTTON_PIN_B);
}

// rotina para definição da intensidade de cores do led
uint32_t matrix_rgb(double b, double r, double g) {
  unsigned char R, G, B;
  R = r * 255;
  G = g * 255;
  B = b * 255;
  return (G << 24) | (R << 16) | (B << 8);
}

// rotina para acionar a matrix de leds - ws2812b
void draw_in_matrix(double* draw) {
  uint32_t led_value;
  for (int16_t i = 0; i < NUM_PIXELS; i++) {
    int index = 24 - i;
    double pixel = draw[index];
    led_value = matrix_rgb(0.0, draw[24 - i], 0.0);
    pio_sm_put_blocking(pio, sm, led_value);
  }
}
