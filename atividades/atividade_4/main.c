// ialy cordeiro de sousa
//Atividade 04: Controle de LED por botão


#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#define LED0 2
#define LED1 4
#define LED2 5
#define LED3 18
#define BTN_INC 12
#define BTN_STEP 14
#define DEBOUNCE_TIME_US 200000  // 200 ms debounce
static uint8_t counter = 0;
static uint8_t step = 1;
static int64_t last_inc_time = 0;
static int64_t last_step_time = 0;
static bool last_btn_inc = false;
static bool last_btn_step = false;
void update_leds(uint8_t value) {
   gpio_set_level(LED0, value & 0x01);
   gpio_set_level(LED1, (value >> 1) & 0x01);
   gpio_set_level(LED2, (value >> 2) & 0x01);
   gpio_set_level(LED3, (value >> 3) & 0x01);
}
void app_main() {
   // Configuração dos LEDs
   gpio_config_t led_conf = {
       .pin_bit_mask = (1ULL << LED0) | (1ULL << LED1) | (1ULL << LED2) | (1ULL << LED3),
       .mode = GPIO_MODE_OUTPUT,
       .intr_type = GPIO_INTR_DISABLE,
   };
   gpio_config(&led_conf);
   // Configuração dos botões
   gpio_config_t btn_conf = {
       .pin_bit_mask = (1ULL << BTN_INC) | (1ULL << BTN_STEP),
       .mode = GPIO_MODE_INPUT,
       .pull_up_en = GPIO_PULLUP_ENABLE,
       .pull_down_en = GPIO_PULLDOWN_DISABLE,
       .intr_type = GPIO_INTR_DISABLE,
   };
   gpio_config(&btn_conf);
   update_leds(counter);
   while (true) {
       int64_t now = esp_timer_get_time();
       bool btn_inc_state = !gpio_get_level(BTN_INC);    // Pressionado = 1
       bool btn_step_state = !gpio_get_level(BTN_STEP);  // Pressionado = 1
       // Detecção de borda de subida 
       if (btn_inc_state && !last_btn_inc && (now - last_inc_time > DEBOUNCE_TIME_US)) {
           counter = (counter + step) & 0x0F;
           update_leds(counter);
           last_inc_time = now;
       }
       if (btn_step_state && !last_btn_step && (now - last_step_time > DEBOUNCE_TIME_US)) {
           step = (step == 1) ? 2 : 1;
           last_step_time = now;
       }
       last_btn_inc = btn_inc_state;
       last_btn_step = btn_step_state;
   }
}
