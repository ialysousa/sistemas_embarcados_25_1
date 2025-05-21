// ialy cordeiro de sousa
// Atividade 05: Controle de LED por botão com interrupção e debounce

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#define LED0 2
#define LED1 4
#define LED2 5
#define LED3 18
#define BTN_INC 12
#define BTN_STEP 14
#define DEBOUNCE_DELAY_MS 200
static uint8_t counter = 0;
static uint8_t step = 1;
static int64_t last_press_inc = 0;
static int64_t last_press_step = 0;
SemaphoreHandle_t sem_inc;
SemaphoreHandle_t sem_step;
void update_leds(uint8_t value) {
   gpio_set_level(LED0, value & 0x01);
   gpio_set_level(LED1, (value >> 1) & 0x01);
   gpio_set_level(LED2, (value >> 2) & 0x01);
   gpio_set_level(LED3, (value >> 3) & 0x01);
}
// ISR do botão incremento
static void IRAM_ATTR isr_inc_handler(void* arg) {
   int64_t now = esp_timer_get_time();
   if (now - last_press_inc > DEBOUNCE_DELAY_MS * 1000) {
       last_press_inc = now;
       xSemaphoreGiveFromISR(sem_inc, NULL);
   }
}
// ISR do botão step
static void IRAM_ATTR isr_step_handler(void* arg) {
   int64_t now = esp_timer_get_time();
   if (now - last_press_step > DEBOUNCE_DELAY_MS * 1000) {
       last_press_step = now;
       xSemaphoreGiveFromISR(sem_step, NULL);
   }
}
// Task que trata eventos de botão
void button_task(void* arg) {
   while (true) {
       if (xSemaphoreTake(sem_inc, portMAX_DELAY)) {
           counter = (counter + step) & 0x0F;
           update_leds(counter);
       }
       if (xSemaphoreTake(sem_step, 0)) {
           step = (step == 1) ? 2 : 1;
       }
   }
}
void app_main() {
   // Configuração dos LEDs
   gpio_config_t led_conf = {
       .pin_bit_mask = (1ULL << LED0) | (1ULL << LED1) | (1ULL << LED2) | (1ULL << LED3),
       .mode = GPIO_MODE_OUTPUT,
       .intr_type = GPIO_INTR_DISABLE
   };
   gpio_config(&led_conf);
   // Configuração dos botões
   gpio_config_t btn_conf = {
       .pin_bit_mask = (1ULL << BTN_INC) | (1ULL << BTN_STEP),
       .mode = GPIO_MODE_INPUT,
       .pull_up_en = GPIO_PULLUP_ENABLE,
       .pull_down_en = GPIO_PULLDOWN_DISABLE,
       .intr_type = GPIO_INTR_NEGEDGE
   };
   gpio_config(&btn_conf);
   // Cria semáforos binários
   sem_inc = xSemaphoreCreateBinary();
   sem_step = xSemaphoreCreateBinary();
   // Instala serviço de interrupção
   gpio_install_isr_service(0);
   gpio_isr_handler_add(BTN_INC, isr_inc_handler, NULL);
   gpio_isr_handler_add(BTN_STEP, isr_step_handler, NULL);
   update_leds(counter);
   xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);
}
