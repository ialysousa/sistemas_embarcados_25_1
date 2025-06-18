// ialy cordeiro de sousa
//Atividade 04: Controle de LED por botão

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#define btn1 GPIO_NUM_4
#define btn2 GPIO_NUM_1

#define LED0 GPIO_NUM_9
#define LED1 GPIO_NUM_11
#define LED2 GPIO_NUM_13
#define LED3 GPIO_NUM_14

#define DEBOUNCE_TIME_US 200000

uint8_t counter = 0;
uint8_t increment = 1;

int64_t last_press_a = 0;
int64_t last_press_b = 0;

void update_leds(uint8_t value) {
    gpio_set_level(LED0, value & 0x01);
    gpio_set_level(LED1, (value >> 1) & 0x01);
    gpio_set_level(LED2, (value >> 2) & 0x01);
    gpio_set_level(LED3, (value >> 3) & 0x01);
}

void init_gpio() {
    // LEDs como saída
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED0) | (1ULL << LED1) | (1ULL << LED2) | (1ULL << LED3),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Botões como entrada com pull-down
    io_conf.pin_bit_mask = (1ULL << btn1) | (1ULL << btn2);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);
}

void app_main() {
    init_gpio();
    update_leds(counter);

    last_press_a = esp_timer_get_time(); 
    last_press_b = esp_timer_get_time();
    printf(" %d \n", gpio_get_level(btn1));
    while (1) {
        int64_t now = esp_timer_get_time();

        // botao 1 faz a contagem
        if (gpio_get_level(btn1) == 1 && now - last_press_a > DEBOUNCE_TIME_US) {
          last_press_a = now;
          counter = (counter + increment) & 0x0F;
          update_leds(counter);
          printf("Botão A pressionado. Contador = %d (0x%X)\n", counter, counter);

        }
        
        // alternancia entre 1 e 2
        if (gpio_get_level(btn2) == 1 && now - last_press_b > DEBOUNCE_TIME_US) {
          last_press_b = now;
          increment = (increment == 1) ? 2 : 1;
          printf("Botão B pressionado. Incremento = %d\n", increment);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}
