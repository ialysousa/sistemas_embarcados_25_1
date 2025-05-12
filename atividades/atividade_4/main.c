#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

// LEDs
#define LED0 2
#define LED1 4
#define LED2 5
#define LED3 18

// Botões
#define BTN_INC 12
#define BTN_DEC 14

void app_main() {
    // Inicializa GPIOs dos LEDs
    gpio_set_direction(LED0, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED1, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED2, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED3, GPIO_MODE_OUTPUT);

    // Inicializa GPIOs dos botões como entrada
    gpio_set_direction(BTN_INC, GPIO_MODE_INPUT);
    gpio_set_direction(BTN_DEC, GPIO_MODE_INPUT);

    int counter = 0;
    bool btnIncLast = false;
    bool btnDecLast = false;

    while (1) {
        bool btnInc = gpio_get_level(BTN_INC);
        bool btnDec = gpio_get_level(BTN_DEC);

        if (btnInc && !btnIncLast) {
            counter = (counter + 1) % 16;
            vTaskDelay(pdMS_TO_TICKS(200)); // debounce
        }

        if (btnDec && !btnDecLast) {
            counter = (counter - 1 + 16) % 16;
            vTaskDelay(pdMS_TO_TICKS(200)); // debounce
        }

        btnIncLast = btnInc;
        btnDecLast = btnDec;

        gpio_set_level(LED0, counter & 0x01);
        gpio_set_level(LED1, (counter >> 1) & 0x01);
        gpio_set_level(LED2, (counter >> 2) & 0x01);
        gpio_set_level(LED3, (counter >> 3) & 0x01);

        vTaskDelay(pdMS_TO_TICKS(10)); // loop mais leve
    }
}
