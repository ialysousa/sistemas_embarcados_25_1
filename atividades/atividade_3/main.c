// Sistemas Embarcados 2025.1 - IFPB
// Ialy Cordeiro de Sousa - 201921250043
//Atividade 03: Controle de LED utilizando o ESP32 no Simulador Wokwi

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#define LED1_GPIO GPIO_NUM_18  // LED que pisca a cada 1000ms
#define LED2_GPIO GPIO_NUM_19  // LED que pisca a cada 200ms
void led1_task(void *pvParameter) {
   while (1) {
       gpio_set_level(LED1_GPIO, 1);
       vTaskDelay(pdMS_TO_TICKS(500));
       gpio_set_level(LED1_GPIO, 0);
       vTaskDelay(pdMS_TO_TICKS(500));
   }
}
void led2_task(void *pvParameter) {
   while (1) {
       gpio_set_level(LED2_GPIO, 1);
       vTaskDelay(pdMS_TO_TICKS(100));
       gpio_set_level(LED2_GPIO, 0);
       vTaskDelay(pdMS_TO_TICKS(100));
   }
}
void app_main(void)
{
   gpio_set_direction(LED1_GPIO, GPIO_MODE_OUTPUT);
   gpio_set_direction(LED2_GPIO, GPIO_MODE_OUTPUT);
   xTaskCreate(led1_task, "LED1 Task", 2048, NULL, 5, NULL);
   xTaskCreate(led2_task, "LED2 Task", 2048, NULL, 5, NULL);
}
