// ialy cordeiro de sousa
// Atividade 06: Controle de PWM para LED e display LCD I2C

// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/semphr.h"
// #include "driver/gpio.h"
// #include "esp_timer.h"
// #include "int_i2c.h"
// #include "driver/gpio.h"
// #include "driver/ledc.h"
// #include "esp_log.h"
// #include "esp_timer.h"
// #include "hd44780.h"

// #define LED0 2
// #define LED1 4
// #define LED2 5
// #define LED3 18
// #define BTN_INC 12
// #define BTN_STEP 14
// #define LED_PWM 21
// #define I2C_SDA 6
// #define I2C_SCL 7
// #define I2C_PORT I2C_NUM_0
// #define DEBOUNCE_DELAY_MS 200
// static uint8_t counter = 0;
// static uint8_t step = 1;
// static int64_t last_press_inc = 0;
// static int64_t last_press_step = 0;
// SemaphoreHandle_t sem_inc;
// SemaphoreHandle_t sem_step;
// hd44780_t lcd;
// void update_leds(uint8_t value) {
//    gpio_set_level(LED0, value & 0x01);
//    gpio_set_level(LED1, (value >> 1) & 0x01);
//    gpio_set_level(LED2, (value >> 2) & 0x01);
//    gpio_set_level(LED3, (value >> 3) & 0x01);
// }
// void update_lcd(uint8_t value) {
//    char buf[16];
//    sprintf(buf, "Contador: %u", value);
//    hd44780_clear(&lcd);
//    hd44780_puts(&lcd, 0, 0, buf);
// }
// void update_pwm(uint8_t value) {
//    uint32_t duty = (value * 255) / 15;
//    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
//    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
// }
// static void IRAM_ATTR isr_inc_handler(void* arg) {
//    int64_t now = esp_timer_get_time();
//    if (now - last_press_inc > DEBOUNCE_DELAY_MS * 1000) {
//        last_press_inc = now;
//        xSemaphoreGiveFromISR(sem_inc, NULL);
//    }
// }
// static void IRAM_ATTR isr_step_handler(void* arg) {
//    int64_t now = esp_timer_get_time();
//    if (now - last_press_step > DEBOUNCE_DELAY_MS * 1000) {
//        last_press_step = now;
//        xSemaphoreGiveFromISR(sem_step, NULL);
//    }
// }
// void button_task(void* arg) {
//    while (true) {
//        if (xSemaphoreTake(sem_inc, portMAX_DELAY)) {
//            counter = (counter + step) & 0x0F;
//            update_leds(counter);
//            update_pwm(counter);
//            update_lcd(counter);
//        }
//        if (xSemaphoreTake(sem_step, 0)) {
//            step = (step == 1) ? 2 : 1;
//        }
//    }
// }
// void init_lcd() {
//    i2cdev_init();
//    memset(&lcd, 0, sizeof(hd44780_t));
//    lcd.addr = 0x27; // endereço I2C padrão do módulo
//    lcd.lines = 2;
//    lcd.cols = 16;
//    lcd.backlight = true;
//    ESP_ERROR_CHECK(hd44780_init(&lcd));
// }
// void init_pwm() {
//    ledc_timer_config_t timer = {
//        .speed_mode = LEDC_LOW_SPEED_MODE,
//        .timer_num = LEDC_TIMER_0,
//        .duty_resolution = LEDC_TIMER_8_BIT,
//        .freq_hz = 5000,
//        .clk_cfg = LEDC_AUTO_CLK
//    };
//    ledc_timer_config(&timer);
//    ledc_channel_config_t channel = {
//        .gpio_num = LED_PWM,
//        .speed_mode = LEDC_LOW_SPEED_MODE,
//        .channel = LEDC_CHANNEL_0,
//        .timer_sel = LEDC_TIMER_0,
//        .duty = 0,
//        .hpoint = 0
//    };
//    ledc_channel_config(&channel);
// }
// void app_main() {
//    // LEDs binários
//    gpio_config_t led_conf = {
//        .pin_bit_mask = (1ULL << LED0) | (1ULL << LED1) | (1ULL << LED2) | (1ULL << LED3),
//        .mode = GPIO_MODE_OUTPUT
//    };
//    gpio_config(&led_conf);
//    // Botões
//    gpio_config_t btn_conf = {
//        .pin_bit_mask = (1ULL << BTN_INC) | (1ULL << BTN_STEP),
//        .mode = GPIO_MODE_INPUT,
//        .pull_up_en = GPIO_PULLUP_ENABLE,
//        .pull_down_en = GPIO_PULLDOWN_DISABLE,
//        .intr_type = GPIO_INTR_NEGEDGE
//    };
//    gpio_config(&btn_conf);
//    // Semáforos
//    sem_inc = xSemaphoreCreateBinary();
//    sem_step = xSemaphoreCreateBinary();
//    // Interrupções
//    gpio_install_isr_service(0);
//    gpio_isr_handler_add(BTN_INC, isr_inc_handler, NULL);
//    gpio_isr_handler_add(BTN_STEP, isr_step_handler, NULL);
//    // Inicializações
//    init_lcd();
//    init_pwm();
//    update_leds(counter);
//    update_pwm(counter);
//    update_lcd(counter);
//    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);
// }

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "int_i2c.h" // Biblioteca fornecida pelo monitor
#include "rom/ets_sys.h"

// Pinos dos LEDs binários
#define LED0 2
#define LED1 4
#define LED2 5
#define LED3 18
#define LED_PWM 21

// Pinos dos botões
#define BTN_INC 12
#define BTN_STEP 14

// Pinos do I2C
#define I2C_SDA 6
#define I2C_SCL 7

#define DEBOUNCE_DELAY_MS 200

// Contador
static uint8_t counter = 0;
static uint8_t step = 1;
static int64_t last_press_inc = 0;
static int64_t last_press_step = 0;

// Semáforos para interrupções
SemaphoreHandle_t sem_inc;
SemaphoreHandle_t sem_step;

// LCD handle
lcd_i2c_handle_t lcd = {
    .address = 0x27,
    .num = I2C_NUM_0,
    .backlight = 1,
    .size = DISPLAY_16X02
};

void update_leds(uint8_t value) {
    gpio_set_level(LED0, value & 0x01);
    gpio_set_level(LED1, (value >> 1) & 0x01);
    gpio_set_level(LED2, (value >> 2) & 0x01);
    gpio_set_level(LED3, (value >> 3) & 0x01);
}

void update_pwm(uint8_t value) {
    uint32_t duty = (value * 255) / 15;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void update_lcd(uint8_t value) {
    lcd_i2c_write(&lcd, 0, CLEAR_DISPLAY);
    lcd_i2c_cursor_set(&lcd, 0, 0);
    lcd_i2c_print(&lcd, "Contador: %u", value);
}

static void IRAM_ATTR isr_inc_handler(void* arg) {
    int64_t now = esp_timer_get_time();
    if (now - last_press_inc > DEBOUNCE_DELAY_MS * 1000) {
        last_press_inc = now;
        xSemaphoreGiveFromISR(sem_inc, NULL);
        ets_printf("DEBUG: Botão INC pressionado\n");
    }
}

static void IRAM_ATTR isr_step_handler(void* arg) {
    int64_t now = esp_timer_get_time();
    if (now - last_press_step > DEBOUNCE_DELAY_MS * 1000) {
        last_press_step = now;
        xSemaphoreGiveFromISR(sem_step, NULL);
        ets_printf("DEBUG: Botão STEP pressionado\n");
    }
}


void button_task(void* arg) {
    while (true) {
        if (xSemaphoreTake(sem_inc, portMAX_DELAY)) {
            counter = (counter + step) & 0x0F;
            update_leds(counter);
            update_pwm(counter);
            update_lcd(counter);
        }
        if (xSemaphoreTake(sem_step, 0)) {
            step = (step == 1) ? 2 : 1;
        }
    }
}

void init_pwm() {
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t channel = {
        .gpio_num = LED_PWM,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel);
}

void init_i2c_lcd() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);

    lcd_i2c_init(&lcd);
    lcd_i2c_cursor_set(&lcd, 0, 0);
    lcd_i2c_print(&lcd, "Contador: %u", counter);
}

void app_main() {
    // LEDs binários
    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << LED0) | (1ULL << LED1) | (1ULL << LED2) | (1ULL << LED3),
        .mode = GPIO_MODE_OUTPUT
    };
    gpio_config(&led_conf);

    // Botões
    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << BTN_INC) | (1ULL << BTN_STEP),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&btn_conf);

    // Semáforos
    sem_inc = xSemaphoreCreateBinary();
    sem_step = xSemaphoreCreateBinary();

    // Interrupções
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN_INC, isr_inc_handler, NULL);
    gpio_isr_handler_add(BTN_STEP, isr_step_handler, NULL);

    // Inicializações
    init_pwm();
    init_i2c_lcd();

    update_leds(counter);
    update_pwm(counter);

    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);
}
