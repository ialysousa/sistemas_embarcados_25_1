// ialy cordeiro de sousa
// Atividade 06: Controle de PWM para LED e display LCD I2C

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "int_i2c.h"

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_8_BIT
#define LEDC_FREQUENCY 5000
#define LED0 GPIO_NUM_9
#define LED1 GPIO_NUM_11
#define LED2 GPIO_NUM_13
#define LED3 GPIO_NUM_14
#define btn1 GPIO_NUM_4
#define btn2 GPIO_NUM_1
#define I2C_MASTER_SCL_IO  GPIO_NUM_47
#define I2C_MASTER_SDA_IO  GPIO_NUM_48
#define I2C_MASTER_NUM     I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define LCD_ADDRESS        0x27      
#define LCD_SIZE           DISPLAY_16X02
static const char *TAG = "BIN_COUNTER";
#define LED_PWM GPIO_NUM_37 
#define DEBOUNCE_TIME_US 200000

lcd_i2c_handle_t lcd;
uint8_t counter = 0;
uint8_t increment = 1;

int64_t last_press_a = 0;
int64_t last_press_b = 0;

volatile bool bnt_a_interrupt = false;
volatile bool bnt_b_interrupt = false;

void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}


static void IRAM_ATTR bnt_a_ISR_handler(void* arg) {
    gpio_intr_disable(btn1);
    bnt_a_interrupt = true;
}

static void IRAM_ATTR bnt_b_ISR_handler(void* arg) {
    gpio_intr_disable(btn2);
    bnt_b_interrupt = true;
}

void set_pwm_duty(uint32_t duty) {
  ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
  ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

void update_leds(uint8_t value) {
    gpio_set_level(LED0, value & 0x01);
    gpio_set_level(LED1, (value >> 1) & 0x01);
    gpio_set_level(LED2, (value >> 2) & 0x01);
    gpio_set_level(LED3, (value >> 3) & 0x01);

    uint32_t duty = (value & 0x0F) * (255 / 15);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}



void init_gpio() {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED0) | (1ULL << LED1) | (1ULL << LED2) | (1ULL << LED3),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << btn1) | (1ULL << btn2);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);

    
    gpio_set_intr_type(btn1, GPIO_INTR_NEGEDGE); 
    gpio_set_intr_type(btn2, GPIO_INTR_NEGEDGE); 
    gpio_install_isr_service(0);
    gpio_isr_handler_add(btn1, bnt_a_ISR_handler, NULL);
    gpio_isr_handler_add(btn2, bnt_b_ISR_handler, NULL);

    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 5000,                     
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .channel = LEDC_CHANNEL,
        .duty       = 0,
        .gpio_num   = LED_PWM,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel  = LEDC_TIMER_0
    };

    ledc_channel_config(&ledc_channel);
}


static void update_lcd_display(uint8_t count) {
    char buffer[20];
    
    lcd_i2c_write(&lcd, 0, CLEAR_DISPLAY);
    vTaskDelay(10 / portTICK_PERIOD_MS); 

    lcd_i2c_cursor_set(&lcd, 0, 0);
    snprintf(buffer, sizeof(buffer), "Bin: 0x%X", count);
    lcd_i2c_print(&lcd, buffer);
    
    lcd_i2c_cursor_set(&lcd, 0, 1);
    snprintf(buffer, sizeof(buffer), "Hex: %d", count);
    lcd_i2c_print(&lcd, buffer);
    
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

void app_main() {
  init_gpio();
  i2c_master_init();

  lcd.address = LCD_ADDRESS;
  lcd.num = I2C_MASTER_NUM;
  lcd.backlight = 1; 
  lcd.size = LCD_SIZE;
  lcd_i2c_init(&lcd);


  lcd_i2c_cursor_set(&lcd, 0, 0);
  lcd_i2c_print(&lcd, "Count");
  lcd_i2c_cursor_set(&lcd, 0, 1);
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  lcd_i2c_write(&lcd, 0, CLEAR_DISPLAY);
  
  update_lcd_display(counter);

  last_press_a = esp_timer_get_time(); 
  last_press_b = esp_timer_get_time();
    
    while (1) {
        int64_t now = esp_timer_get_time();

        if (bnt_a_interrupt) {
            if (now - last_press_a > DEBOUNCE_TIME_US) {
              last_press_a = now;
              counter = (counter + 1) & 0x0F;
              update_leds(counter);
              printf("Press buttom A. Increase = %d (0x%X)\n", counter, counter);
              
              update_lcd_display(counter);
            }
            bnt_a_interrupt = false;
            gpio_intr_enable(btn1);
        }

        if (bnt_b_interrupt) {
            if (now - last_press_b > DEBOUNCE_TIME_US) {
              last_press_b = now;
              last_press_a = now;
              counter = (counter - 1) & 0x0F;
              update_leds(counter);
              printf("Press buttom B. Decrease = %d (0x%X)\n", counter, counter);
              
              update_lcd_display(counter);



            }
            bnt_b_interrupt = false;
            gpio_intr_enable(btn2);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); 
  }
}


