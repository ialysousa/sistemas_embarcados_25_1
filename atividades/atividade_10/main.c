// ialy cordeiro de sousa


 //  Projeto Atividade 10 FreeRTOS 

#include <stdio.h>
#include <math.h>
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "int_i2c.h"
#include "esp_log.h"
#include "driver/sdspi_host.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

// Pin definitions
#define BTN_A         GPIO_NUM_4
#define BTN_B         GPIO_NUM_1
#define BUZZER_PIN    GPIO_NUM_37
#define SEG_LED0      GPIO_NUM_9
#define SEG_LED1      GPIO_NUM_8
#define SEG_LED2      GPIO_NUM_3
#define SEG_LED3      GPIO_NUM_14
#define SENSOR_ADC    ADC1_CHANNEL_6
#define I2C_SCL       GPIO_NUM_47
#define I2C_SDA       GPIO_NUM_48
#define LCD_ADDRESS   0x27
#define SD_CS         GPIO_NUM_10
#define SD_MOSI       GPIO_NUM_11
#define SD_MISO       GPIO_NUM_12
#define SD_SCLK       GPIO_NUM_13

static const char* TAG = "FreeRTOSApp";

// Shared data
static volatile int current_temp = 0;
static volatile int alarm_temp = 25;
static volatile bool alarm_active = false;

// Debounce
#define DEBOUNCE_US 200000
static int64_t last_button_a = 0;
static int64_t last_button_b = 0;
static volatile bool button_a_flag = false;
static volatile bool button_b_flag = false;

// Function prototypes
void init_hardware(void);
float read_temperature(void);
void buzzer_init(void);
void lcd_task(void*);
void button_task(void*);
void temp_task(void*);
void alarm_task(void*);
void leds_task(void*);
void sd_task(void*);

// ISR handlers
static void IRAM_ATTR btn_a_isr(void* arg) {
    button_a_flag = true;
}
static void IRAM_ATTR btn_b_isr(void* arg) {
    button_b_flag = true;
}

void app_main() {
    init_hardware();

    xTaskCreate(temp_task, "TempTask", 2048, NULL, 5, NULL);
    xTaskCreate(button_task, "BtnTask", 2048, NULL, 6, NULL);
    xTaskCreate(alarm_task, "AlarmTask", 2048, NULL, 7, NULL);
    xTaskCreate(leds_task, "LEDsTask", 2048, NULL, 4, NULL);
    xTaskCreate(lcd_task, "LCDTask", 4096, NULL, 3, NULL);
    xTaskCreate(sd_task,  "SDTask", 4096, NULL, 2, NULL);
}

void init_hardware() {
    // ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(SENSOR_ADC, ADC_ATTEN_DB_11);
    // Buttons
    gpio_config_t io_conf = { .pin_bit_mask = (1ULL<<BTN_A)|(1ULL<<BTN_B),
                              .mode = GPIO_MODE_INPUT,
                              .pull_up_en = GPIO_PULLUP_ENABLE,
                              .intr_type = GPIO_INTR_NEGEDGE };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN_A, btn_a_isr, NULL);
    gpio_isr_handler_add(BTN_B, btn_b_isr, NULL);
    // PWM Buzzer
    buzzer_init();
    // LEDs segments
    io_conf.pin_bit_mask = (1ULL<<SEG_LED0)|(1ULL<<SEG_LED1)|(1ULL<<SEG_LED2)|(1ULL<<SEG_LED3);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
    // I2C LCD
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    i2c_param_config(I2C_NUM_0, &i2c_conf);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    lcd_i2c_handle_t lcd = { .address = LCD_ADDRESS, .num = I2C_NUM_0, .size = DISPLAY_16X02, .backlight = 1 };
    lcd_i2c_init(&lcd);
    // SDCard mount
    sdmmc_host_t host = SDSPI_HOST_DEFAULT(); host.slot = SPI2_HOST;
    spi_bus_config_t bus_cfg = { .mosi_io_num=SD_MOSI, .miso_io_num=SD_MISO, .sclk_io_num=SD_SCLK };
    spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    sdspi_device_config_t slot_cfg = SDSPI_DEVICE_CONFIG_DEFAULT(); slot_cfg.gpio_cs = SD_CS;
    esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_cfg, NULL, NULL);
}

float read_temperature() {
    int raw = adc1_get_raw(SENSOR_ADC);
    float resistance = 10000.0/((4095.0/raw)-1.0);
    const float BETA = 3950.0;
    float celsius = 1.0/(log(resistance/10000.0)/BETA + 1.0/298.15) - 273.15;
    return celsius;
}

void buzzer_init() {
    ledc_timer_config_t timer = { .speed_mode=LEDC_LOW_SPEED_MODE, .timer_num=LEDC_TIMER_0,
                                  .duty_resolution=LEDC_TIMER_8_BIT, .freq_hz=5000 };
    ledc_timer_config(&timer);
    ledc_channel_config_t ch = { .channel=LEDC_CHANNEL_0, .duty=0, .gpio_num=BUZZER_PIN,
                                  .speed_mode=LEDC_LOW_SPEED_MODE, .timer_sel=LEDC_TIMER_0 };
    ledc_channel_config(&ch);
}

void temp_task(void* pv) {
    while(1) {
        float t = read_temperature();
        current_temp = (int)t;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void button_task(void* pv) {
    while(1) {
        int64_t now = esp_timer_get_time();
        if(button_a_flag && now - last_button_a > DEBOUNCE_US) {
            alarm_temp += 5;
            last_button_a = now; button_a_flag = false;
        }
        if(button_b_flag && now - last_button_b > DEBOUNCE_US) {
            alarm_temp -= 5;
            last_button_b = now; button_b_flag = false;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void alarm_task(void* pv) {
    while(1) {
        if(current_temp >= alarm_temp) {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 128);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            alarm_active = true;
        } else {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            alarm_active = false;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void leds_task(void* pv) {
    while(1) {
        // logic to light segment LEDs based on proximity
        int diff = alarm_temp - current_temp;
        gpio_set_level(SEG_LED0, (diff<=20));
        gpio_set_level(SEG_LED1, (diff<=15));
        gpio_set_level(SEG_LED2, (diff<=10));
        gpio_set_level(SEG_LED3, (diff<=2));
        if(alarm_active) {
            bool on = ((esp_timer_get_time()/500000)%2);
            gpio_set_level(SEG_LED0, on);
            gpio_set_level(SEG_LED1, on);
            gpio_set_level(SEG_LED2, on);
            gpio_set_level(SEG_LED3, on);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void lcd_task(void* pv) {
    lcd_i2c_handle_t lcd = { .address=LCD_ADDRESS, .num=I2C_NUM_0, .size=DISPLAY_16X02, .backlight=1 };
    while(1) {
        char buf[20];
        lcd_i2c_cursor_set(&lcd,0,0);
        snprintf(buf,sizeof(buf),"Temp:%d C", current_temp);
        lcd_i2c_print(&lcd, buf);
        lcd_i2c_cursor_set(&lcd,0,1);
        snprintf(buf,sizeof(buf),"Alarm:%d C", alarm_temp);
        lcd_i2c_print(&lcd, buf);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void sd_task(void* pv) {
    FILE* f;
    while(1) {
        f = fopen("/sdcard/temp_log.txt","a");
        if(f) {
            fprintf(f, "%lld,%d\n", esp_timer_get_time(), current_temp);
            fclose(f);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
