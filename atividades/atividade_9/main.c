// ialy cordeiro de sousa

  // Projeto Atividade 09 máquina de estados

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
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"

typedef enum {
    ESTADO_INICIALIZACAO,
    ESTADO_VERIFICAR_TEMPERATURA,
    ESTADO_ATUALIZAR_LCD,
    ESTADO_TRATAR_BOTOES,
    ESTADO_ATUALIZAR_LEDS,
    ESTADO_TRATAR_ALARME
} estado_t;


estado_t estado_atual = ESTADO_INICIALIZACAO;

// Saidas
#define btn1 GPIO_NUM_4
#define btn2 GPIO_NUM_1
#define BUZZER GPIO_NUM_37
#define LED0 GPIO_NUM_9
#define LED1 GPIO_NUM_8
#define LED2 GPIO_NUM_3
#define LED3 GPIO_NUM_14

// NTC
#define SENSOR_GPIO 1
#define SENSOR_ADC ADC1_CHANNEL_6
#define MAX_ADC 4095.0
#define T0_KELVIN 298.15
int tempdef = 25;
int ntc = 20;

//PWM
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_8_BIT
#define LEDC_FREQUENCY 2000

// Display LCD
#define I2C_MASTER_SCL_IO  GPIO_NUM_47
#define I2C_MASTER_SDA_IO  GPIO_NUM_48
#define I2C_MASTER_NUM     I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define LCD_ADDRESS        0x27      
#define LCD_SIZE           DISPLAY_16X02

lcd_i2c_handle_t lcd;
//static const char *TAG = "BIN_COUNTER";

// Definições de Pinos SD
#define SD_MNT_POINT "/files"
#define MISO GPIO_NUM_12 //
#define SCLK GPIO_NUM_13 //sck
#define MOSI GPIO_NUM_11 //di
#define CS   GPIO_NUM_10 //cs

#define DEBOUNCE_TIME_US 200000

//Interrupção
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

float ler_temp() {
    int adc = adc1_get_raw(SENSOR_ADC);
    float resistencia = 10000.0 / ((MAX_ADC / (float)adc) - 1.0);
    const float BETA = 3950; 
    float celsius = 1 / (log(resistencia / 10000.0) / BETA + 1.0 / 298.15) - 273.15;
    printf("%f",celsius);
    return celsius;

}

void update_leds(int temp, int alarme, bool alarme_ativo) {
    bool ligar = false;

    if (alarme_ativo) {
        if ((esp_timer_get_time() / 500000) % 2) {
            ligar = true;
        } else {
            ligar = false;
        }
    }

    if (ntc >= alarme) {
        gpio_set_level(LED0, ligar);
        gpio_set_level(LED1, ligar);
        gpio_set_level(LED2, ligar);
        gpio_set_level(LED3, ligar);
        vTaskDelay(20/ portTICK_PERIOD_MS);

    } 
      else {
        if (alarme - temp <= 20) {
            gpio_set_level(LED0, true);
        } else {
            gpio_set_level(LED0, false);
        }

        if (alarme - temp <= 15) {
            gpio_set_level(LED1, true);
        } else {
            gpio_set_level(LED1, false);
        }

        if (alarme - temp <= 10) {
            gpio_set_level(LED2, true);
        } else {
            gpio_set_level(LED2, false);
        }

        if (alarme - temp <= 2) {
            gpio_set_level(LED3, true);
        } else {
            gpio_set_level(LED3, false);
        }
    }
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

    // Botões como entrada
    io_conf.pin_bit_mask = (1ULL << btn1) | (1ULL << btn2);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);

    // Interrupções
    gpio_set_intr_type(btn1, GPIO_INTR_NEGEDGE); 
    gpio_set_intr_type(btn2, GPIO_INTR_NEGEDGE); 
    gpio_install_isr_service(0);
    gpio_isr_handler_add(btn1, bnt_a_ISR_handler, NULL);
    gpio_isr_handler_add(btn2, bnt_b_ISR_handler, NULL);

}

void inicializar_pwm_buzzer() {

  // Configuração do PWM (buzzer)
  ledc_timer_config_t ledc_timer = {
      .duty_resolution = LEDC_TIMER_8_BIT, // 0-255
      .freq_hz = 5000,                     // 5kHz
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .timer_num = LEDC_TIMER_0
  };
  ledc_timer_config(&ledc_timer);

  ledc_channel_config_t ledc_channel = {
      .channel = LEDC_CHANNEL,
      .duty       = 0,
      .gpio_num   = BUZZER,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .timer_sel  = LEDC_TIMER_0
  };

  ledc_channel_config(&ledc_channel);
}

void ligar_buzzer() {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL, 128);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL);
}

void desligar_buzzer() {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL);
}

void update_lcd_display(int ntc, int tempdef) {
    char buffer[20];
    
    lcd_i2c_cursor_set(&lcd, 0, 0);
    snprintf(buffer, sizeof(buffer), "NTC:%d C", ntc);
    lcd_i2c_print(&lcd, buffer);

    lcd_i2c_cursor_set(&lcd, 0, 1);
    snprintf(buffer, sizeof(buffer), "Temp: %d C", tempdef);
    lcd_i2c_print(&lcd, buffer);
    
}

void sd_card() {
  esp_vfs_fat_sdmmc_mount_config_t mnt_config = {
    .format_if_mount_failed = false,
    .max_files = 5,
    .allocation_unit_size = 16 * 1024
  };

  sdmmc_card_t *sd_card;

  //Inicializa a estrutura com os valores padrão.
  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  //Sobrescreve manualmente o slot para usar o SPI3, evitando o conflito de pinos.
  host.slot = SPI2_HOST;
  host.max_freq_khz = SDMMC_FREQ_PROBING;

  spi_bus_config_t spi_io = {
    .mosi_io_num = MOSI,
    .miso_io_num = MISO,
    .sclk_io_num = SCLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 4000
  };

  esp_err_t init_status = spi_bus_initialize(host.slot, &spi_io, SDSPI_DEFAULT_DMA);

  if (init_status != ESP_OK) {
    printf("Falha ao inicializar o spi");
    return;
  }

  sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
  slot_config.gpio_cs = CS;
  slot_config.host_id = host.slot;

  init_status = esp_vfs_fat_sdspi_mount(SD_MNT_POINT, &host, &slot_config, &mnt_config, &sd_card);

  if (init_status != ESP_OK) {
    printf("Falha ao montar o cartao SD. Erro: %s\n", esp_err_to_name(init_status));
    return;
  }

  printf("Cartao SD inicializado e montado com sucesso!\n");
  sdmmc_card_print_info(stdout, sd_card);
}


void app_main() {
  init_gpio();
  i2c_master_init();

  lcd.address = LCD_ADDRESS;
  lcd.num = I2C_MASTER_NUM;
  lcd.backlight = 1; 
  lcd.size = LCD_SIZE;
  lcd_i2c_init(&lcd);

  inicializar_pwm_buzzer();


  lcd_i2c_cursor_set(&lcd, 0, 0);
  lcd_i2c_print(&lcd, "Temp");
  lcd_i2c_cursor_set(&lcd, 0, 1);
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  lcd_i2c_write(&lcd, 0, CLEAR_DISPLAY);
  
  ntc = (int)ler_temp();

  adc1_config_width(ADC_WIDTH_BIT_12);  // resolução de 12 bits
  adc1_config_channel_atten(SENSOR_ADC, ADC_ATTEN_DB_11);  // atenuação 

  //int temp = adc1_get_raw(ADC1_CHANNEL_17);
  
  last_press_a = esp_timer_get_time(); 
  last_press_b = esp_timer_get_time();

  sd_card();

  bool alarme_ativo;
    
  while (1) {
    switch (estado_atual) {
        case ESTADO_INICIALIZACAO:
            init_gpio();
            i2c_master_init();
            inicializar_pwm_buzzer();
            lcd.address = LCD_ADDRESS;
            lcd.num = I2C_MASTER_NUM;
            lcd.backlight = 1;
            lcd.size = LCD_SIZE;
            lcd_i2c_init(&lcd);
            lcd_i2c_cursor_set(&lcd, 0, 0);
            lcd_i2c_print(&lcd, "Temp");
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            lcd_i2c_write(&lcd, 0, CLEAR_DISPLAY);
            adc1_config_width(ADC_WIDTH_BIT_12);
            adc1_config_channel_atten(SENSOR_ADC, ADC_ATTEN_DB_11);
            sd_card();
            last_press_a = esp_timer_get_time();
            last_press_b = esp_timer_get_time();
            estado_atual = ESTADO_VERIFICAR_TEMPERATURA;
            break;

        case ESTADO_VERIFICAR_TEMPERATURA:
            ntc = (int)ler_temp();
            estado_atual = ESTADO_TRATAR_BOTOES;
            break;

        case ESTADO_TRATAR_BOTOES: {
            int64_t now = esp_timer_get_time();
            if (bnt_a_interrupt && now - last_press_a > DEBOUNCE_TIME_US) {
                tempdef += 5;
                last_press_a = now;
                bnt_a_interrupt = false;
                gpio_intr_enable(btn1);
            }
            if (bnt_b_interrupt && now - last_press_b > DEBOUNCE_TIME_US) {
                tempdef -= 5;
                last_press_b = now;
                bnt_b_interrupt = false;
                gpio_intr_enable(btn2);
            }
            estado_atual = ESTADO_TRATAR_ALARME;
            break;
        }

        case ESTADO_TRATAR_ALARME:
            if (ntc >= tempdef) {
                ligar_buzzer();
                estado_atual = ESTADO_ATUALIZAR_LEDS;
            } else {
                desligar_buzzer();
                estado_atual = ESTADO_ATUALIZAR_LEDS;
            }
            break;

        case ESTADO_ATUALIZAR_LEDS:
            update_leds(ntc, tempdef, ntc >= tempdef);
            estado_atual = ESTADO_ATUALIZAR_LCD;
            break;

        case ESTADO_ATUALIZAR_LCD:
            update_lcd_display(ntc, tempdef);
            vTaskDelay(pdMS_TO_TICKS(10));
            estado_atual = ESTADO_VERIFICAR_TEMPERATURA; // volta ao ciclo
            break;
    }
  }
}
