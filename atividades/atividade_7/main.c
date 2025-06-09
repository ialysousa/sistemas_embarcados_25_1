// ialy cordeiro de sousa

/*
   Projeto Atividade 07: Leitura do sensor de temperatura analógico (NTC)
   · ESP32-S3 (ADC, GPIO, PWM, I²C, Interrupções)
   · Wokwi Simulation
   → Sensores: NTC (10 kΩ) em divisor de tensão
   → Botões A (GPIO32) e B (GPIO33) com debounce
   → Buzzer (GPIO25) via PWM (LEDC)
   → LCD 16×2 I²C (GPIO21: SDA, GPIO22: SCL)
   → LEDs (GPIO14, GPIO27, GPIO26, GPIO13)
   → Alarm threshold default: 25 °C (incremento/decremento de ± 5 °C)
*/

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_timer.h"

/* ------------------------------------------------------------
   DEFINIÇÕES DE HARDWARE (pinos, parâmetros, constantes)
   ------------------------------------------------------------ */

// ADC (NTC)
#define ADC_CHANNEL         ADC_CHANNEL_0     // ADC1_CH0 → GPIO36
#define ADC_UNIT            ADC_UNIT_1
#define ADC_ATTEN           ADC_ATTEN_DB_11   // Atenuação 11 dB → range ~0-3.3 V
#define ADC_BIT_WIDTH       ADC_WIDTH_BIT_12  // Resolução 12 bits (0-4095)
#define DEFAULT_VREF        1100              // (mV) – Vref estimado (fazer calibração se precisar precisão >1%)
static esp_adc_cal_characteristics_t adc_chars;

// Parâmetros do NTC (exemplo: 10 kΩ a 25 °C, Beta=3950 K)
#define R_SERIES            10000.0f   // 10 kΩ (resistor fixo em série)
#define BETA_VALUE          3950.0f    // Beta do termistor
#define T0_KELVIN           298.15f    // 25 °C em Kelvin (25 + 273.15)
#define R0                  10000.0f   // Resistência do NTC a 25 °C (10 kΩ)

// Botões (com pull-up e interrupção)
#define BOTAO_A_GPIO        GPIO_NUM_32  // Incrementa alarme (+5 °C)
#define BOTAO_B_GPIO        GPIO_NUM_33  // Decrementa alarme (−5 °C)
#define DEBOUNCE_DELAY_MS   50           // Debounce por software em ms

// Buzzer (PWM via LEDC)
#define BUZZER_GPIO         GPIO_NUM_25
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL        LEDC_CHANNEL_0
#define LEDC_DUTY_RES       LEDC_TIMER_10_BIT // resolução de 10 bits (0-1023)
#define LEDC_FREQUENCY      2000               // 2 kHz para o buzzer

// LCD I²C (utilizamos endereço 0x27 por padrão, mas pode variar)
#define I2C_MASTER_SCL_IO   GPIO_NUM_22   // SCL do LCD
#define I2C_MASTER_SDA_IO   GPIO_NUM_21   // SDA do LCD
#define I2C_MASTER_FREQ_HZ  100000        // 100 kHz
#define I2C_MASTER_PORT     I2C_NUM_0

#define LCD_I2C_ADDR        0x27          // Endereço I²C do PCF8574 (ver etiqueta no módulo)
#define LCD_COLS            16
#define LCD_ROWS            2

// LEDs de proximidade (4 níveis)
#define LED1_GPIO           GPIO_NUM_14
#define LED2_GPIO           GPIO_NUM_27
#define LED3_GPIO           GPIO_NUM_26
#define LED4_GPIO           GPIO_NUM_13

// Parâmetros gerais
#define TEMP_ALARME_DEFAULT 25    // 25 °C como valor padrão de alarme
#define TEMP_STEP           5     // Incremento/decremento de 5 °C por botão
#define REFRESH_PERIOD_MS   500   // 0.5 s de atualização do loop principal

/* ------------------------------------------------------------
   VARIÁVEIS GLOBAIS (compartilhadas por ISR e tarefas)
   ------------------------------------------------------------ */
static volatile int temp_alarme = TEMP_ALARME_DEFAULT;  // valor atual do alarme (°C)
static volatile bool botaoA_pressionado = false;
static volatile bool botaoB_pressionado = false;

/* ------------------------------------------------------------
   FUNÇÕES AUXILIARES
   ------------------------------------------------------------ */

static const char *TAG = "ATV07_TEMP";

// ISR de debounce simplificado (para cada botão)
// Marca uma flag, e a tarefa principal fará debounce por software
static void IRAM_ATTR botaoA_isr_handler(void *arg) {
    // Desliga interrupção para evitar rebotes
    gpio_intr_disable(BOTAO_A_GPIO);
    botaoA_pressionado = true;
}

static void IRAM_ATTR botaoB_isr_handler(void *arg) {
    gpio_intr_disable(BOTAO_B_GPIO);
    botaoB_pressionado = true;
}

// Tarefa para reabilitar interrupção após debounce
static void debounce_task(void *arg) {
    int *pino = (int *)arg; // pino do botão
    vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY_MS));
    gpio_intr_enable((gpio_num_t)(*pino));
    vTaskDelete(NULL);
}

/**
 * @brief Configura GPIOs dos botões com pull-up e interrupção
 */
static void botao_configurar(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,       // disparo em borda de descida (botão “ACTIVE LOW”)
        .mode = GPIO_MODE_INPUT,              // entrada
        .pin_bit_mask = (1ULL << BOTAO_A_GPIO) | (1ULL << BOTAO_B_GPIO),
        .pull_up_en = GPIO_PULLUP_ENABLE,     // habilita pull-up interno
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };
    gpio_config(&io_conf);

    // Instala serviço de GPIO ISR
    gpio_install_isr_service(0);

    // Conecta as ISRs
    gpio_isr_handler_add(BOTAO_A_GPIO, botaoA_isr_handler, (void *)&(int){BOTAO_A_GPIO});
    gpio_isr_handler_add(BOTAO_B_GPIO, botaoB_isr_handler, (void *)&(int){BOTAO_B_GPIO});
}

/**
 * @brief Converte leitura de ADC (0-4095) para voltagem em mV
 */
static uint32_t adc_to_voltage(uint32_t adc_reading) {
    // calibrações típicas de ADC: usa esp_adc_cal_characterize + esp_adc_cal_raw_to_voltage
    uint32_t voltage = 0;
    esp_adc_cal_raw_to_voltage(adc_reading, &adc_chars, &voltage);
    return voltage;  // valor em mV
}

/**
 * @brief Calcula a temperatura (°C) a partir do valor da voltagem no divisor NTC+R
 *        - Usa fórmula de termistor (Steinhart-Hart simplificado, usando Beta)
 * @param   voltage_mV: tensão do divisor (mV)
 * @return  temperatura em °C (inteiro, aproximado)
 */
static int adc_to_temperatura(uint32_t voltage_mV) {
    // 1) Converte mV para V
    float v = (float)voltage_mV / 1000.0f;  // Volts
    // 2) Calcula resistência do NTC: R_ntc = R_SERIES * (V / (3.3 - V))
    float r_ntc = R_SERIES * (v / (3.3f - v));
    // 3) Aplica fórmula de Beta: 1/T = 1/T0 + (1/B) * ln(R_ntc / R0)
    float ln_ratio = logf(r_ntc / R0);
    float inv_T = (1.0f / T0_KELVIN) + (ln_ratio / BETA_VALUE);
    float T_kelvin = 1.0f / inv_T;
    float T_celsius = T_kelvin - 273.15f;

    return (int)(T_celsius + 0.5f);  // arredonda para inteiro
}

/* ------------------------------------------------------------
   SETUP DO ADC (calibração e configuração)
   ------------------------------------------------------------ */
static void adc_configurar(void) {
    // 1) Configurar largura do ADC
    adc1_config_width(ADC_BIT_WIDTH);
    // 2) Configurar atenuação do canal (0–3.3 V)
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);
    // 3) Caracterizar/calibrar a ADC
    esp_adc_cal_characterize(
        ADC_UNIT,
        ADC_ATTEN,
        ADC_BIT_WIDTH,
        DEFAULT_VREF,
        &adc_chars
    );
}

/* ------------------------------------------------------------
   SETUP DO PWM PARA O BUZZER (LEDC)
   ------------------------------------------------------------ */
static void buzzer_pwm_configurar(void) {
    // Configura timer do LEDC (pwm)
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_DUTY_RES,   // resolução 10 bits
        .freq_hz         = LEDC_FREQUENCY,  // frequência 2 kHz
        .speed_mode      = LEDC_MODE,
        .timer_num       = LEDC_TIMER,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // Configura canal do LEDC (GPIO do buzzer)
    ledc_channel_config_t ledc_channel = {
        .channel    = LEDC_CHANNEL,
        .duty       = 0,
        .gpio_num   = BUZZER_GPIO,
        .speed_mode = LEDC_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER
    };
    ledc_channel_config(&ledc_channel);
}

/**
 * @brief Liga o buzzer (seta duty cycle conforme nível desejado)
 */
static void buzzer_ligar(void) {
    // Ex.: duty cycle 50% = (2^10 − 1) / 2 = 512
    uint32_t duty = (1 << (LEDC_DUTY_RES - 1));
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

/**
 * @brief Desliga o buzzer (duty = 0)
 */
static void buzzer_desligar(void) {
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

/* ------------------------------------------------------------
   SETUP DOS LEDs DE PROXIMIDADE (GPIO OUTPUT)
   ------------------------------------------------------------ */
static void leds_configurar(void) {
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED1_GPIO) |
                        (1ULL << LED2_GPIO) |
                        (1ULL << LED3_GPIO) |
                        (1ULL << LED4_GPIO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Inicializa todos os LEDs desligados
    gpio_set_level(LED1_GPIO, 0);
    gpio_set_level(LED2_GPIO, 0);
    gpio_set_level(LED3_GPIO, 0);
    gpio_set_level(LED4_GPIO, 0);
}

/**
 * @brief Atualiza o estado dos LEDs segundo proximidade à temperatura de alarme
 * @param temp_medida: temperatura atual medida (°C)
 * @param temp_alarme: limiar de alarme (°C)
 */
static void leds_atualizar(int temp_medida, int temp_alarme) {
    int delta = temp_alarme - temp_medida;

    // Desliga todos antes de definir novos estados
    gpio_set_level(LED1_GPIO, 0);
    gpio_set_level(LED2_GPIO, 0);
    gpio_set_level(LED3_GPIO, 0);
    gpio_set_level(LED4_GPIO, 0);

    if (temp_medida >= temp_alarme) {
        // Temperatura igual ou acima do limite: piscará externamente (tratado em outra parte do loop)
        return;
    }
    // Proximidades definidas:
    // Se delta ≤ 20 °C → liga LED1
    // Se delta ≤ 15 °C → liga LED2 (e LED1 deve permanecer, ou apenas LED2? Na tarefa, é cumulativo: liga 2 LEDs se ≤ 15 °C, etc.)
    // Aqui, interpretamos como:  
    //   delta ≤ 20 → LED1  
    //   delta ≤ 15 → LED1+LED2  
    //   delta ≤ 10 → LED1+LED2+LED3  
    //   delta ≤ 2  → LED1+LED2+LED3+LED4

    if (delta <= 20) {
        gpio_set_level(LED1_GPIO, 1);
    }
    if (delta <= 15) {
        gpio_set_level(LED2_GPIO, 1);
    }
    if (delta <= 10) {
        gpio_set_level(LED3_GPIO, 1);
    }
    if (delta <= 2) {
        gpio_set_level(LED4_GPIO, 1);
    }
}

/* ------------------------------------------------------------
   CONFIGURAÇÃO DO I2C E DA BIBLIOTECA DO LCD
   ------------------------------------------------------------ */
/*
   Para o LCD baseado em HD44780 via PCF8574, você pode usar uma biblioteca
   simples que implemente as funções de inicialização, enviar comandos e
   imprimir strings. No exemplo a seguir, supomos que exista uma camada
   "lcd_i2c.h" com as funções:
     lcd_init(port, addr, cols, rows);
     lcd_clear();
     lcd_set_cursor(col, row);
     lcd_print(str);
   Se você não tiver essa camada, pode adaptar qualquer biblioteca pública
   disponível (por exemplo: “esp-idf-lcd-esp32” ou “hd44780_i2c”).
*/

#include "lcd_i2c.h"  // inclui funções de controle do LCD I²C

/**
 * @brief Inicializa barramento I²C e LCD
 */
static void i2c_lcd_configurar(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    i2c_param_config(I2C_MASTER_PORT, &conf);
    i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);

    // Inicializa o LCD (supor biblioteca compatível)
    lcd_init(I2C_MASTER_PORT, LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);
    lcd_clear();
}

/**
 * @brief Atualiza o LCD com:  
 *   - Na **linha 1**: “Temp: XX °C” (temperatura medida)  
 *   - Na **linha 2**: “Alarm: YY °C” (temperatura de alarme)  
 */
static void lcd_atualizar(int temp_medida, int temp_alarme) {
    char linha1[17], linha2[17];
    snprintf(linha1, sizeof(linha1), "Temp: %3d *C", temp_medida);
    snprintf(linha2, sizeof(linha2), "Alarm: %3d *C", temp_alarme);

    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print(linha1);
    lcd_set_cursor(0, 1);
    lcd_print(linha2);
}

/* ------------------------------------------------------------
   TAREFA PRINCIPAL: LEITURA, CONTROLE, ATUALIZAÇÃO
   ------------------------------------------------------------ */
void app_main(void) {
    ESP_LOGI(TAG, "=== Iniciando Atividade 07: Sensor de Temperatura Analógico ===");

    // 1) Configurar ADC
    adc_configurar();

    // 2) Configurar botões com interrupção + debounce
    botao_configurar();

    // 3) Configurar buzzer (PWM)
    buzzer_pwm_configurar();
    buzzer_desligar();  // garante estar desligado no início

    // 4) Configurar LEDs de proximidade
    leds_configurar();

    // 5) Configurar I2C e LCD
    i2c_lcd_configurar();
    lcd_atualizar(0, temp_alarme);  // mostra “Temp:  0 °C” e “Alarm: 25 °C” inicialmente

    // Variáveis locais
    uint32_t adc_reading = 0;
    int temp_medida = 0;
    TickType_t ultimo_tick = xTaskGetTickCount();

    while (1) {
        // a) Lê ADC em média de várias amostras para reduzir ruído
        const int num_amostras = 10;
        uint32_t adc_sum = 0;
        for (int i = 0; i < num_amostras; i++) {
            adc_sum += adc1_get_raw((adc1_channel_t)ADC_CHANNEL);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        adc_reading = adc_sum / num_amostras;

        // b) Converte ADC → voltagem (mV) → temperatura (°C)
        uint32_t voltage_mV = adc_to_voltage(adc_reading);
        temp_medida = adc_to_temperatura(voltage_mV);

        // c) Verifica flags de botões (A e B) e trata debounce
        if (botaoA_pressionado) {
            temp_alarme += TEMP_STEP;
            if (temp_alarme > 100) {  // limite superior (ex.: 100 °C)
                temp_alarme = 100;
            }
            ESP_LOGI(TAG, "Botão A pressionado: novo alarme = %d °C", temp_alarme);
            botaoA_pressionado = false;
            // Inicia tarefa de debounce para reabilitar interrupção
            xTaskCreate(debounce_task, "debounceA", 2048, (void *)&(int){BOTAO_A_GPIO}, 10, NULL);
        }
        if (botaoB_pressionado) {
            temp_alarme -= TEMP_STEP;
            if (temp_alarme < 0) {  // limite inferior (ex.: 0 °C)
                temp_alarme = 0;
            }
            ESP_LOGI(TAG, "Botão B pressionado: novo alarme = %d °C", temp_alarme);
            botaoB_pressionado = false;
            xTaskCreate(debounce_task, "debounceB", 2048, (void *)&(int){BOTAO_B_GPIO}, 10, NULL);
        }

        // d) Gera alarme sonoro e LEDs piscando se temp_medida ≥ temp_alarme
        if (temp_medida >= temp_alarme) {
            // Aciona buzzer
            buzzer_ligar();
            // Pisca todos os 4 LEDs a uma frequência de ~1 Hz
            gpio_set_level(LED1_GPIO, 1);
            gpio_set_level(LED2_GPIO, 1);
            gpio_set_level(LED3_GPIO, 1);
            gpio_set_level(LED4_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(250));
            gpio_set_level(LED1_GPIO, 0);
            gpio_set_level(LED2_GPIO, 0);
            gpio_set_level(LED3_GPIO, 0);
            gpio_set_level(LED4_GPIO, 0);
            vTaskDelay(pdMS_TO_TICKS(250));
        } else {
            // Desativa buzzer
            buzzer_desligar();
            // Atualiza LEDs segundo proximidade
            leds_atualizar(temp_medida, temp_alarme);
        }

        // e) Atualiza LCD se já passou um ciclo de refresh
        if ((xTaskGetTickCount() - ultimo_tick) >= pdMS_TO_TICKS(REFRESH_PERIOD_MS)) {
            lcd_atualizar(temp_medida, temp_alarme);
            ultimo_tick = xTaskGetTickCount();
        }

        // f) Pequena pausa antes da próxima leitura
        //    (já contemplado nos delays internos, mas assegura ciclo de ~500 ms)
        vTaskDelay(pdMS_TO_TICKS(REFRESH_PERIOD_MS));
    }
}
