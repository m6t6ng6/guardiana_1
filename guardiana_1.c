#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdbool.h>
#include "lcd.h"

// Eleccion de I2C a usar
#define I2C         i2c0
// Eleccion de GPIO para SDA
#define SDA_GPIO    8
// Eleccion de GPIO para SCL
#define SCL_GPIO    9
// Direccion de 7 bits del adaptador del LCD
#define ADDR        0x27

#define ADC_CH1 0  // GPIO26
#define ADC_CH2 1  // GPIO27
#define TEMP_CH 4 // Sensor interno
#define GPIO_LED_ALERTA 10
#define UMBRAL_ADC 4000
#define ALERTA_TIMEOUT_MS 5000

QueueHandle_t queue_ch1;
QueueHandle_t queue_ch2;
QueueHandle_t queue_temp;

void task_adc_ch1(void *params);
void task_adc_ch2(void *params);
void task_adc_temp(void *params);
void task_guardian_lcd(void *params);

void adc_setup() {
    adc_init();
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_set_temp_sensor_enabled(true);
}

void gpio_setup() {
    gpio_init(GPIO_LED_ALERTA);
    gpio_set_dir(GPIO_LED_ALERTA, GPIO_OUT);
    gpio_put(GPIO_LED_ALERTA, 0);
}

int main() {
    stdio_init_all();
    adc_setup();
    gpio_setup();

    // Inicializo el I2C con un clock de 100 KHz
    i2c_init(I2C, 100000);
    // Habilito la funcion de I2C en los GPIOs
    gpio_set_function(SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(SCL_GPIO, GPIO_FUNC_I2C);
    // Habilito pull-ups
    gpio_pull_up(SDA_GPIO);
    gpio_pull_up(SCL_GPIO);
    // Inicializo LCD
    lcd_init(I2C, ADDR);

    queue_ch1 = xQueueCreate(10, sizeof(uint16_t));
    queue_ch2 = xQueueCreate(10, sizeof(uint16_t));
    queue_temp = xQueueCreate(10, sizeof(uint16_t));

    xTaskCreate(task_adc_ch1, "ADC_CH1", 256, NULL, 1, NULL);
    xTaskCreate(task_adc_ch2, "ADC_CH2", 256, NULL, 1, NULL);
    xTaskCreate(task_adc_temp, "ADC_TEMP", 256, NULL, 1, NULL);
    xTaskCreate(task_guardian_lcd, "Guardian_LCD", 512, NULL, 2, NULL);

    vTaskStartScheduler();
    while (1);
}

void task_adc_ch1(void *params) {
    while (1) {
        adc_select_input(ADC_CH1);
        uint16_t value = adc_read();
        xQueueSend(queue_ch1, &value, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void task_adc_ch2(void *params) {
    while (1) {
        adc_select_input(ADC_CH2);
        uint16_t value = adc_read();
        xQueueSend(queue_ch2, &value, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void task_adc_temp(void *params) {
    while (1) {
        adc_select_input(TEMP_CH);
        uint16_t raw = adc_read();
        xQueueSend(queue_temp, &raw, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void task_guardian_lcd(void *params) {
    uint16_t val1 = 0, val2 = 0, raw_temp = 0;
    bool alerta_latched = false;
    TickType_t tick_ultima_alerta = 0;

    while (1) {
        xQueueReceive(queue_ch1, &val1, pdMS_TO_TICKS(100));
        xQueueReceive(queue_ch2, &val2, pdMS_TO_TICKS(100));
        xQueueReceive(queue_temp, &raw_temp, pdMS_TO_TICKS(100));

        const float conv_factor = 3.3f / (1 << 12);
        float voltage = raw_temp * conv_factor;
        float temp_c = 27 - (voltage - 0.706f) / 0.001721f;

        // Mostrar datos
        printf("ADC1: %.2f \t| ADC2: %.2f \t| Temp: %.2f°C\n", val1 * conv_factor, val2 * conv_factor, temp_c);

        // Limpio el LCD
        lcd_clear();
        // Escribo al comienzo
        lcd_string("<< DASHBOARD >>");
        // Muevo el cursor a la segunda fila, tercer columna
        lcd_set_cursor(1, 2);
        // Escribo
        char buffer[20];
        sprintf(buffer, "ADC_1: %.2f V", val1 * conv_factor);
        lcd_string(buffer);
        // Muevo el cursor a la segunda fila, tercer columna
        lcd_set_cursor(2, 2);
        // Escribo
        sprintf(buffer, "ADC_2: %.2f V", val2 * conv_factor);
        lcd_string(buffer);
        // Muevo el cursor a la segunda fila, tercer columna
        lcd_set_cursor(3, 2);
        // Escribo
        sprintf(buffer, "Temp: %.2f C", temp_c);
        lcd_string(buffer);
        

        // Si superó el umbral → activa latch y guarda tiempo
        if (val1 > UMBRAL_ADC || val2 > UMBRAL_ADC) {
            alerta_latched = true;
            tick_ultima_alerta = xTaskGetTickCount();
        }

        // Si está activo y ya pasó el timeout → apagar
        if (alerta_latched) {
            TickType_t ahora = xTaskGetTickCount();
            if ((ahora - tick_ultima_alerta) > pdMS_TO_TICKS(ALERTA_TIMEOUT_MS)) {
                alerta_latched = false;
            }
        }

        // Control de LED
        gpio_put(GPIO_LED_ALERTA, alerta_latched);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}