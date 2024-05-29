#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

#include "ssd1306.h"
#include "gfx.h"

const uint BTN_1_OLED = 28;
const uint BTN_2_OLED = 26;
#define ADC_PIN 27  

QueueHandle_t xQueueBtn;
QueueHandle_t xQueuePot;

volatile bool timer_fired = false;

typedef struct {
    int id;
    int status;
} btn_t;

typedef struct {
    float valor_lido;
    float valor_anterior;
} pot_t;

bool pot_updated = false;
pot_t last_pot_value;

int64_t alarm_callback(alarm_id_t id, void *user_data) {
    timer_fired = true;
    return 0;  // Não reagendar
}

void potenciometro_task() {
    pot_t pot_data;
    pot_data.valor_anterior = -1.0f;

    adc_init();
    adc_gpio_init(ADC_PIN);
    const float conversion_factor = 3.3f / (1 << 12);
    uint16_t result;

    while (1) {
        adc_select_input(1);
        result = adc_read();
        pot_data.valor_lido = result * conversion_factor;
        if (fabs(pot_data.valor_lido - pot_data.valor_anterior) > 0.01) {
            pot_data.valor_anterior = pot_data.valor_lido;
            last_pot_value = pot_data;
            pot_updated = true;
            xQueueSend(xQueuePot, &pot_data, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void btn_callback(uint gpio, uint32_t events) {
    static uint32_t last_press_time = 0;
    uint32_t now = to_us_since_boot(get_absolute_time());
    
    if ((events == GPIO_IRQ_EDGE_FALL) && (now - last_press_time > 200000)) { // Debounce
        last_press_time = now;
        btn_t btn_data = {gpio, events};
        xQueueSendFromISR(xQueueBtn, &btn_data, NULL);
        add_alarm_in_ms(300, alarm_callback, NULL, false);  // Agendar alarme para 300ms
    }
}

void oled1_btn_led_init(void) {
    gpio_init(BTN_1_OLED);
    gpio_set_dir(BTN_1_OLED, GPIO_IN);
    gpio_pull_up(BTN_1_OLED);
    gpio_set_irq_enabled_with_callback(BTN_1_OLED, GPIO_IRQ_EDGE_FALL, true, &btn_callback);

    gpio_init(BTN_2_OLED);
    gpio_set_dir(BTN_2_OLED, GPIO_IN);
    gpio_pull_up(BTN_2_OLED);
    gpio_set_irq_enabled_with_callback(BTN_2_OLED, GPIO_IRQ_EDGE_FALL, true, &btn_callback);
}

void oled_task(void *pvParameters) {
    ssd1306_t disp;
    ssd1306_init();
    gfx_init(&disp, 128, 32);

    char buffer[128];

    while (1) {
        if (xQueueReceive(xQueueBtn, NULL, pdMS_TO_TICKS(10)) && timer_fired) {
            if (pot_updated) {
                gfx_clear_buffer(&disp);
                snprintf(buffer, sizeof(buffer), "Valor pot: %.2f V", last_pot_value.valor_lido);
                gfx_draw_string(&disp, 0, 0, 1, buffer);
                gfx_show(&disp);
                timer_fired = false;  // Resetar flag de alarme
                pot_updated = false;  // Resetar flag de atualização do potenciômetro
            }
        }
    }
}

int main() {
    stdio_init_all();
    adc_init();
    adc_gpio_init(ADC_PIN);

    oled1_btn_led_init();

    xQueueBtn = xQueueCreate(10, sizeof(btn_t));
    xQueuePot = xQueueCreate(10, sizeof(pot_t));

    xTaskCreate(oled_task, "OLED Task", 256, NULL, 1, NULL);
    xTaskCreate(potenciometro_task, "Potentiometer Task", 256, NULL, 1, NULL);

    vTaskStartScheduler();
    while (true);
}
