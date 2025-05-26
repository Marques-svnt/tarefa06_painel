#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#define BUZZER_PIN 21

// Inicializa o PWM para o buzzer
int buzzer_init() {
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 4.0f); // Clock divider = 4.0
    pwm_init(slice_num, &config, true);
    pwm_set_gpio_level(BUZZER_PIN, 0); // Desliga inicialmente
    return slice_num;
}

// Toca um tom no buzzer em Hz
void buzz(uint freq) {
    if (freq == 0) return; // Evita divisão por zero

    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);
    uint channel = pwm_gpio_to_channel(BUZZER_PIN);

    float clkdiv = 4.0f; // Mesmo valor usado em buzzer_init
    uint32_t clock_freq = clock_get_hz(clk_sys);
    uint32_t pwm_freq = clock_freq / clkdiv;

    uint32_t top = pwm_freq / freq - 1;
    if (top > 65535) top = 65535; // Limite do contador de 16 bits

    pwm_set_wrap(slice_num, top);
    pwm_set_chan_level(slice_num, channel, top / 2); // 50% duty cycle
}

// Desliga o buzzer
void buzzer_stop() {
    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);
    uint channel = pwm_gpio_to_channel(BUZZER_PIN);
    pwm_set_chan_level(slice_num, channel, 0); // Sem pulso PWM
}
