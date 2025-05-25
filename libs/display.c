// Bibliotecas padrão em C
#include <stdlib.h>

// Bibliotecas de hardware do Raspberry Pi Pico
#include "hardware/i2c.h"
#include "pico/stdlib.h"

// Headers do projeto
#include "ssd1306.h"

#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C


bool cor = true;
ssd1306_t ssd;

void initI2C()
{
    // Inicializa o I2C e configura como 400kHz
    i2c_init(I2C_PORT, 400 * 1000);

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);                    // Set the GPIO pin function to I2C
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);                    // Set the GPIO pin function to I2C
    gpio_pull_up(I2C_SDA);                                        // Pull up the data line
    gpio_pull_up(I2C_SCL);                                        // Pull up the clock line
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT); // Inicializa o display
    ssd1306_config(&ssd);                                         // Configura o display
    ssd1306_send_data(&ssd);                                      // Envia os dados para o display

    // Limpa o display. O display inicia com todos os pixels apagados.
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);
}

// Função que recebe o texto e coordenadas para exibir no display a mensagem
void draw_text(const char *texto, int x, int y)
{
    ssd1306_draw_string(&ssd, texto, x, y);
}

void draw_rect(void)
{
    ssd1306_rect(&ssd, 3, 3, 122, 58, cor, !cor);
    ssd1306_rect(&ssd, 3, 3, 122, 40, cor, !cor);
    ssd1306_rect(&ssd, 3, 3, 122, 39, cor, !cor);
}

// envia o buffer completo ao display
void flush_display(void)
{
    ssd1306_send_data(&ssd);
}

// Função que limpa o display
void limpar()
{
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);
}
