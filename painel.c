// Bibliotecas padrão para Raspberry
#include "stdio.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

// Bibliotecas do projeto
#include "libs/ssd1306.h"

// Bibliotecas para FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// Definições globais
#define BOTAO_A 5   // Incrementa
#define BOTAO_B 6   // Decrementa
#define BOTAO_JB 22 // Reset

// Semáforos
SemaphoreHandle_t xSemEntrada;
SemaphoreHandle_t xSemSaida;
SemaphoreHandle_t xSemReset;
SemaphoreHandle_t xContadorSem;

// Variaveis globais
unsigned int MAX = 10;
uint16_t eventosProcessados = 0;
static volatile uint32_t last_time_A = 0; // Armazena o tempo do último evento (em microssegundos)
static volatile uint32_t last_time_B = 0;
static volatile uint32_t last_time_JB = 0;

// Função responsável pelo debounce
bool debounce(volatile uint32_t *last_time, uint32_t debounce_time)
{
    uint32_t current_time = to_us_since_boot(get_absolute_time());
    if (current_time - *last_time > debounce_time)
    {
        *last_time = current_time;
        return true;
    }
    return false;
}

// ISR para BOOTSEL e botão de evento
void gpio_irq_handler(uint gpio, uint32_t events)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t current_time = to_us_since_boot(get_absolute_time());

    if (gpio == BOTAO_A && debounce(&last_time_A, 300000))
    {
        last_time_A = current_time;
        // Evento do Botão A (Entrada)
        if (xSemEntrada != NULL) // Verifica se o semáforo foi criado
        {
            xSemaphoreGiveFromISR(xSemEntrada, &xHigherPriorityTaskWoken);
        }
    }
    else if (gpio == BOTAO_B && debounce(&last_time_A, 300000))
    {
        last_time_B = current_time;
        // Evento do Botão B (Saída)
        if (xSemSaida != NULL) // Verifica se o semáforo foi criado
        {
            xSemaphoreGiveFromISR(xSemSaida, &xHigherPriorityTaskWoken);
        }
    }
    else if (gpio == BOTAO_JB && debounce(&last_time_A, 300000))
    {
        last_time_JB = current_time;
        // Evento do Botão do Joystick (Reset)
        if (xSemReset != NULL) // Verifica se o semáforo foi criado
        {
            xSemaphoreGiveFromISR(xSemReset, &xHigherPriorityTaskWoken);
        }
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

int main()
{
    stdio_init_all();

    // Configura os botões
    gpio_init(BOTAO_A);
    gpio_set_dir(BOTAO_A, GPIO_IN);
    gpio_pull_up(BOTAO_A);

    gpio_init(BOTAO_B);
    gpio_set_dir(BOTAO_B, GPIO_IN);
    gpio_pull_up(BOTAO_B);

    gpio_init(BOTAO_JB);
    gpio_set_dir(BOTAO_JB, GPIO_IN);
    gpio_pull_up(BOTAO_JB);

    gpio_set_irq_enabled_with_callback(BOTAO_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(BOTAO_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(BOTAO_JB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    // Cria semáforo de contagem (máximo 10, inicial 0)
    xContadorSem = xSemaphoreCreateCounting(MAX, 0);

    // Cria tarefa

    vTaskStartScheduler();
    panic_unsupported();
}