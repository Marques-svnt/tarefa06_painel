// Bibliotecas padrão para Raspberry
#include "stdio.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

// Bibliotecas do projeto
#include "libs/ssd1306.h"
#include "libs/display.h"
#include "libs/buzzer.h"
#include "libs/pio.h"

// Bibliotecas para FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// Definições globais
#define BOTAO_A 5   // Incrementa
#define BOTAO_B 6   // Decrementa
#define BOTAO_JB 22 // Reset
#define VERDE 11
#define AZUL 12
#define VERMELHO 13

// Semáforos
SemaphoreHandle_t xSemEntrada;
SemaphoreHandle_t xSemSaida;
SemaphoreHandle_t xSemReset;
SemaphoreHandle_t xContadorSem;
SemaphoreHandle_t xOledMutex;

// Variaveis globais
bool display_inicializado = false;
volatile unsigned int MAX = 9;
uint16_t usuarios_atual = 0;
bool usuario_saiu;
unsigned int usuarios_atual_reset;
static unsigned int previous_count_for_display = 0;
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
    else if (gpio == BOTAO_B && debounce(&last_time_B, 300000))
    {
        last_time_B = current_time;
        // Evento do Botão B (Saída)
        if (xSemSaida != NULL) // Verifica se o semáforo foi criado
        {
            xSemaphoreGiveFromISR(xSemSaida, &xHigherPriorityTaskWoken);
        }
    }
    else if (gpio == BOTAO_JB && debounce(&last_time_JB, 300000))
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

void update_count(unsigned int count, unsigned int capacity)
{
    // Nenhum usuário -> Azul
    if (count == 0)
    {
        gpio_put(VERDE, 0);
        gpio_put(AZUL, 1);
        gpio_put(VERMELHO, 0);

        set_one_led(count, 0, 0, 20);
    }
    // Mais que 1 usuário e menos que o máximo - 2 -> Verde
    else if (count <= (capacity - 2))
    {
        gpio_put(VERDE, 1);
        gpio_put(AZUL, 0);
        gpio_put(VERMELHO, 0);
        set_one_led(count, 0, 20, 0);
    }
    // Apenas uma vaga restante -> Amarelo
    else if (count == (capacity - 1))
    {
        gpio_put(VERDE, 1);
        gpio_put(AZUL, 0);
        gpio_put(VERMELHO, 1);
        set_one_led(count, 20, 10, 0);
    }
    // Sem vagas -> Vermelho
    else if (count == capacity)
    {
        gpio_put(VERDE, 0);
        gpio_put(AZUL, 0);
        gpio_put(VERMELHO, 1);
        set_one_led(count, 20, 0, 0);
    }

    // Função para atualização do display protegido por mutex
    // Determina a mudança para a lógica do display ---
    bool count_aumentou = (count > previous_count_for_display);
    bool count_diminuiu = (count < previous_count_for_display);
    if (xOledMutex != NULL && xSemaphoreTake(xOledMutex, portMAX_DELAY) == pdTRUE)
    {
        char vagas[20];

        sprintf(vagas, "0%u/0%u", count, capacity);

        if (count == 0)
        {
            limpar();
            interface_display("Lab Vazio", vagas);
        }
        else if (count == capacity)
        {
            sprintf(vagas, "0%u/0%u ()", count, capacity); // () = sinal de alerta
            limpar();
            interface_display("Lab Cheio", vagas);
        }
        else
        {
            if (count_aumentou)
            {
                limpar();
                interface_display("Pessoa entrou", vagas);
            }
            else if (count_diminuiu)
            {
                limpar();
                interface_display("Pessoa saiu", vagas);
            }
        }
        previous_count_for_display = count;
        xSemaphoreGive(xOledMutex);
    }
}

void init()
{
    stdio_init_all();

    initI2C(); // Inicializa display

    initializePio(); // Inicializa Matriz de leds

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

    buzzer_init(); // Inicializa buzzer

    // Configura os leds
    gpio_init(VERDE);
    gpio_set_dir(VERDE, GPIO_OUT);

    gpio_init(AZUL);
    gpio_set_dir(AZUL, GPIO_OUT);

    gpio_init(VERMELHO);
    gpio_set_dir(VERMELHO, GPIO_OUT);
}

// ----- Tarefas do FreeRTOS -----

void vTaskEntrada(void *pvParameters)
{
    // Executa a atualização inicial do display APENAS UMA VEZ
    if (!display_inicializado)
    {
        // 'usuarios_atual' é global e deve ser 0 no início.
        // 'MAX' também é global.
        update_count(usuarios_atual, MAX);
        display_inicializado = true;
    }
    while (1)
    {
        // Aguarda o sinal do botão A
        if (xSemaphoreTake(xSemEntrada, portMAX_DELAY) == pdTRUE)
        {
            // Tenta pegar uma vaga no semafóro de contagem
            if (xSemaphoreTake(xContadorSem, (TickType_t)0) == pdTRUE)
            {
                // Se há vaga, incrementa e atualiza o led, matriz e display
                taskENTER_CRITICAL();
                usuarios_atual++;
                taskEXIT_CRITICAL();

                // Atualiza o led, display e matriz para indicar o incremento
                update_count(usuarios_atual, MAX);
            }
            else
            {
                // Se não há vaga
                // Beep curto
                buzz(100);
                vTaskDelay(100);
                buzzer_stop();
            }
        }
    }
}

void vTaskSaida(void *pvParameters)
{
    while (1)
    {
        // Aguarda o sinal do botão B
        if (xSemaphoreTake(xSemSaida, portMAX_DELAY) == pdTRUE)
        {
            usuario_saiu = false;

            // Verifica se há usuários para sair e caso haja, decrementa e atualiza a booleana
            taskENTER_CRITICAL();
            if (usuarios_atual > 0)
            {
                usuarios_atual--;
                usuario_saiu = true;
            }
            taskEXIT_CRITICAL();

            // Se houve decremento
            if (usuario_saiu)
            {
                // Devolve uma vaga ao semáforo de contagem
                xSemaphoreGive(xContadorSem);

                // Atualiza o LED RGB, a matriz e o display para indicar a nova ocupação
                update_count(usuarios_atual, MAX);
            }
            else
            {
                // Se não houve decremento
                // Informar no painel que não há nínguém
                update_count(usuarios_atual, MAX);
            }
        }
    }
}

void vTaskReset(void *pvParameters)
{

    while (1)
    {
        // Aguarda o sinal de que o Botão do Joystick foi pressionado
        if (xSemaphoreTake(xSemReset, portMAX_DELAY) == pdTRUE)
        {

            // 2. Zera a contagem de usuários e prepara para resetar o semáforo de contagem
            // Protegendo o acesso a 'usuarios_atual'
            taskENTER_CRITICAL();
            usuarios_atual_reset = usuarios_atual; // Salva quantos "takes" foram feitos no xContadorSem
            usuarios_atual = 0;                    // Zera a contagem global de usuários
            taskEXIT_CRITICAL();

            // Restaura o semáforo de contagem 'xContadorSem e devolve ao semáforo as "vagas" que estavam ocupadas
            for (unsigned int i = 0; i < usuarios_atual_reset; i++)
            {
                xSemaphoreGive(xContadorSem);
            }
            // Após este loop, xContadorSem deve refletir que todas as MAX vagas estão disponíveis.

            // Beep duplo
            buzz(100);
            vTaskDelay(pdMS_TO_TICKS(100));
            buzzer_stop();
            vTaskDelay(pdMS_TO_TICKS(50));
            buzz(100);
            vTaskDelay(pdMS_TO_TICKS(100));
            buzzer_stop();

            // Atualiza a matriz, led e display para o estado de reset
            update_count(usuarios_atual, MAX); // Com usuarios_atual = 0, deve ficar Azul
        }
    }
}

int main()
{
    init();

    gpio_set_irq_enabled_with_callback(BOTAO_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(BOTAO_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(BOTAO_JB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    // Semáforo de contagem (máximo 10, inicial 10)
    xContadorSem = xSemaphoreCreateCounting(MAX, MAX);

    // Semáforos binários
    xSemEntrada = xSemaphoreCreateBinary();
    xSemSaida = xSemaphoreCreateBinary();
    xSemReset = xSemaphoreCreateBinary();

    // Mutex para display
    xOledMutex = xSemaphoreCreateMutex();

    // Tarefas
    xTaskCreate(vTaskEntrada, "TaskEntrada", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(vTaskSaida, "TaskSaida", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(vTaskReset, "TaskReset", configMINIMAL_STACK_SIZE, NULL, 2, NULL);

    vTaskStartScheduler();
    panic_unsupported();
}              