# Painel de Controle Interativo com Acesso Concorrente (FreeRTOS)

Este projeto implementa um **painel de controle interativo** para simular o gerenciamento de acesso concorrente de usuários a um espaço físico (como um laboratório, biblioteca ou refeitório). Utiliza o microcontrolador **RP2040** (na placa BitDogLab), o sistema operacional de tempo real **FreeRTOS**, e periféricos embarcados para fornecer feedback visual, sonoro e textual.

## Periféricos Utilizados

* **Botões de Usuário (A e B)** – Simulam a entrada e saída de usuários.
* **Botão de Joystick** – Utilizado para a função de reset do sistema via interrupção
* **LED RGB** – Sinalização visual da ocupação atual do espaço. 
* **Buzzer** – Emite alertas sonoros para indicar sistema cheio ou reset.
* **Display OLED SSD1306 (I2C)** – Exibe a contagem de usuários, vagas e mensagens de status.
* **Matriz de LEDs WS2812 (via PIO)** – Pode ser usada para ícones ou feedback adicional.

## Funcionalidades

* Estrutura **multitarefa com FreeRTOS**, utilizando pelo menos 3 tarefas para entrada, saída e reset.
* Controle do número de usuários simultâneos utilizando **semáforo de contagem**. 
* Sistema de reset acionado por botão com **semáforo binário e interrupção**.
* Proteção de acesso concorrente ao display OLED utilizando **mutex**. 
* Feedback visual instantâneo sobre a lotação através de LED RGB com cores distintas. 
* Alertas sonoros para indicar tentativa de entrada em local lotado e reset do sistema. 
* Exibição clara de mensagens de status e contagem de usuários no display OLED. 

## Estados do Sistema (Indicados pelo LED RGB)

1.  **Nenhum Usuário Logado**
    * LED RGB: **Azul**
    * Display: Indica "Lab Vazio" ou similar, 0 usuários ativos.
2.  **Usuários Ativos (Lotação Baixa/Média)**
    * Condição: `0 < count <= MAX - 2` usuários
    * LED RGB: **Verde**
    * Display: Mostra contagem atual, mensagem de entrada/saída OK.
3.  **Apenas 1 Vaga Restante**
    * Condição: `count == MAX - 1` usuários
    * LED RGB: **Amarelo**
    * Display: Mostra contagem atual, alerta de poucas vagas.
4.  **Capacidade Máxima Atingida**
    * Condição: `count == MAX` usuários
    * LED RGB: **Vermelho**
    * Display: Indica "Lab Cheio" ou "LOTADO!".
    * Buzzer: Emite beep curto ao tentar nova entrada. 

## Destaque Técnico

* Sistema construído sobre **FreeRTOS**, com uma arquitetura modular de tarefas dedicadas:
    * `vTaskEntrada`: Gerencia a lógica de entrada de usuários via Botão A. 
    * `vTaskSaida`: Gerencia a lógica de saída de usuários via Botão B.
    * `vTaskReset`: Gerencia o reset do sistema via Botão do Joystick e interrupção.
* Uso eficiente de primitivas de sincronização do FreeRTOS:
    * `xSemaphoreCreateCounting()` para controle de vagas.
    * `xSemaphoreCreateBinary()` para sinalização de eventos de botões (especialmente reset por IRQ).
    * `xSemaphoreCreateMutex()` para garantir acesso seguro e exclusivo ao display OLED.
* Tratamento de interrupções para o botão de reset, desacoplando a lógica da ISR para uma tarefa específica.
* Lógica de debounce para os botões para evitar múltiplas ativações.

## Hardware Utilizado

* **RP2040 (Placa BitDogLab ou similar)** 
* Botões (Push-buttons para A e B, botão do Joystick)
* LED RGB (catodo comum ou anodo comum, conforme sua montagem)
* Buzzer passivo
* Display OLED SSD1306 (comunicação I2C)
* *(Opcional)* Matriz de LEDs WS2812 (se utilizada)

## Software e Ferramentas

* **FreeRTOS Kernel**
* **Pico SDK** para acesso ao hardware do RP2040
* Compilador C (GCC para ARM)
* **CMake** e **Ninja** (ou `make`) para o sistema de build
* **VS Code** com a extensão Raspberry Pi Pico (recomendado)
* *(Opcional)* PIO assembler para controle da matriz WS2812 (se utilizada)

## Como Executar o Projeto

1.  **Clone** o repositório:
    ```bash
    git clone <url_do_seu_repositorio>
    cd <nome_do_seu_repositorio>
    ```

2.  **Configure o Ambiente:**
    * Certifique-se de que o Pico SDK está instalado e as variáveis de ambiente (`PICO_SDK_PATH`) estão configuradas.
    * Se o FreeRTOS Kernel não for um submódulo ou não estiver em um caminho padrão, ajuste o `FREERTOS_KERNEL_PATH` no arquivo `CMakeLists.txt`.

3.  **Compile o projeto:**
    ```bash
    mkdir build
    cd build
    cmake ..
    make 
    # ou, se configurado para Ninja:
    # ninja
    ```

4.  **Carregue o Firmware:**
    * Conecte sua placa RP2040 ao computador enquanto segura o botão BOOTSEL.
    * Arraste o arquivo `.uf2` gerado (localizado em `build/nome_do_seu_projeto.uf2`) para o dispositivo de armazenamento massivo que aparece (RPI-RP2).

5.  **Teste:**
    * Pressione os botões A e B para simular a entrada e saída de usuários.
    * Pressione o botão do joystick para resetar o sistema.
    * Observe o comportamento do LED RGB, do display OLED e do buzzer.

## Vídeo de Demonstração

🎥 [Link para o vídeo de demonstração do projeto](SEU_LINK_AQUI)
*(Substitua SEU_LINK_AQUI pelo link real do seu vídeo no YouTube, Google Drive, etc.)*

## Licença

Este projeto pode ser utilizado livremente para fins educacionais e não comerciais.
Créditos: `<Seu Nome Aqui>`