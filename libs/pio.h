#ifndef PIO_H
#define PIO_H

//Declaração das funções do PIO
void set_one_led(int index, uint8_t r, uint8_t g, uint8_t b);
void initializePio();

//Declaração das definições gerais de botões do projeto
#define NUM_PIXELS 25
#define WS2812_PIN 7
#define BUTTON_A 5
#define BUTTON_B 6
#define PIN_LED_R 13
#define PIN_LED_B 12
#define PIN_LED_G 11
#define IS_RGBW false

#endif