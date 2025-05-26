#ifndef DISPLAY_H
#define DISPLAY_H

// Declaração da função de display
void initI2C();
void draw_text(const char *texto, int x, int y);
void draw_rect(void);
void flush_display(void);
void interface_display(const char *texto,const char *vagas);
void limpar();

#endif