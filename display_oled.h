#ifndef DISPLAY_H
#define DISPLAY_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "inc/ssd1306.h"

// Definição dos pinos I2C
extern const uint I2C_SDA;
extern const uint I2C_SCL;

// Declarações das funções
void inicializar_display();
void cachorro();

#endif // DISPLAY_H