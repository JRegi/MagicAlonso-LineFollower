#ifndef IR_NEC_H
#define IR_NEC_H

#include <stdint.h>

void ir_init(void);             // Configura GPIO, EXTI y Timer
int  ir_available(void);        // Devuelve 1 si hay nuevo código
uint32_t ir_read(void);         // Devuelve último código y lo limpia

#endif
