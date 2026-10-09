#ifndef __KEY_H
#define __KEY_H

#include "main.h" 

void Key_Init(void);
/* 松手事件：0=无，1=PE0，2=PB9。 */
uint8_t Key(void);
/* 原始按下位图：bit0=PE0，bit1=PB9。 */
uint8_t Key_GetState(void);
void Key_Loop(void);

#endif
