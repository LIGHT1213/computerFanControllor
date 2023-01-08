//
// Created by light on 2021/4/12.
//

#ifndef WS2812B_H
#include "main.h"
#define LedTotalNumer 40
#define T0H 18-1
#define T1H 72-1
#define WS2812_TIM htim8
typedef struct ws2812b
{
    uint8_t ledNumber;
    uint8_t Green[LedTotalNumer];
    uint8_t Red[LedTotalNumer];
    uint8_t Blue[LedTotalNumer];
		uint32_t TimChannel;
}ws2812b_t;
extern uint32_t ledPixelBuff[LedTotalNumer*24+4];
void SetLedColor(ws2812b_t *ws2812,uint8_t ledNumber,uint8_t G,uint8_t R,uint8_t B);
void Ws2812bUpdate(ws2812b_t *ws2812);
#define WS2812B_H

#endif //WS2812B_F411_WS2812B_H
