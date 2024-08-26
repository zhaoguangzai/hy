#ifndef _ADC_CHANNEL_H
#define _ADC_CHANNEL_H



#include "stm32f10x.h"

// ??:??ADC???IO??????,??????????
/********************ADC1????(??)??**************************/
#define    ADC_APBxClock_FUN             RCC_APB2PeriphClockCmd
#define    ADC_CLK                       RCC_APB2Periph_ADC1

#define    ADC_GPIO_APBxClock_FUN        RCC_APB2PeriphClockCmd
#define    ADC_GPIO_CLK                  RCC_APB2Periph_GPIOA 
#define    ADC_PORT                      GPIOA

// ??
// 1-PC1 ???????????????
// 2-PC0 ?MINI?????????? IR,?????
// ?? PC0 ? ADC ???????,????????

// ??????
#define    NOFCHANEL										 3

#define    ADC_PIN1                      GPIO_Pin_5
#define    ADC_CHANNEL1                  ADC_Channel_5

#define    ADC_PIN2                      GPIO_Pin_6
#define    ADC_CHANNEL2                  ADC_Channel_6

#define    ADC_PIN3                      GPIO_Pin_7
#define    ADC_CHANNEL3                  ADC_Channel_7




// ADC1 ?? DMA1??1,ADC3??DMA2??5,ADC2??DMA??
#define    ADC_x                         ADC1
#define    ADC_DMA_CHANNEL               DMA1_Channel1
#define    ADC_DMA_CLK                   RCC_AHBPeriph_DMA1


/**************************????********************************/
void               ADCx_Init                               (void);

#endif /* __ADC_H */
