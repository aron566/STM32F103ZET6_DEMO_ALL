/*
 *  FILE: UART_Port.h
 *
 *  Created on: 2020/2/22
 *
 *         Author: aron66
 *
 *  DESCRIPTION:--
 */
#ifndef UART_PORT_H
#define UART_PORT_H
#ifdef __cplusplus //use C compiler
extern "C" {
#endif
/*库接口*/
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
/*外部接口*/
#include "stm32f1xx_hal.h"
#include "usart.h"
#include "cmsis_os.h"
/*内部接口*/
#include "CircularQueue.h"
    
#define UART_MAX_NUM    6
    
typedef enum
{
    UART_NUM_0 = 0,
    UART_NUM_1,
    UART_NUM_2,
    UART_NUM_3,
    UART_NUM_4,
    UART_NUM_5,
    UART_NUM_6,
}Uart_num_t;

typedef struct
{
    UART_HandleTypeDef *phuart;      //uart端口
    DMA_HandleTypeDef  *phdma_rx;
    CQ_handleTypeDef *cb;           //环形队列
    uint8_t *RX_Buff_Temp;          //接收缓冲
    uint16_t MAX_RX_Temp;           //最大接收数量
    int Is_Half_Duplex;             //半双工模式
    osSemaphoreId *pRX_Sem;         //接收二值信号量
}Uart_Dev_info_t;

void Uart_Port_Init(void);
void USER_UART_IRQHandler(UART_HandleTypeDef *huart);
#ifdef __cplusplus //end extern c
}
#endif
#endif