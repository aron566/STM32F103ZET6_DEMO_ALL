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
/*��ӿ�*/
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
/*�ⲿ�ӿ�*/
#include "stm32f1xx_hal.h"
#include "usart.h"
#include "cmsis_os.h"
/*�ڲ��ӿ�*/
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
    UART_HandleTypeDef *phuart;      //uart�˿�
    DMA_HandleTypeDef  *phdma_rx;
    CQ_handleTypeDef *cb;           //���ζ���
    uint8_t *RX_Buff_Temp;          //���ջ���
    uint16_t MAX_RX_Temp;           //����������
    int Is_Half_Duplex;             //��˫��ģʽ
    osSemaphoreId *pRX_Sem;         //���ն�ֵ�ź���
}Uart_Dev_info_t;

void Uart_Port_Init(void);
void USER_UART_IRQHandler(UART_HandleTypeDef *huart);
#ifdef __cplusplus //end extern c
}
#endif
#endif