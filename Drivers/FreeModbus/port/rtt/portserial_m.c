/*
 * FreeModbus Libary: RT-Thread Port
 * Copyright (C) 2013 Armink <armink.ztl@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: portserial_m.c,v 1.60 2013/08/13 15:07:05 Armink add Master Functions $
 */

#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

#include "bsp.h"

#if MB_MASTER_RTU_ENABLED > 0 || MB_MASTER_ASCII_ENABLED > 0
/* ----------------------- Static variables ---------------------------------*/
//ALIGN(RT_ALIGN_SIZE)
/* software simulation serial transmit IRQ handler thread stack */
//static rt_uint8_t serial_soft_trans_irq_stack[512];
/* software simulation serial transmit IRQ handler thread */
//static struct rt_thread thread_serial_soft_trans_irq;

static osThreadId serial_soft_trans_irq_Handle;
static uint32_t serial_soft_trans_irq_stack[128];
static osStaticThreadDef_t master_transControlBlock;

/* serial event */
//static struct rt_event event_serial;
static StaticEventGroup_t   event_serial;      /*  事件存储，事件组  */
static EventGroupHandle_t   event_serial_Handle; /*  事件标志组句柄    */
/* modbus master serial device */
//static rt_serial_t *serial;
static UART_HandleTypeDef *huart_x;

/* ----------------------- Defines ------------------------------------------*/
/* serial transmit event */
#define EVENT_SERIAL_TRANS_START    (1<<0)

/* ----------------------- static functions ---------------------------------*/
static void prvvUARTTxReadyISR(void);
static void prvvUARTRxISR(void);
static BOOL serial_rx_ind(UART_HandleTypeDef *dev, uint16_t size);
static void serial_soft_trans_irq(void const *parameter);

/* ----------------------- Start implementation -----------------------------*/
BOOL xMBMasterPortSerialInit(UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits,
        eMBParity eParity)
{
    /**
     * set 485 mode receive and transmit control IO
     * @note MODBUS_MASTER_RT_CONTROL_PIN_INDEX need be defined by user
     */
//    rt_pin_mode(MODBUS_MASTER_RT_CONTROL_PIN_INDEX, PIN_MODE_OUTPUT);

    /* set serial name */
    switch (ucPORT)
    {
      case 1:
        huart_x = &huart1;
        HAL_UART_Abort(&huart1);
        MX_USART1_UART_Init();
        break;
      case 2:
        huart_x = &huart2;
        HAL_UART_Abort(&huart2);
        MX_USART2_UART_Init();
        break;
      case 3:
        
        break;
      default:
        return FALSE;
        break;
    }
    /* set serial configure parameter */

    /* set serial configure */

    /* open serial device */

    /* software initialize */
//    rt_event_init(&event_serial, "master event", RT_IPC_FLAG_PRIO);
      event_serial_Handle = xEventGroupCreateStatic(&event_serial);/*创建事件组，成功返回句柄*/
    //    rt_thread_init(&thread_serial_soft_trans_irq,
//                   "master trans",
//                   serial_soft_trans_irq,
//                   RT_NULL,
//                   serial_soft_trans_irq_stack,
//                   sizeof(serial_soft_trans_irq_stack),
//                   10, 5);
//    rt_thread_startup(&thread_serial_soft_trans_irq);
    osThreadStaticDef(master_trans, serial_soft_trans_irq, osPriorityNormal, 0, 128, serial_soft_trans_irq_stack, &master_transControlBlock);
    serial_soft_trans_irq_Handle = osThreadCreate(osThread(master_trans), NULL);
    return TRUE;
}

void vMBMasterPortSerialEnable(BOOL xRxEnable, BOOL xTxEnable)
{
//    rt_uint32_t recved_event;
    uint32_t recvedEvent;
    if (xRxEnable)
    {
        /* enable RX interrupt */
//        serial->ops->control(serial, RT_DEVICE_CTRL_SET_INT, (void *)RT_DEVICE_FLAG_INT_RX);
        __HAL_UART_ENABLE_IT( huart_x, UART_IT_RXNE );
        /* switch 485 to receive mode */
//        rt_pin_write(MODBUS_MASTER_RT_CONTROL_PIN_INDEX, PIN_LOW);
        HAL_GPIO_WritePin(RS485_SEL_GPIO_Port, RS485_SEL_Pin, GPIO_PIN_RESET);		//拉低RS485_SEL脚，RS485为接收状态
    }
    else
    {
        /* switch 485 to transmit mode */
//        rt_pin_write(MODBUS_MASTER_RT_CONTROL_PIN_INDEX, PIN_HIGH);
        HAL_GPIO_WritePin(RS485_SEL_GPIO_Port, RS485_SEL_Pin, GPIO_PIN_SET);		//拉高RS485_SEL脚，RS485为发送状态
        /* disable RX interrupt */
//        serial->ops->control(serial, RT_DEVICE_CTRL_CLR_INT, (void *)RT_DEVICE_FLAG_INT_RX);
        __HAL_UART_DISABLE_IT( huart_x, UART_IT_RXNE );
    }
    if (xTxEnable)
    {
        /* start serial transmit */
//        rt_event_send(&event_serial, EVENT_SERIAL_TRANS_START);
        __HAL_UART_ENABLE_IT( huart_x, UART_IT_TXE );
        xEventGroupSetBits(event_serial_Handle, EVENT_SERIAL_TRANS_START);    /*设置事件标志组 EVENT_SERIAL_TRANS_START 置1*/
    }
    else
    {
        /* stop serial transmit */
//        rt_event_recv(&event_serial, EVENT_SERIAL_TRANS_START,
//                RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 0,
//                &recved_event);
        recvedEvent = xEventGroupGetBits(event_serial_Handle);
        __HAL_UART_DISABLE_IT( huart_x, UART_IT_TXE );
    }
}

void vMBMasterPortClose(void)
{
    HAL_UART_Abort(huart_x);
}

BOOL xMBMasterPortSerialPutByte(CHAR ucByte)
{
//    serial->parent.write(&(serial->parent), 0, &ucByte, 1);
    HAL_UART_Transmit_DMA(huart_x,(uint8_t *)&ucByte, 1);
    return TRUE;
}

BOOL xMBMasterPortSerialGetByte(CHAR * pucByte)
{
//    serial->parent.read(&(serial->parent), 0, pucByte, 1);
    HAL_UART_Receive_DMA(huart_x, (uint8_t*)pucByte, 1);
    return TRUE;
}

/* 
 * Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call 
 * xMBPortSerialPutByte( ) to send the character.
 */
void prvvUARTTxReadyISR(void)
{
    pxMBMasterFrameCBTransmitterEmpty();
}

/* 
 * Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
void prvvUARTRxISR(void)
{
    pxMBMasterFrameCBByteReceived();
}

/**
 * Software simulation serial transmit IRQ handler.
 *
 * @param parameter parameter
 */
static void serial_soft_trans_irq(void const *parameter) {
//    rt_uint32_t recved_event;
    uint32_t recved_event;
    
    while (1)
    {
        /* waiting for serial transmit start */
//        rt_event_recv(&event_serial, EVENT_SERIAL_TRANS_START, RT_EVENT_FLAG_OR,
//                RT_WAITING_FOREVER, &recved_event);
        recved_event = xEventGroupGetBits(event_serial_Handle);
        /* execute modbus callback */
        prvvUARTTxReadyISR();
    }
}

/**
 * This function is serial receive callback function
 *
 * @param dev the device of serial
 * @param size the data size that receive
 *
 * @return return RT_EOK
 */
static BOOL serial_rx_ind(UART_HandleTypeDef *dev, uint16_t size) {
    prvvUARTRxISR();
    return TRUE;
}

#endif
