/*
 *  FILE: UART_Port.c
 *
 *  Created on: 2020/2/22
 *
 *         Author: aron66
 *
 *  DESCRIPTION:--
 */
#ifdef __cplusplus //use C compiler
extern "C" {
#endif
#include "UART_Port.h"/*外部接口*/
    
/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6; 
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart4_rx;
extern DMA_HandleTypeDef hdma_usart4_tx;
extern DMA_HandleTypeDef hdma_usart5_rx;
extern DMA_HandleTypeDef hdma_usart5_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

static uint8_t get_uart_index(USART_TypeDef *Instance);
static Uart_Dev_info_t *Create_Uart_Dev(Uart_num_t uart_num ,UART_HandleTypeDef *huart ,DMA_HandleTypeDef *hdma_rx ,uint16_t rx_temp_size ,uint32_t rxsize ,int work_mode ,osSemaphoreId *pRX_Sem);

/*预定义串口设备信息*/
Uart_Dev_info_t *Uart_pDevice[UART_MAX_NUM+1];
/**
  ******************************************************************
  * @brief   初始化串口设备信息
  * @author  aron66
  * @version v1.0
  * @date    2020/3/15
  ******************************************************************
  */
void Uart_Port_Init(void)
{
    Uart_pDevice[UART_NUM_1] = Create_Uart_Dev(UART_NUM_1 ,&huart1 ,&hdma_usart1_rx ,128 ,128 ,0 ,NULL);
    Uart_pDevice[UART_NUM_2] = Create_Uart_Dev(UART_NUM_2 ,&huart2 ,&hdma_usart2_rx ,128 ,128 ,0 ,NULL);

}

/**
  ******************************************************************
  * @brief   建立串口设备，为其建立双缓冲区-->使能串口空闲中断
  * @param   串口号 串口设备指针 dma操作地址 ，临时缓冲大小 接收队列大小 工作模式 二值信号量
  * @author  aron66
  * @version v1.0
  * @date    2020/3/15
  ******************************************************************
  */
static Uart_Dev_info_t *Create_Uart_Dev(Uart_num_t uart_num ,UART_HandleTypeDef *huart ,DMA_HandleTypeDef *hdma_rx ,uint16_t rx_temp_size ,uint32_t rxsize ,int work_mode ,osSemaphoreId *pRX_Sem)
{
    Uart_Dev_info_t *pUart_Dev = (Uart_Dev_info_t *)malloc(sizeof(Uart_Dev_info_t));
    pUart_Dev->phuart = huart;
    pUart_Dev->phdma_rx = hdma_rx;
    pUart_Dev->cb = cb_create(rxsize);
    pUart_Dev->MAX_RX_Temp = rx_temp_size;
    pUart_Dev->RX_Buff_Temp = (uint8_t *)malloc(sizeof(uint8_t)*rx_temp_size);
    if(NULL == pUart_Dev->RX_Buff_Temp)
	{
		return NULL;
	}
    pUart_Dev->Is_Half_Duplex = work_mode;
    pUart_Dev->pRX_Sem = pRX_Sem;

    //打开空闲中断
    __HAL_UART_ENABLE_IT(huart,UART_IT_IDLE);
    //使能DMA接收
    HAL_UART_Receive_DMA(huart, pUart_Dev->RX_Buff_Temp, pUart_Dev->MAX_RX_Temp);
    return pUart_Dev;
}

/************************************************************
  * @brief   Rx Transfer IRQ
  * @param   huart UART handle.
  * @return  None
  * @author  aron66
  * @date    2020/3/15
  * @version v1.0
  * @note    @@
  ***********************************************************/
void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    uint8_t index = get_uart_index(huart->Instance);
    if(index != 0)
    {
        if((__HAL_UART_GET_FLAG(Uart_pDevice[index]->phuart ,UART_FLAG_IDLE) != RESET))
        {
			/*
			首先停止DMA传输，
			1.防止后面又有数据接收到，产生干扰，因为此时的数据还未处理。
			2.DMA需要重新配置。
			*/
			HAL_UART_DMAStop(Uart_pDevice[index]->phuart);
			/*清楚空闲中断标志，否则会一直不断进入中断*/
			__HAL_UART_CLEAR_IDLEFLAG(Uart_pDevice[index]->phuart);
			/*计算本次接收数据长度*/
			uint32_t data_length  = Uart_pDevice[index]->MAX_RX_Temp - __HAL_DMA_GET_COUNTER(Uart_pDevice[index]->phdma_rx);
			/*将数据记录至环形区*/
			CQ_putData(Uart_pDevice[index]->cb ,Uart_pDevice[index]->RX_Buff_Temp ,(uint32_t)data_length);
            HAL_UART_Transmit(&huart1, (uint8_t *)Uart_pDevice[index]->RX_Buff_Temp,(uint16_t)data_length,0xFFFF);
			/*清空临时缓冲区*/
			memset(Uart_pDevice[index]->RX_Buff_Temp ,0 ,data_length);
			data_length = 0;
			/*打开空闲中断*/
			__HAL_UART_ENABLE_IT(Uart_pDevice[index]->phuart ,UART_IT_IDLE);
			/*重启开始DMA传输*/
			HAL_UART_Receive_DMA(Uart_pDevice[index]->phuart ,Uart_pDevice[index]->RX_Buff_Temp, Uart_pDevice[index]->MAX_RX_Temp);
        }
    }
}   

/*获得当前串口信息索引*/
static uint8_t get_uart_index(USART_TypeDef *Instance)
{
    uint8_t index = 0;
    for(;index < UART_MAX_NUM+1;index++)
    {
        if(Uart_pDevice[index]->phuart->Instance == Instance)
        {
            return index;
        }  
    }
    return 0;
}

#if (USE_NEW_REDIRECT == 0)
#include "stdio.h"
/*************************************************
  * 函数功能: 重定向c库函数printf到HAL_UART_Transmit
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 10);//原来使用阻塞式传输
  return ch;
}
/**
  * 函数功能: 重定向c库函数getchar,scanf
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
int fgetc(FILE * f)
{
  uint8_t ch = 0;
  while(HAL_UART_Receive(&huart1,&ch, 1, 0xffff)!=HAL_OK);
  return ch;
}

#else
/*新式重定向*/
#include "stdio.h"
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1 ,()uint8_t)&ch ,1 ,0xFFFF);
    return ch;
}
int __write(int file, char *ptr, int len)
{
    int DataIdx;
    for(DataIdx = 0; DataIdx < len; DataIdx++)
    {
        __io_putchar(*ptr++);
    }
    return len;
}
#endif

#ifdef __cplusplus //end extern c
}
#endif




