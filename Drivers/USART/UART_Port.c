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
#include "UART_Port.h"/*�ⲿ�ӿ�*/
    
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

/*Ԥ���崮���豸��Ϣ*/
Uart_Dev_info_t *Uart_pDevice[UART_MAX_NUM+1];
/**
  ******************************************************************
  * @brief   ��ʼ�������豸��Ϣ
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
  * @brief   ���������豸��Ϊ�佨��˫������-->ʹ�ܴ��ڿ����ж�
  * @param   ���ں� �����豸ָ�� dma������ַ ����ʱ�����С ���ն��д�С ����ģʽ ��ֵ�ź���
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

    //�򿪿����ж�
    __HAL_UART_ENABLE_IT(huart,UART_IT_IDLE);
    //ʹ��DMA����
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
			����ֹͣDMA���䣬
			1.��ֹ�����������ݽ��յ����������ţ���Ϊ��ʱ�����ݻ�δ����
			2.DMA��Ҫ�������á�
			*/
			HAL_UART_DMAStop(Uart_pDevice[index]->phuart);
			/*��������жϱ�־�������һֱ���Ͻ����ж�*/
			__HAL_UART_CLEAR_IDLEFLAG(Uart_pDevice[index]->phuart);
			/*���㱾�ν������ݳ���*/
			uint32_t data_length  = Uart_pDevice[index]->MAX_RX_Temp - __HAL_DMA_GET_COUNTER(Uart_pDevice[index]->phdma_rx);
			/*�����ݼ�¼��������*/
			CQ_putData(Uart_pDevice[index]->cb ,Uart_pDevice[index]->RX_Buff_Temp ,(uint32_t)data_length);
            HAL_UART_Transmit(&huart1, (uint8_t *)Uart_pDevice[index]->RX_Buff_Temp,(uint16_t)data_length,0xFFFF);
			/*�����ʱ������*/
			memset(Uart_pDevice[index]->RX_Buff_Temp ,0 ,data_length);
			data_length = 0;
			/*�򿪿����ж�*/
			__HAL_UART_ENABLE_IT(Uart_pDevice[index]->phuart ,UART_IT_IDLE);
			/*������ʼDMA����*/
			HAL_UART_Receive_DMA(Uart_pDevice[index]->phuart ,Uart_pDevice[index]->RX_Buff_Temp, Uart_pDevice[index]->MAX_RX_Temp);
        }
    }
}   

/*��õ�ǰ������Ϣ����*/
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
  * ��������: �ض���c�⺯��printf��HAL_UART_Transmit
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 10);//ԭ��ʹ������ʽ����
  return ch;
}
/**
  * ��������: �ض���c�⺯��getchar,scanf
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
int fgetc(FILE * f)
{
  uint8_t ch = 0;
  while(HAL_UART_Receive(&huart1,&ch, 1, 0xffff)!=HAL_OK);
  return ch;
}

#else
/*��ʽ�ض���*/
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




