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
 * File: $Id: porttimer.c,v 1.60 2013/08/13 15:07:05 Armink $
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- static functions ---------------------------------*/
//static struct rt_timer timer;
static TIM_HandleTypeDef timer;
static void prvvTIMERExpiredISR(void);
void Slave_timer_timeout_ind(void* parameter);
uint32_t slave_timer_tick;
static USHORT RT_TICK_PER_SECOND = 50;
/* ----------------------- Start implementation -----------------------------*/
BOOL xMBPortTimersInit(USHORT usTim1Timerout50us)
{
//    rt_timer_init(&timer, "slave timer",
//                   timer_timeout_ind, /* bind timeout callback function */
//                   RT_NULL,
//                   (50 * usTim1Timerout50us) / (1000 * 1000 / RT_TICK_PER_SECOND) + 1,
//                   RT_TIMER_FLAG_ONE_SHOT); /* one shot */
    timer = htim2;
    slave_timer_tick = (50 * usTim1Timerout50us) / RT_TICK_PER_SECOND;
    __HAL_TIM_CLEAR_IT(&timer, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&timer, TIM_IT_UPDATE);
    return TRUE;
}

void vMBPortTimersEnable()
{
//    rt_timer_start(&timer);
    __HAL_TIM_SET_COUNTER(&timer, 0);
    HAL_TIM_Base_Start(&timer);
}

void vMBPortTimersDisable()
{
//    rt_timer_stop(&timer);
    HAL_TIM_Base_Stop(&timer);
}

void prvvTIMERExpiredISR(void)
{
    (void) pxMBPortCBTimerExpired();
}

void Slave_timer_timeout_ind(void* parameter)
{
    prvvTIMERExpiredISR();
}
