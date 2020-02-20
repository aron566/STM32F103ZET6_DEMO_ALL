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
 * File: $Id: portevent.c,v 1.60 2013/08/13 15:07:05 Armink $
 */

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Variables ----------------------------------------*/
//static struct rt_event     xSlaveOsEvent;
static StaticEventGroup_t   xSlaveOsEvent;      /*  事件存储，事件组  */
static EventGroupHandle_t   slave_event_Handle; /*  事件标志组句柄    */
/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortEventInit( void )
{
//    rt_event_init(&xSlaveOsEvent,"slave event",RT_IPC_FLAG_PRIO);
//    xEventGroupCreate();
    slave_event_Handle = xEventGroupCreateStatic(&xSlaveOsEvent);/*创建事件组，成功返回句柄*/
    return TRUE;
}

BOOL
xMBPortEventPost( eMBEventType eEvent )
{
//    rt_event_send(&xSlaveOsEvent, eEvent);
    xEventGroupSetBits(slave_event_Handle, eEvent);    /*设置事件标志组 eEvent 置1*/
    return TRUE;
}

BOOL
xMBPortEventGet( eMBEventType * eEvent )
{
//    rt_uint32_t recvedEvent;
    /* waiting forever OS event */
//    rt_event_recv(&xSlaveOsEvent,
//            EV_READY | EV_FRAME_RECEIVED | EV_EXECUTE | EV_FRAME_SENT,
//            RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER,
//            &recvedEvent);
    uint32_t recvedEvent;
    recvedEvent = xEventGroupGetBits(slave_event_Handle);
    switch (recvedEvent)
    {
    case EV_READY:
        *eEvent = EV_READY;
        break;
    case EV_FRAME_RECEIVED:
        *eEvent = EV_FRAME_RECEIVED;
        break;
    case EV_EXECUTE:
        *eEvent = EV_EXECUTE;
        break;
    case EV_FRAME_SENT:
        *eEvent = EV_FRAME_SENT;
        break;
    }
    return TRUE;
}
