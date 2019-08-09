/*
 * Copyright (c) 2016, NXP Semiconductor, Inc.
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "wifi_common.h"
#include "mbed_assert.h"
#include "mbed_critical.h"

/*TODO: check return value */
void a_free(void *addr, uint8_t id)
{
    // UNUSED_ARGUMENT(id);
    free(addr);
}

/* FIXME !! */
extern uint32_t g_totAlloc;
void *a_malloc(int32_t size, uint8_t id)
{
    void *addr;

    addr = (void *)malloc(size);

    if (addr != NULL) {
        /*FIXME: !!!*/
        g_totAlloc += size;
    }
    // UNUSED_ARGUMENT(id);

    return addr;
}

mbed_rtos_storage_mutex_t _mutex_mem;

A_STATUS a_mutex_init(osMutexId_t *pMutex)
{
    osMutexAttr_t attr = { 0 };
    attr.name = "qca400x_mutex";
    attr.cb_mem = &_mutex_mem;
    attr.cb_size = sizeof(_mutex_mem);
    attr.attr_bits = osMutexRecursive | osMutexPrioInherit | osMutexRobust;
    *pMutex = osMutexNew(&attr);

    if (NULL == *pMutex) {
        return A_ERROR;
    } else {
        return A_OK;
    }
}

A_STATUS a_mutex_acquire(osMutexId_t *pMutex)
{
    osStatus status = osMutexAcquire(*pMutex, osWaitForever);

    if (status != osOK) {
        return A_ERROR;
    } else {
        return A_OK;
    }
}

A_STATUS a_mutex_release(osMutexId_t *pMutex)
{
    osStatus status = osMutexRelease(*pMutex);

    if (status != osOK) {
        return A_ERROR;
    } else {
        return A_OK;
    }
}

boolean a_is_mutex_valid(osMutexId_t *pMutex)
{
    // FIXME: check owner of mutex
    return true;
}

A_STATUS a_mutex_delete(osMutexId_t *pMutex)
{
    osMutexDelete(*pMutex);

    return A_OK;
}

mbed_rtos_storage_event_flags_t _event_mem;

A_STATUS a_event_delete(event_t *pEvent)
{
    MBED_ASSERT(pEvent);
    osEventFlagsDelete(pEvent->eventHandler);

    return A_OK;
}

A_STATUS a_event_init(event_t *pEvent, osa_event_clear_mode_t clearMode)
{
    osEventFlagsAttr_t attr = { 0 };
    attr.name = "qca400x_event_flags";
    attr.cb_mem = &_event_mem;
    attr.cb_size = sizeof(_event_mem);
    pEvent->eventHandler = osEventFlagsNew(&attr);

    if (pEvent->eventHandler) {
        pEvent->clearMode = clearMode;
        return A_ERROR;
    } else {
        return A_OK;
    }
}

A_STATUS a_event_clear(event_t *pEvent, uint32_t flagsToClear)
{
    MBED_ASSERT(pEvent);

    osEventFlagsClear(pEvent->eventHandler, flagsToClear);

    return A_OK;
}

A_STATUS a_event_set(event_t *pEvent, uint32_t flagsToSet)
{
    MBED_ASSERT(pEvent);

    osEventFlagsSet(pEvent->eventHandler, flagsToSet);

    return A_OK;
}

A_STATUS a_event_wait(
    event_t *pEvent, uint32_t flagsToWait, boolean waitAll, uint32_t timeout, uint32_t *setFlags)
{
    MBED_ASSERT(pEvent);
    uint32_t flagsSave;
    uint32_t clearMode;

    clearMode = (kEventAutoClear == pEvent->clearMode) ? 0 : osFlagsNoClear;

    if (waitAll) {
        flagsSave = osEventFlagsWait(pEvent->eventHandler, flagsToWait, osFlagsWaitAll | clearMode, timeout);
    } else {
        flagsSave = osEventFlagsWait(pEvent->eventHandler, flagsToWait, osFlagsWaitAny | clearMode, timeout);
    }

    *setFlags = flagsSave & flagsToWait;

    if ((flagsSave & osFlagsError) || !(flagsSave & flagsToWait)) {
        return A_TIMEOUT; // TODO: unify with caller
    } else {
        return A_OK;

    }
}

uint32_t a_time_get_msec(void)
{
    uint32_t ticks;

    ticks = osKernelGetTickCount();

    return TICKS_TO_MSEC(ticks);
}

void OSA_EnterCritical(osa_critical_section_mode_t mode)
{
    core_util_critical_section_enter();
}

void OSA_ExitCritical(osa_critical_section_mode_t mode)
{
    core_util_critical_section_exit();
}
