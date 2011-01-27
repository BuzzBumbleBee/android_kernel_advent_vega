/*
 * Copyright (c) 2009 NVIDIA Corporation.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of the NVIDIA Corporation nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "nvcommon.h"
#include "nvodm_battery.h"
#include "nvrm_gpio.h"
#include "nvodm_query_discovery.h"
#include "nvodm_query_gpio.h"
#include "nvos.h"

typedef struct NvOdmBatteryDeviceRec
{
    NvRmGpioPinHandle           hPin;
    NvU32                       PinCount;
    NvRmDeviceHandle            hRm;
    NvRmGpioHandle              hGpio;
    NvBool                      bBattPresent;
    NvBool                      bBattFull;
    const NvOdmGpioPinInfo      *pGpioPinInfo;

} NvOdmBatteryDevice;

/**
 * Gets the battery event.
 *
 * @param hDevice A handle to the EC.
 * @param pBatteryEvent Battery events
 *
 */
void NvOdmBatteryGetEvent(
     NvOdmBatteryDeviceHandle hDevice,
     NvU8   *pBatteryEvent)
{
    NvOdmBatteryDevice *pBattContext = NULL;

    pBattContext = (NvOdmBatteryDevice *)hDevice;

    *pBatteryEvent = 0;
}

NvBool NvOdmBatteryDeviceOpen(NvOdmBatteryDeviceHandle *hDevice,
                              NvOdmOsSemaphoreHandle *hOdmSemaphore)
{
    NvOdmBatteryDevice *pBattContext = NULL;
    NvU32 i;
    NvError NvStatus = NvError_Success;
    NvU32 PinState;

    pBattContext = NvOdmOsAlloc(sizeof(NvOdmBatteryDevice));
    if (!pBattContext)
    {
        NvOsDebugPrintf(("NvOdmOsAlloc failed to allocate pBattContext."));
        return NV_FALSE;
    }

    NvOdmOsMemset(pBattContext, 0, sizeof(NvOdmBatteryDevice));
    NvStatus = NvRmOpen(&pBattContext->hRm, 0);
    if (NvStatus != NvError_Success)
        goto Cleanup;

    NvStatus = NvRmGpioOpen(pBattContext->hRm, &pBattContext->hGpio);
    if (NvStatus != NvError_Success)
        goto Cleanup;

    pBattContext->pGpioPinInfo = NvOdmQueryGpioPinMap(
                                     NvOdmGpioPinGroup_Battery,
                                     0,
                                     &pBattContext->PinCount);
    if (pBattContext->pGpioPinInfo == NULL)
    {
        goto Cleanup;
    }
    for (i = 0; i < pBattContext->PinCount; i++ )
    {
        /*Need the pin 1 to be set to Output for charging of the battery. */
        if (i == 1)
        {
            NvRmGpioAcquirePinHandle(
                        pBattContext->hGpio,
                        pBattContext->pGpioPinInfo[i].Port,
                        pBattContext->pGpioPinInfo[i].Pin,
                        &pBattContext->hPin);
            if (!pBattContext->hPin)
            {
                goto Cleanup;
            }
            NvRmGpioConfigPins(pBattContext->hGpio, &pBattContext->hPin, 1, NvRmGpioPinMode_Output);
            PinState = NvRmGpioPinState_Low;
            NvRmGpioWritePins(pBattContext->hGpio, &pBattContext->hPin, &PinState,1);
        }
    }
    *hDevice = pBattContext;
    return NV_TRUE;
Cleanup:
    NvOdmBatteryDeviceClose(pBattContext);
    return NV_FALSE;
}

void NvOdmBatteryDeviceClose(NvOdmBatteryDeviceHandle hDevice)
{
    NvOdmBatteryDevice *pBattContext = NULL;
    pBattContext = (NvOdmBatteryDevice *)hDevice;

    if (pBattContext->hGpio)
    {
        NvRmGpioReleasePinHandles(pBattContext->hGpio, &pBattContext->hPin,
                  pBattContext->PinCount);
        NvRmGpioClose(pBattContext->hGpio);
    }

    if (pBattContext->hRm)
    {
        NvRmClose(pBattContext->hRm);
        pBattContext->hRm = NULL;
    }
    if (pBattContext)
        NvOdmOsFree(pBattContext);
}
/**
 * Gets the AC line status.
 *
 * @param hDevice A handle to the EC.
 * @param pStatus A pointer to the AC line
 *  status returned by the ODM.
 *
 * @return NV_TRUE if successful, or NV_FALSE otherwise.
 */
NvBool NvOdmBatteryGetAcLineStatus(
       NvOdmBatteryDeviceHandle hDevice,
       NvOdmBatteryAcLineStatus *pStatus)
{
    *pStatus = NvOdmBatteryAcLine_Offline;
    return NV_FALSE;
}


/**
 * Gets the battery status.
 *
 * @param hDevice A handle to the EC.
 * @param batteryInst The battery type.
 * @param pStatus A pointer to the battery
 *  status returned by the ODM.
 *
 * @return NV_TRUE if successful, or NV_FALSE otherwise.
 */
NvBool NvOdmBatteryGetBatteryStatus(
       NvOdmBatteryDeviceHandle hDevice,
       NvOdmBatteryInstance batteryInst,
       NvU8 *pStatus)
{
    *pStatus = NVODM_BATTERY_STATUS_UNKNOWN;
    return NV_FALSE;
}

/**
 * Gets the battery data.
 *
 * @param hDevice A handle to the EC.
 * @param batteryInst The battery type.
 * @param pData A pointer to the battery
 *  data returned by the ODM.
 *
 * @return NV_TRUE if successful, or NV_FALSE otherwise.
 */
NvBool NvOdmBatteryGetBatteryData(
       NvOdmBatteryDeviceHandle hDevice,
       NvOdmBatteryInstance batteryInst,
       NvOdmBatteryData *pData)
{
    NvOdmBatteryData BatteryData;

    BatteryData.BatteryAverageCurrent  = NVODM_BATTERY_DATA_UNKNOWN;
    BatteryData.BatteryAverageInterval = NVODM_BATTERY_DATA_UNKNOWN;
    BatteryData.BatteryCurrent         = NVODM_BATTERY_DATA_UNKNOWN;
    BatteryData.BatteryLifePercent     = NVODM_BATTERY_DATA_UNKNOWN;
    BatteryData.BatteryLifeTime        = NVODM_BATTERY_DATA_UNKNOWN;
    BatteryData.BatteryMahConsumed     = NVODM_BATTERY_DATA_UNKNOWN;
    BatteryData.BatteryTemperature     = NVODM_BATTERY_DATA_UNKNOWN;
    BatteryData.BatteryVoltage         = NVODM_BATTERY_DATA_UNKNOWN;

    *pData = BatteryData;
    return NV_FALSE;
}

/**
 * Gets the battery full life time.
 *
 * @param hDevice A handle to the EC.
 * @param batteryInst The battery type.
 * @param pLifeTime A pointer to the battery
 *  full life time returned by the ODM.
 *
 */
void NvOdmBatteryGetBatteryFullLifeTime(
     NvOdmBatteryDeviceHandle hDevice,
     NvOdmBatteryInstance batteryInst,
     NvU32 *pLifeTime)
{
    *pLifeTime = NVODM_BATTERY_DATA_UNKNOWN;
}


/**
 * Gets the battery chemistry.
 *
 * @param hDevice A handle to the EC.
 * @param batteryInst The battery type.
 * @param pChemistry A pointer to the battery
 *  chemistry returned by the ODM.
 *
 */
void NvOdmBatteryGetBatteryChemistry(
     NvOdmBatteryDeviceHandle hDevice,
     NvOdmBatteryInstance batteryInst,
     NvOdmBatteryChemistry *pChemistry)
{
    *pChemistry = NVODM_BATTERY_DATA_UNKNOWN;
}

