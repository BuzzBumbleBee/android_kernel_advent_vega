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

#ifndef INCLUDED_TPS6586X_BATTERYCHARGER_HEADER
#define INCLUDED_TPS6586X_BATTERYCHARGER_HEADER



#include "nvodm_pmu_tps6586x.h"

/* the battery charger functions */
#if defined(__cplusplus)
extern "C"
{
#endif

/* Initliase all registers that related to battery charger */
NvBool 
Tps6586xBatteryChargerSetup(NvOdmPmuDeviceHandle hDevice);

/* check CBC main batt presence */
NvBool
Tps6586xBatteryChargerCBCMainBatt(NvOdmPmuDeviceHandle hDevice, NvBool *status);

/* check batt_ful status */
NvBool
Tps6586xBatteryChargerCBCBattFul(NvOdmPmuDeviceHandle hDevice, NvBool *status);

/* check main charger status */
NvBool
Tps6586xBatteryChargerMainChgPresent(NvOdmPmuDeviceHandle hDevice, NvBool *status);

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_TPS6586X_BATTERYCHARGER_HEADER

