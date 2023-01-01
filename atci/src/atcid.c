/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein is
 * confidential and proprietary to MediaTek Inc. and/or its licensors. Without
 * the prior written permission of MediaTek inc. and/or its licensors, any
 * reproduction, modification, use or disclosure of MediaTek Software, and
 * information contained herein, in whole or in part, shall be strictly
 * prohibited.
 *
 * MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER
 * ON AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL
 * WARRANTIES, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR
 * NONINFRINGEMENT. NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH
 * RESPECT TO THE SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY,
 * INCORPORATED IN, OR SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES
 * TO LOOK ONLY TO SUCH THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO.
 * RECEIVER EXPRESSLY ACKNOWLEDGES THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO
 * OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES CONTAINED IN MEDIATEK
 * SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE
 * RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S
 * ENTIRE AND CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE
 * RELEASED HEREUNDER WILL BE, AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE
 * MEDIATEK SOFTWARE AT ISSUE, OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE
 * CHARGE PAID BY RECEIVER TO MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek
 * Software") have been modified by MediaTek Inc. All revisions are subject to
 * any receiver's applicable license agreements with MediaTek Inc.
 */

#include "atcid.h"
#include "atcid_serial.h"

#include <sys/socket.h>
#include <cutils/sockets.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

/*misc global vars */
Serial serial;
bool supportMIPC;

#define PROP_ATM_MODE "ro.boot.atm"
#define BUILD_TYPE_PROP "ro.vendor.build.type"

#define BOOTMODE_PATH "/proc/device-tree/chosen/atag,boot"  //for common kernel


extern int s_fdMeta;
extern int adb_socket_listen(int socketFd);
extern void connectTarget(ATCI_DataType dataType);
extern int metaReaderLoop();

/*
* Purpose:  The main program loop
* Return:    0
*/
int main() {
    int i = 0;
    char prop[MTK_PROPERTY_VALUE_MAX] = {0};
    LOGATCI(LOG_INFO, "atcid-daemon start!!!");

    mtk_property_get("ro.vendor.mtk_mipc_support", prop, "");
    if (strcmp(prop, "1") == 0) {
        supportMIPC = true;
        memset(prop, 0, sizeof(prop));
        mtk_property_get("persist.vendor.service.atci.rild", prop, "");
        if (strcmp(prop, "1") == 0){
            LOGATCI(LOG_ERR, "conenct to modem via rild");
            supportMIPC = false;
        }
    }
    int bootMode = readSys_int(BOOTMODE_PATH);
    //Initial the parameter for serial dervice
    initSerialDevice(&serial, bootMode);
    if (snprintf(serial.devicename[0], strlen(TTY_GS0) + 1, "%s", TTY_GS0) < 0) {
        LOGATCI(LOG_ERR, "snprintf devicename fail");
    }

#ifdef __ATCI_ENABLE__
    int enableAtciUserLoad = 1;
    LOGATCI(LOG_INFO, "enable atci even userload");
#else
    int enableAtciUserLoad = 0;
#endif

    int listenVCOM = 1;
    int metaMode = 0;
    int atmMode = 0;

    if (bootMode == FACTORY_BOOT){
        int UART_Port_ID = getUartPort();
        if (UART_Port_ID > 0) {
            listenVCOM = 0;
            serial.fd[UART_CONN_NUM] = open_uart(&serial, UART_Port_ID);
        }
    }

    char buildType[MTK_PROPERTY_VALUE_MAX +1] = {0};
    mtk_property_get(BUILD_TYPE_PROP, buildType, "");
    if (enableAtciUserLoad != 1 && strncmp(buildType, "user", strlen(buildType)) == 0) {
        listenVCOM = 0;
    }

    if (bootMode == META_BOOT) {
        LOGATCI(LOG_INFO, "meta boot, don't listen VCOM");
        listenVCOM = 0;
        metaMode = 1;
    }

    char buf[MTK_PROPERTY_VALUE_MAX + 1] = {0};
    mtk_property_get(PROP_ATM_MODE, buf, "");
    if (strncmp(buf, "enable", strlen("enable")) == 0) {
        LOGATCI(LOG_INFO, "atm mode, don't listen");
        listenVCOM = 0;
        atmMode = 1;
    }

    //always connect adb socket
    serial.fd[ADB_SKT_SERVER_NUM] = android_get_control_socket(ADB_ATCI_SOCKET);
    if (serial.fd[ADB_SKT_SERVER_NUM] < 0) {
        LOGATCI(LOG_ERR, "Failed to get socket '%s'", ADB_ATCI_SOCKET);
    } else {
        if (ATCID_SUCCESS != adb_socket_listen(serial.fd[ADB_SKT_SERVER_NUM])) {
            LOGATCI(LOG_ERR, "Could not open adb socket [%d]", i);
        }
    }

    //always register hidl service
    prepareHidlSocket();

    //if start with meta mode, waiting for calibration_stop from meta_tst
    if (metaMode == 1) {
        do {
            LOGATCI(LOG_INFO, "meta mode, connect meta socket");
            usleep(500 * 1000);
            connectTarget(META_TYPE);
        } while (s_fdMeta < 0);

        while (metaReaderLoop() < 0) {
            LOGATCI(LOG_ERR, "Error when receiving from meta");
            connectTarget(META_TYPE);
            usleep(1000 * 1000);
        }
        //received "calibration_stop" from meta_tst, connect to VCOM
        listenVCOM = 1;
    }
    if (atmMode == 1) {
        do {
            LOGATCI(LOG_INFO, "atm mode, connect meta socket");
            usleep(500 * 1000);
            connectTarget(META_TYPE);
        } while (s_fdMeta < 0);
    }

    if (listenVCOM == 1) {
        int count = 0;
        while (i < MAX_DEVICE_VCOM_NUM) {
            if ((serial.fd[i] = open_serial_device(&serial, serial.devicename[i])) ==
                    INVALIDE_SOCKET_FD) {
                LOGATCI(LOG_ERR, "Could not open serial device [%d] wait 2s and retry", i);
                if (count < 20) {
                    usleep(2000*1000);
                    count++;
                    continue;
                } else {
                    LOGATCI(LOG_ERR, "Could not open serial device [%d] and time out", i);
                }
            }
            i++;
        }
    }

    readerLoop((void*) &serial);

    return 0;
}
