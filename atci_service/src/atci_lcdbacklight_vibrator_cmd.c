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

#include "atci_lcdbacklight_vibrator_cmd.h"
#include "atci_service.h"
#include "atcid_util.h"
#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define DEV_VIBR_PATH    "/sys/class/timed_output/vibrator/vibr_on"
#define BACKLIGHT_DISABLE                   '0'
#define BACKLIGHT_ENABLE_FOR_THE_DURATION   '1'
#define BACKLIGHT_ENABLE_INDEFINITELY       '2'
#define BACKLIGHT_ENABLE_BY_UE              '3'
#define MAX_AT_RESPONSE_SIZE 2047

#define VIBR_ON     "1"
#define VIBR_OFF    "0"

char vibrator_state = '0';
char backlight_state = '0';
#define MAX_AT_RESPONSE 2047


int lcdbacklight_power_on_cmd_handler(__attribute__((unused)) char *cmdline, ATOP_t at_op, char *response) {
    int ret = 0;
    switch(at_op) {
        case AT_ACTION_OP:
        case AT_READ_OP:
        case AT_TEST_OP:
        case AT_SET_OP:
            ret = execve("echo 254 > /sys/class/leds/lcd-backlight/brightness", NULL, NULL);
            break;
        default:
            break;
    }
    int bytes = snprintf(response, MAX_AT_RESPONSE, "\r\n\r\nLEDON OK\r\n\r\n");
    if (bytes < 0 || bytes >= MAX_AT_RESPONSE || ret < 0) {
        return -1;
    }

    return 0;

}

int lcd_backlight_cmd_handler(char *cmdline, ATOP_t at_op, char *response) {
    char log_info[100] = {'\0'};
    unsigned int duration = 0;
    int bytes = 0, ret = 0;
    char state = cmdline[0];
    switch(at_op) {
        case AT_ACTION_OP:
        case AT_READ_OP:
            bytes = snprintf(log_info, sizeof(log_info), "\r\n+CBKLT: %c,%d\r\n", backlight_state, duration);
            break;
        case AT_TEST_OP:
            bytes = snprintf(log_info, sizeof(log_info), "\r\n+CBKLT: (0, 1, 2)\r\n");
            break;
        case AT_SET_OP:
            if (state == BACKLIGHT_DISABLE || state == BACKLIGHT_ENABLE_FOR_THE_DURATION || \
                state == BACKLIGHT_ENABLE_INDEFINITELY || state == BACKLIGHT_ENABLE_BY_UE) {
                backlight_state = state;
                if(backlight_state == '0')
                {
                    bytes = snprintf(log_info, sizeof(log_info), "\r\nBacklight Disable\r\n");
                    ret = execve("echo 0 > /sys/class/leds/lcd-backlight/brightness", NULL, NULL);
                }else if(backlight_state == '1') {
                    if (sscanf(cmdline, "%*[^,],%d", &duration) != 1) {
                        return -1;
                    }
                    bytes = snprintf(log_info, sizeof(log_info), "\r\nBacklight Enable for %d second\r\n", duration);
                    ret = execve("echo 255 > /sys/class/leds/lcd-backlight/brightness", NULL, NULL);
                    if (ret >= 0) {
                        sleep(duration);
                        ret = execve("echo 0 > /sys/class/leds/lcd-backlight/brightness", NULL, NULL);
                    }
                }else if(backlight_state == '2') {
                    bytes = snprintf(log_info, sizeof(log_info), "\r\nBacklight Enable Indefinitely\r\n");
                    ret = execve("echo 255 > /sys/class/leds/lcd-backlight/brightness", NULL, NULL);
                } else {
                    bytes = snprintf(log_info, sizeof(log_info), "\r\nCME ERROR: 50\r\n");
                }
            } else {
                bytes = snprintf(log_info, sizeof(log_info), "\r\n+CME ERROR: 50\r\n");
            }
            break;
        default:
            break;
    }
    if (bytes < 0 || bytes >= sizeof(log_info) || ret < 0) {
        return -1;
    }

    bytes = snprintf(response, MAX_AT_RESPONSE, "\r\n%s\n\r\n", log_info);
    if (bytes < 0 || bytes >= MAX_AT_RESPONSE) {
        return -1;
    }
    return 0;
}

int vibrator_power_off_cmd_handler(char *cmdline, ATOP_t at_op, char *response) {
    int fd = 0, bytes = 0;
    char log_info[100] = {'\0'};

    ALOGD("handle cmdline:%s", cmdline);
    fd = open(DEV_VIBR_PATH, O_RDWR);
    if(fd < 0) {
        bytes = snprintf(log_info, sizeof(log_info), "Open FD error");
        if (bytes < 0 || bytes >= sizeof(log_info)) {
            ALOGE("write log error: Open FD error!");
        }
        goto error;
    }
    switch(at_op) {
        case AT_ACTION_OP:
        case AT_READ_OP:
        case AT_TEST_OP:
            bytes = snprintf(log_info, sizeof(log_info), "\r\n%c\r\n", vibrator_state);
            break;
        case AT_SET_OP:
            {
                if (strcmp(cmdline, VIBR_ON) == 0 || strcmp(cmdline, VIBR_OFF) == 0) {
                    vibrator_state = cmdline[0];
                    if(vibrator_state == '1') {
                        bytes = snprintf(log_info, sizeof(log_info), "MOTOR ON");
                        write(fd, VIBR_ON, 1);
                    } else {
                        bytes = snprintf(log_info, sizeof(log_info), "MOTOR OFF");
                        write(fd, VIBR_OFF, 1);
                    }
                } else {
                    if(vibrator_state == 0 )
                        bytes = snprintf(log_info, sizeof(log_info), "\r\n%c\r\n", vibrator_state);
                    else
                        bytes = snprintf(log_info, sizeof(log_info), "MOTOR ERROR");
                }
            }
            break;
        default:
            bytes = snprintf(log_info, sizeof(log_info), "MOTOR ERROR%s", cmdline);
            break;
    }
    if (bytes < 0 || bytes >= sizeof(log_info)) {
        ALOGE("write log_info error!");
    }

error:

    bytes = snprintf(response, sizeof(MAX_AT_RESPONSE), "\r\n%s\n\r\n", log_info);
    if (bytes < 0 || bytes >= MAX_AT_RESPONSE) {
        ALOGE("write response error!");
    }

    if (fd >= 0) {
        close(fd);
        fd = -1;
    }
    return 0;

}

