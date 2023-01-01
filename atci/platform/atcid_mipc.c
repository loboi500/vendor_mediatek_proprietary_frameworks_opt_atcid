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
#include <pthread.h>
#include "atcid_mipc.h"
#include "atcid_adaptation.h"
#include "property/mtk_properties.h"

#ifdef MTK_ATCI_MIPC
#include "mipc.h"
#include "mipc_msg.h"
#include "mipc_msg_tlv_api.h"
#include "mipc_msg_host.h"
#include "mipc_msg_tlv_api.h"
#include "platform_mtk.h"


#define ATCI_SIM "persist.vendor.service.atci.sim"
#define ENABLE_URC_PROP "persist.vendor.service.atci_urc.enable"

int mFdMIPC = -1;
bool isModemOff = false;

static pthread_t s_tid_md_status;
static pthread_t s_tid_atcid_urc;
static bool listen_URC = false;
bool atci_mipc_inited = false;


enum md_bc_event {
    MD_STA_EV_INVALID = 0,
    MD_STA_EV_RESET_REQUEST,
    MD_STA_EV_F_ASSERT_REQUEST,
    MD_STA_EV_STOP_REQUEST,
    MD_STA_EV_START_REQUEST,
    MD_STA_EV_ENTER_FLIGHT_REQUEST,
    MD_STA_EV_LEAVE_FLIGHT_REQUEST,
    MD_STA_EV_ENTER_FLIGHT_E_REQUEST,
    MD_STA_EV_LEAVE_FLIGHT_E_REQUEST,
    MD_STA_EV_HS1,
    MD_STA_EV_READY,
    MD_STA_EV_EXCEPTION,
    MD_STA_EV_STOP,
};

struct md_status_event {
    struct timeval time_stamp;
    int md_id;
    int event_type;
    char reason[32];
};

int connectToMPCI() {
    LOGATCI(LOG_INFO, "connectToMPCI");
    int fd[2] = {0};
    if (socketpair(AF_UNIX, SOCK_STREAM, 0, fd) >= 0) {
        mFdMIPC = fd[1];
        return fd[0];
    } else {
        LOGATCI(LOG_ERR, "socketpair failed. errno:%d", errno);
        return -1;
    }
    return 1;
}

static bool modemEnabled () {
    char buf[64];
    int ccci_fd = open("/sys/kernel/ccci/md_en", O_RDONLY);
    if (ccci_fd >= 0) {
        memset(buf, 0, sizeof(buf));
        int readCount = read(ccci_fd, buf, sizeof(buf));
        close(ccci_fd);

        if (readCount > 0) {
            LOGATCI(LOG_DEBUG, "md_en buf: %s", buf);
            if (strncmp(buf, "E", 1) == 0) {
                //MD1 is enabled
                LOGATCI(LOG_DEBUG, "MD Enabled");
                return true;
            }
        } else {
            LOGATCI(LOG_ERR, "read md_en status fail");
            return false;
        }
    } else {
        LOGATCI(LOG_ERR, "fail to open md_en");
    }
    return false;
}
static void waitModemReady () {
    //wait MD ready
    do {
        char buf[64];
        int ccci_fd = open("/sys/kernel/ccci/boot", O_RDONLY);
        if (ccci_fd >= 0) {
            memset(buf, 0, sizeof(buf));
            int readCount = read(ccci_fd, buf, sizeof(buf));
            close(ccci_fd);

            if (readCount > 0) {
                if (strncmp(buf, "md1:4", 5) == 0) {
                    //MD1 is ready
                    LOGATCI(LOG_DEBUG, "MD ready");
                    break;
                }
            } else {
                LOGATCI(LOG_ERR, "read boot status fail");
            }
        }
        sleep(1);
    } while (1);
}

static void mdStatusEventCb(void * priv_ptr, struct md_status_event_t * event_ptr) {
    LOGATCI(LOG_DEBUG, "mdStatusEventCb: event_type: %d", event_ptr->event_type);

    if (!isModemOff && MD_STA_EV_STOP == event_ptr->event_type) {
        LOGATCI(LOG_DEBUG, "reset init flag");
        isModemOff = true;

    } else if (isModemOff && MD_STA_EV_READY== event_ptr->event_type) {
        LOGATCI(LOG_DEBUG, "deinit after MD ready");
        //mipc_deinit();
        //atci_mipc_inited = false;
        isModemOff = false;
        listen_URC = false;
    }
}

static void registModemEvent() {
    LOGATCI(LOG_DEBUG, "registModemEvent");
    waitModemReady();

    LOGATCI(LOG_INFO, "atcid start to listen modem event!!!");
    if (!atci_mipc_inited) {
        SETCOM("/dev/ttyCMIPC2");
        mipc_init_debug("ttyCMIPC2");
        atci_mipc_inited = true;
    }
    mipc_md_event_register(mdStatusEventCb, NULL);
}

void monitorMdStatus() {
    int ret = 0;
    LOGATCI(LOG_DEBUG, "monitorMdStatus");
    if (!modemEnabled()) {
        LOGATCI(LOG_INFO, "modem disabled, skip monitor!!!");
        return;
    }
    ret = pthread_create(&s_tid_md_status, NULL, registModemEvent, NULL);
    if (ret != 0) {
        LOGATCI(LOG_ERR, "fail to create md status thread. errno:%d", errno);
        return;
    }

    LOGATCI(LOG_INFO, "create atcid md status successfully");
    pthread_join(s_tid_md_status, NULL);
}


void sendCommandByMIPC(char *data, int size) {
    run_at_cmd(data);
}

static void sendCommandResponse(const char* data) {

    int sendLen = 0;
    int len = 0;
    if (data != NULL) {
        len = strlen(data);
        sendLen = send(mFdMIPC, data, len, 0);

        LOGATCI(LOG_DEBUG, "sendCommandResponse to atcid. data = %s, sendLen = %d, len = %d", data, sendLen, len);

        if (sendLen != len) {
            LOGATCI(LOG_ERR, "lose data when sendCommandResponse to atcid. errno = %d, sendLen = %d, len = %d", errno, sendLen, len);
        }
    }
}

static void atcid_mipc_ind_cb(mipc_msg_t *msg_ptr, void *priv_ptr)
{
    char *urc_ptr;
    uint16_t urc_len;

    char enable_urc_prop[MTK_PROPERTY_VALUE_MAX] = {0};
    mtk_property_get(ENABLE_URC_PROP, enable_urc_prop, "0");
    int urc_enabled = atoi(enable_urc_prop);

    urc_ptr = mipc_sys_at_ind_get_atcmd(msg_ptr, &urc_len);

    LOGATCI(LOG_INFO, "urc_enabled=%d, URC to atcid =%s\n", urc_enabled, urc_ptr);

    if (urc_enabled == 1) {
        sendCommandResponse(urc_ptr);
    } else {
        LOGATCI(LOG_INFO, "URC disabled, ignore and unregister");
        mipc_msg_unregister_ind_api(MIPC_MSG_PS0, MIPC_SYS_AT_IND);
        listen_URC = false;
    }
}

static void waitMIPCUrc(void *param){
    UNUSED(param);

    waitModemReady();

    char enable_urc_prop[MTK_PROPERTY_VALUE_MAX] = {0};
    mtk_property_get(ENABLE_URC_PROP, enable_urc_prop, "0");
    int urc_enabled = atoi(enable_urc_prop);

    LOGATCI(LOG_INFO, "atcid start to wait for MIPC URC!!!");
    if(!listen_URC) {
        if (!atci_mipc_inited) {
            SETCOM("/dev/ttyCMIPC2");
            mipc_init_debug("ttyCMIPC2");
            atci_mipc_inited = true;
        }

        if (urc_enabled == 1) {
            mipc_msg_register_ind(MIPC_MSG_PS0, MIPC_SYS_AT_IND, (void*)atcid_mipc_ind_cb, NULL);
            listen_URC = true;
        }
    }
}

void startUrcThread(){
    int ret = 0;

    LOGATCI(LOG_INFO, "create atcid urc thread");
    ret = pthread_create(&s_tid_atcid_urc, NULL, waitMIPCUrc, NULL);
    if (ret != 0) {
        LOGATCI(LOG_ERR, "fail to create atcid urc thread. errno:%d", errno);
        return;
    }

    LOGATCI(LOG_INFO, "create atcid urc thread successfully");
    pthread_join(s_tid_atcid_urc, NULL);
}

void switchUrc(int on) {
    waitModemReady();

    LOGATCI(LOG_INFO, "switchUrc on: %d", on);
    if (on == 1) {
        if(!listen_URC) {
            if (!atci_mipc_inited) {
                SETCOM("/dev/ttyCMIPC2");
                mipc_init_debug("ttyCMIPC2");
                atci_mipc_inited = true;
            }
            LOGATCI(LOG_INFO, "URC enabled, register urc");
            mipc_msg_register_ind(MIPC_MSG_PS0, MIPC_SYS_AT_IND, (void*)atcid_mipc_ind_cb, NULL);
            listen_URC = true;
        } else {
            LOGATCI(LOG_INFO, "listen already, skip");
        }
    } else {
        if (listen_URC) {
            LOGATCI(LOG_INFO, "URC disabled, ignore and unregister");
            mipc_msg_unregister_ind_api(MIPC_MSG_PS0, MIPC_SYS_AT_IND);
            listen_URC = false;
        } else {
            LOGATCI(LOG_INFO, "not listen yet, skip");
        }
    }
}

void init_port() {
    LOGATCI(LOG_INFO, "init_port ttyCMIPC2");
    SETCOM("/dev/ttyCMIPC2");
    mipc_init_debug("ttyCMIPC2");
    atci_mipc_inited = true;
    LOGATCI(LOG_INFO, "set mipc_inited true");
}
void run_at_cmd(char* cmd)
{
    char atci_sim[MTK_PROPERTY_VALUE_MAX] = {0};
    mtk_property_get(ATCI_SIM, atci_sim, "0");
    int slotId = atoi(atci_sim);
    mipc_msg_t *msg_req_ptr;
    mipc_msg_t *msg_cnf_ptr;
    mipc_result_enum result;
    char *atcicmd_req_ptr, *atcmd_res_ptr, *atci_res;
    uint16_t atcmd_req_len, atcmd_res_len;

    LOGATCI(LOG_INFO, "AT command execution:%s", cmd);

    if (!modemEnabled()) {
        LOGATCI(LOG_INFO, "MD off, return");
        sendCommandResponse("ERROR:MIPC FAIL\r\n");
        return;
    }

    if (!listen_URC && !atci_mipc_inited) {
        init_port();
        mipc_md_event_register(mdStatusEventCb, NULL);
        LOGATCI(LOG_INFO, "register modem status after init mipc");
    }

    // add '\r' for all cmd
    // in MIPC spec, the string need including '\r' and the size passing need including '\0'
    atcmd_req_len = strlen(cmd);
    atcicmd_req_ptr = (char *)malloc(atcmd_req_len + 2);  // 1 for '\r', 1 for '\0'
    if (atcicmd_req_ptr == NULL) {
        LOGATCI(LOG_INFO, "fail to malloc atcicmd_req_ptr");
        return;
    }
    strncpy(atcicmd_req_ptr, cmd, atcmd_req_len);
    atcicmd_req_ptr[atcmd_req_len] = '\r';
    atcicmd_req_ptr[atcmd_req_len + 1] = '\0';

    LOGATCI(LOG_INFO, "AT command execution (with %s\r)", atcicmd_req_ptr);

    if (slotId == 1) {
        msg_req_ptr = mipc_msg_init(MIPC_SYS_AT_REQ, MIPC_PS1);
    } else {
        msg_req_ptr = mipc_msg_init(MIPC_SYS_AT_REQ, MIPC_PS0);
    }

    LOGATCI(LOG_INFO, "start mipc_msg_add_tlv");
    mipc_msg_add_tlv(msg_req_ptr, MIPC_SYS_AT_REQ_T_ATCMD, atcmd_req_len + 2, atcicmd_req_ptr);

    LOGATCI(LOG_INFO, "start mipc_msg_sync_timeout");
    //msg_cnf_ptr = mipc_msg_sync_timeout(msg_req_ptr,);
    mipc_ret_cause_enum cause = mipc_msg_sync_timeout_with_cause(msg_req_ptr, &msg_cnf_ptr);

    LOGATCI(LOG_INFO, "start mipc_msg_deinit");
    mipc_msg_deinit(msg_req_ptr);
    free(atcicmd_req_ptr);

    if (msg_cnf_ptr == NULL) {
        LOGATCI(LOG_INFO, "send command failed, cause: %d, isModemOff: %d", cause, isModemOff);
        sendCommandResponse("ERROR:MIPC FAIL\r\n");
        return;
    }

    LOGATCI(LOG_INFO, "start mipc_get_result");
    result = mipc_get_result(msg_cnf_ptr);

    if (result == MIPC_RESULT_SUCCESS) {
        atcmd_res_ptr = mipc_msg_get_val_ptr(msg_cnf_ptr, MIPC_SYS_AT_CNF_T_ATCMD, &atcmd_res_len);
        if (atcmd_res_ptr == NULL) {
            LOGATCI(LOG_INFO, "fail to get atcmd_res_ptr");
            return;
        }
        int res_len = strlen(atcmd_res_ptr);
        printf("res_len is:%d, atcmd_res_len is %d\n", res_len, atcmd_res_len);
        atci_res = (char *)malloc(res_len + 3); //2+1, 2 for adding \r\n before response, 1 for termination
        if (atci_res == NULL) {
            LOGATCI(LOG_INFO, "fail to malloc atci_res");
            return;
        }
        strncpy(atci_res, "\r\n", 2); //add \r\n
        strncpy(atci_res + 2, atcmd_res_ptr, res_len); //copy reponse string
        atci_res[res_len + 2] = '\0';  // add termination

        printf("AT response:%s with len:%d\n", atci_res, res_len);
        //send respons to atcid
        sendCommandResponse(atci_res);
        free(atci_res);
    } else {
        sendCommandResponse("Unknown error\n");
        printf("Failed to execute:%d\n", result);
    }
    mipc_msg_deinit(msg_cnf_ptr);

    if (!listen_URC) {
        //mipc_deinit();
    }
}
#else
//no MTK_ATCI_MIPC
void sendCommandByMIPC(char * data,int size) {
    return;
}
int connectToMPCI() {
    return -1;
}
void monitorMdStatus() {
    return;
}
void switchUrc(int on) {
    return;
}

#endif