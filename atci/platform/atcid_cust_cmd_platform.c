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

#include "property/mtk_properties.h"
#include <stdlib.h>
#include "libwifitest.h"
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <ctype.h>
#include <unistd.h>
#include "libnvram.h"
#include "libfile_op.h"

#include "atcid_serial.h"
#include "atcid_util.h"
#include "atcid_cust_cmd_platform.h"
#include "at_tok.h"

#define MAC_ADDR_SIZE 6

int wifiEnableState = -1;
int wifiBand = -1;
uint32_t wifiFreq = 0;
uint32_t wifiRate = 0;
uint32_t wifiGain = 0;
int wifiChannelBandwidth = -1;
int wifiDataBandwidth = -1;
int wifiGain2 = 0;
int wifiTxPacketLength = 0;
int wifiTxPacketCount = 0;
int wifiTxPacketinterval = 0;
int wifiCompen = 0;
int wifiPrimaryChannelOffset = 0;
int wifiTxDataRate = 0;

int gRateCodeWiFi = 0;

extern int sendATCommandToServiceWithResult(char* line);

ATRESPONSE_t pas_wienable_handler(char* cmdline, ATOP_t opType, char* response) {
    int err = 0, cmdID = 0;
    bool b = false;
    int r = -1;

    switch (opType) {
        case AT_SET_OP:
            err = at_tok_nextint(&cmdline, &cmdID);

            if (err < 0) return -1;
            switch (cmdID) {
                case 0:
                    if (wifiEnableState == 1) {
                        r = sendATCommandToServiceWithResult("AT+WITOF=1");
                        if(r != -1)
                            wifiEnableState = 0;
                        else
                            return AT_ERROR;
                    } else {
#ifdef MTK_WLAN_FEATURE
                        b = WIFI_TEST_CloseDUT();
#endif
                        if (b)
                            wifiEnableState = 0;
                        else
                            return AT_ERROR;
                    }
                    return AT_OK;
                case 1:
                    r = sendATCommandToServiceWithResult("AT+WITOF=2");
                    if (r != -1)
                        wifiEnableState = 1;
                    else
                        return AT_ERROR;
                    return AT_OK;
                case 2:
#ifdef MTK_WLAN_FEATURE
                    b = WIFI_TEST_OpenDUT();
#endif
                    if (b)
                        wifiEnableState = 2;
                    else
                        return AT_ERROR;
                    return AT_OK;
            }
            break;
        case AT_READ_OP:
            if (sprintf(response, "%d", wifiEnableState) < 0) {
                LOGATCI(LOG_ERR, "sprintf failed: %d", __LINE__);
                return AT_ERROR;
            }
            return AT_OK;
        default:
            break;

    }

    return AT_ERROR;
}

ATRESPONSE_t pas_wimode_handler(char* cmdline, ATOP_t opType, char* response) {
    int err = 0, cmdID = 0;
    bool b = false;
    uint32_t mode = 0;

    switch (opType) {
        case AT_SET_OP:
            err = at_tok_nextint(&cmdline, &cmdID);

            if (err < 0) return -1;
            switch (cmdID) {
                case 0:
#ifdef MTK_WLAN_FEATURE
                    b = WIFI_TEST_SetMode(WIFI_TEST_MODE_BY_API_CONTROL);
#endif
                    break;
                case 1:
#ifdef MTK_WLAN_FEATURE
                    b = WIFI_TEST_SetMode(WIFI_TEST_MODE_CW_ONLY);
#endif
                    break;
                case 2:
#ifdef MTK_WLAN_FEATURE
                    b = WIFI_TEST_SetMode(WIFI_TEST_MODE_80211A_ONLY);
#endif
                    break;
                case 3:
#ifdef MTK_WLAN_FEATURE
                    b = WIFI_TEST_SetMode(WIFI_TEST_MODE_80211B_ONLY);
#endif
                    break;
                case 4:
#ifdef MTK_WLAN_FEATURE
                    b = WIFI_TEST_SetMode(WIFI_TEST_MODE_80211G_ONLY);
#endif
                    break;
                case 5:
#ifdef MTK_WLAN_FEATURE
                    b = WIFI_TEST_SetMode(WIFI_TEST_MODE_80211N_ONLY);
#endif
                    break;
                case 6:
#ifdef MTK_WLAN_FEATURE
                    b = WIFI_TEST_SetMode(WIFI_TEST_MODE_80211AC_ONLY);
#endif
                    break;
            }
            if (b)
                return AT_OK;
            else
                return AT_ERROR;
        case AT_TEST_OP:
#ifdef MTK_WLAN_FEATURE
            b = WIFI_TEST_GetSupportedMode(&mode);
#endif
            if (sprintf(response, "%d", mode) < 0) {
                LOGATCI(LOG_ERR, "sprintf failed: %d", __LINE__);
                return AT_ERROR;
            }
            break;
        default:
            break;
    }
    if (b)
        return AT_OK;
    else
        return AT_ERROR;
}

ATRESPONSE_t pas_wiband_handler(char* cmdline, ATOP_t opType, char* response) {
    int err = 0, cmdID = 0;
    bool b = false;
    switch (opType) {
        case AT_SET_OP:
            err = at_tok_nextint(&cmdline, &cmdID);
            if (err < 0) return -1;
            switch (cmdID) {
                case 0:
#ifdef MTK_WLAN_FEATURE
                    b = WIFI_TEST_SetBandwidth(WIFI_TEST_BW_20MHZ);
#endif
                    if (b) {
                        if (sprintf(response, "20MHZ") < 0) {
                            LOGATCI(LOG_ERR, "sprintf failed: %d", __LINE__);
                            return AT_ERROR;
                        }
                        wifiBand = 0;
                    }
                    break;
                case 1:
#ifdef MTK_WLAN_FEATURE
                    b = WIFI_TEST_SetBandwidth(WIFI_TEST_BW_40MHZ);
#endif
                    if (b) {
                        if (sprintf(response, "40MHZ") < 0) {
                            LOGATCI(LOG_ERR, "sprintf failed: %d", __LINE__);
                            return AT_ERROR;
                        }
                        wifiBand = 1;
                    }
                    break;
            }
            if (b)
                return AT_OK;
            else
                return AT_ERROR;
        case AT_READ_OP:
            if (wifiBand == 0) {
                if (sprintf(response, "20MHZ") < 0) {
                    LOGATCI(LOG_ERR, "sprintf failed: %d", __LINE__);
                    return AT_ERROR;
                }
                return AT_OK;
            } else if(wifiBand == 1) {
                if (sprintf(response, "40MHZ") < 0) {
                    LOGATCI(LOG_ERR, "sprintf failed: %d", __LINE__);
                    return AT_ERROR;
                }
                return AT_OK;
            } else {
                return AT_ERROR;
            }
            break;
        default:
            break;
    }
    return AT_ERROR;
}

ATRESPONSE_t pas_wifreq_handler(char* cmdline, ATOP_t opType, char* response) {
    int err = 0;
    bool b = false;
    uint32_t freq = 0, offset = 0;

    switch (opType) {
        case AT_SET_OP:
            err = at_tok_nextint(&cmdline, (int *)&freq);
            if (err < 0) return -1;
#ifdef MTK_WLAN_FEATURE
            b = WIFI_TEST_SetFrequency(freq, offset);
#endif
            if (b) {
                wifiFreq = freq;
                return AT_OK;
            } else {
                wifiFreq = -1;
                return AT_ERROR;
            }
        case AT_READ_OP:
            if (sprintf(response, "%d", wifiFreq) < 0) {
                LOGATCI(LOG_ERR, "sprintf failed: %d", __LINE__);
                return AT_ERROR;
            }
            return AT_OK;
        default:
            break;
    }
    return AT_ERROR;
}

ATRESPONSE_t pas_widatarate_handler(char* cmdline, ATOP_t opType, char* response) {
    int err = 0;
    bool b = false;
    uint32_t rate = 0;

    switch (opType) {
        case AT_SET_OP:
            err = at_tok_nextint(&cmdline, (int *)&rate);
            if (err < 0) return -1;
#ifdef MTK_WLAN_FEATURE
            b = WIFI_TEST_SetRate(rate);
#endif
            if (b) {
                wifiRate = rate;
                LOGATCI(LOG_DEBUG, "pas_widatarate_handler [rate=%d] \n", wifiRate);
                gRateCodeWiFi = wifiRate;
                return AT_OK;
            } else {
                wifiRate = -1;
                return AT_ERROR;
            }
        case AT_READ_OP:
            if (sprintf(response, "%d", wifiRate) < 0) {
                LOGATCI(LOG_ERR, "sprintf failed: %d", __LINE__);
                return AT_ERROR;
            }
            return AT_OK;
        default:
            break;
    }
    return AT_ERROR;
}

ATRESPONSE_t pas_wipow_handler(char* cmdline, ATOP_t opType, char* response) {
    int err = 0;
    bool b = false;
    uint32_t gain = 0;

    switch (opType) {
        case AT_SET_OP:
            err = at_tok_nextint(&cmdline, (int *)&gain);
            if (err < 0) return -1;
#ifdef MTK_WLAN_FEATURE
            b = WIFI_TEST_SetTXPower(gain);
#endif
            if (b) {
                wifiGain = gain;
                return AT_OK;
            } else {
                wifiGain = -1;
                return AT_ERROR;
            }
        case AT_READ_OP:
            if (sprintf(response, "%d", wifiGain) < 0) {
                LOGATCI(LOG_ERR, "sprintf failed: %d", __LINE__);
                return AT_ERROR;
            }
            return AT_OK;
        default:
            break;
    }
    return AT_ERROR;
}

ATRESPONSE_t pas_witxpow_handler(char* cmdline, ATOP_t opType, char* response) {
    int err = 0, gain = 0;
    bool b = false;

    switch (opType) {
        case AT_SET_OP:
            err = at_tok_nextint(&cmdline, &gain);
            if (err < 0) return -1;
#ifdef MTK_WLAN_FEATURE
            b = WIFI_TEST_TxGain(gain);
#endif
            wifiGain2 = (b ? gain : -1);
            return (b ? AT_OK : AT_ERROR);
        case AT_READ_OP:
            if (sprintf(response, "%d", wifiGain2) < 0) {
                LOGATCI(LOG_ERR, "sprintf failed: %d", __LINE__);
                return AT_ERROR;
            }
            return AT_OK;
        default:
            break;
    }
    return AT_ERROR;
}

ATRESPONSE_t pas_witx_handler(char* cmdline, ATOP_t opType, char* response) {
    UNUSED(response);
    int err = 0, cmd = -1;
    bool b = false;

    switch (opType) {
        case AT_SET_OP:
            err = at_tok_nextint(&cmdline, &cmd);
            if (err < 0) return -1;
#ifdef MTK_WLAN_FEATURE
            b = WIFI_TEST_SetTX(cmd == 0 ? false : true);
#endif
            if (b)
                return AT_OK;
            else
                return AT_ERROR;
        default:
            break;
    }
    return AT_ERROR;
}

ATRESPONSE_t pas_wirx_handler(char* cmdline, ATOP_t opType, char* response) {
    UNUSED(response);
    int err = 0, cmd = -1;
    bool b = false;
    char srcAddr[MAC_ADDR_SIZE] = {0};
    char dstAddr[MAC_ADDR_SIZE] = {0};
    int64_t tmp = 0;

    switch (opType) {
        case AT_SET_OP:
            err = at_tok_nextint(&cmdline, &cmd);
            if (err < 0) return -1;
            switch (cmd) {
                case 0:
#ifdef MTK_WLAN_FEATURE
                    b = WIFI_TEST_SetRX(false, srcAddr, dstAddr);
#endif
                    break;
                case 1:
                    err = at_tok_nextint64(&cmdline, &tmp);
                    memcpy(srcAddr, (char *)(&tmp), MAC_ADDR_SIZE);
                    if (err < 0) return -1;
                    err = at_tok_nextint64(&cmdline, &tmp);
                    memcpy(dstAddr, (char *)(&tmp), MAC_ADDR_SIZE);
                    if (err < 0) return -1;
#ifdef MTK_WLAN_FEATURE
                    b = WIFI_TEST_SetRX(true, srcAddr, dstAddr);
#endif
                    break;
            }
        default:
            break;
    }
    if(b)
        return AT_OK;
    else
        return AT_ERROR;
}

ATRESPONSE_t pas_wirpckg_handler(char* cmdline, ATOP_t opType, char* response) {
    UNUSED(cmdline);
    bool b = false;
    uint32_t pu4GoodFrameCount = 0;
    uint32_t pu4BadFrameCount = 0;

    switch (opType) {
        case AT_SET_OP:
#ifdef MTK_WLAN_FEATURE
            b = WIFI_TEST_ClearResult();
#endif
            if (b) {
                return AT_OK;
            } else {
                return AT_ERROR;
            }
        case AT_READ_OP:
#ifdef MTK_WLAN_FEATURE
            b = WIFI_TEST_GetResult(&pu4GoodFrameCount, &pu4BadFrameCount);
#endif
            if (sprintf(response, "%d,%d OK",
                    pu4GoodFrameCount, pu4BadFrameCount) < 0) {
                LOGATCI(LOG_ERR, "sprintf failed: %d", __LINE__);
                return AT_ERROR;
            }
            if (b)
                return AT_OK;
            else
                return AT_ERROR;
        default:
            break;
    }
    return AT_ERROR;
}

ATRESPONSE_t pas_wirssi_handler(char* cmdline, ATOP_t opType, char* response) {
    UNUSED(cmdline);
    int rssi = -127, rxok = -1, rxerror = -1;
    bool b = false;

    switch (opType) {
        case AT_READ_OP:
#ifdef MTK_WLAN_FEATURE
            WIFI_TEST_FRGood(&rxok);
            WIFI_TEST_FRError(&rxerror);
            LOGATCI(LOG_DEBUG, "RX ok:%d, error:%d", rxok, rxerror);
            b = WIFI_TEST_RSSI(&rssi);
            LOGATCI(LOG_DEBUG, "WIFI_TEST_RSSI got retrun: %s", b ? "true" : "false");
#endif
            if (sprintf(response, "Current RSSI:%d, RX OK/ERR = %d/%d",
                    rssi, rxok, rxerror) < 0) {
                LOGATCI(LOG_ERR, "sprintf failed: %d", __LINE__);
                return AT_ERROR;
            }
            return (b ? AT_OK : AT_ERROR);
        default:
            break;
    }
    return AT_ERROR;
}

ATRESPONSE_t pas_wigi_handler(char* cmdline, ATOP_t opType, char* response) {
    int err = 0, guardInterval = 0;
    bool b = false;

    switch (opType) {
        case AT_SET_OP:
            err = at_tok_nextint(&cmdline, &guardInterval);
            if (err < 0) return -1;
#ifdef MTK_WLAN_FEATURE
            b = WIFI_TEST_GI(guardInterval);
            if (sprintf(response, "Set guard interval:%d", guardInterval) < 0) {
                LOGATCI(LOG_ERR, "sprintf failed: %d", __LINE__);
                return AT_ERROR;
            }
#endif
            return (b ? AT_OK : AT_ERROR);
        default:
            break;
    }
    return AT_ERROR;
}


ATRESPONSE_t pas_wipreamble_handler(char* cmdline, ATOP_t opType, char* response) {
    int err = 0, preamble = 0;
    bool b = false;

    switch (opType) {
        case AT_SET_OP:
            err = at_tok_nextint(&cmdline, &preamble);
            if (err < 0) return -1;
#ifdef MTK_WLAN_FEATURE
            b = WIFI_TEST_SetPreamble(preamble);
            if (sprintf(response, "Set preamble:%d", preamble) < 0) {
                LOGATCI(LOG_ERR, "sprintf failed: %d", __LINE__);
                return AT_ERROR;
            }
#endif
            return (b ? AT_OK : AT_ERROR);
        default:
            break;
    }
    return AT_ERROR;
}

ATRESPONSE_t pas_witxpktlen_handler(char* cmdline, ATOP_t opType, char* response) {
    int err = 0, txPacketLength = 0;
    bool b = false;

    switch (opType) {
        case AT_SET_OP:
            err = at_tok_nextint(&cmdline, &txPacketLength);
            if (err < 0) return -1;
#ifdef MTK_WLAN_FEATURE
            b = WIFI_TEST_TxPayloadLength(txPacketLength);
#endif
            wifiTxPacketLength = (b ? txPacketLength : -1);
            return (b ? AT_OK : AT_ERROR);
        case AT_READ_OP:
            if (sprintf(response, "%d", wifiTxPacketLength) < 0) {
                LOGATCI(LOG_ERR, "sprintf failed: %d", __LINE__);
                return AT_ERROR;
            }
            return AT_OK;
        default:
            break;
    }
    return AT_ERROR;
}

ATRESPONSE_t pas_witxpktcnt_handler(char* cmdline, ATOP_t opType, char* response) {
    int err = 0, txPacketCount = 0;
    bool b = false;

    switch (opType) {
        case AT_SET_OP:
            err = at_tok_nextint(&cmdline, &txPacketCount);
            if (err < 0) return -1;
#ifdef MTK_WLAN_FEATURE
            b = WIFI_TEST_TxBurstFrames(txPacketCount);
#endif
            wifiTxPacketCount = (b ? txPacketCount : -1);
            return (b ? AT_OK : AT_ERROR);
        case AT_READ_OP:
            if (sprintf(response, "%d", wifiTxPacketCount) < 0) {
                LOGATCI(LOG_ERR, "sprintf failed: %d", __LINE__);
                return AT_ERROR;
            }
            return AT_OK;
        default:
            break;
    }
    return AT_ERROR;
}

ATRESPONSE_t pas_witxpktinterval_handler(char* cmdline, ATOP_t opType, char* response) {
    int err = 0, txPacketInterval = 0;
    bool b = false;

    switch (opType) {
        case AT_SET_OP:
            err = at_tok_nextint(&cmdline, &txPacketInterval);
            if (err < 0) return -1;
#ifdef MTK_WLAN_FEATURE
            b = WIFI_TEST_TxBurstInterval(txPacketInterval);
#endif
            wifiTxPacketinterval = (b ? txPacketInterval : -1);
            return (b ? AT_OK : AT_ERROR);
        case AT_READ_OP:
            if (sprintf(response, "%d", wifiTxPacketinterval) < 0) {
                LOGATCI(LOG_ERR, "sprintf failed: %d", __LINE__);
                return AT_ERROR;
            }
            return AT_OK;
        default:
            break;
    }
    return AT_ERROR;
}

ATRESPONSE_t pas_wichbandwidth_handler(char* cmdline, ATOP_t opType, char* response) {
    int err = 0, cmdID = 0;
    bool b = false;

    switch (opType) {
        case AT_SET_OP:
            err = at_tok_nextint(&cmdline, &cmdID);
            if (err < 0) return -1;
            switch (cmdID) {
                case 0:
#ifdef MTK_WLAN_FEATURE
                    b = WIFI_TEST_SetBandwidthV2(WIFI_TEST_CH_BW_20MHZ);
#endif
                    if (b) {
                        if (sprintf(response,
                                "Set channel bandwidth: 20MHZ") < 0) {
                            LOGATCI(LOG_ERR, "sprintf failed: %d", __LINE__);
                            return AT_ERROR;
                        }
                        wifiChannelBandwidth = 0;
                    }
                    break;
                case 1:
#ifdef MTK_WLAN_FEATURE
                    b = WIFI_TEST_SetBandwidthV2(WIFI_TEST_CH_BW_40MHZ);
#endif
                    if (b) {
                        if (sprintf(response,
                                "Set channel bandwidth: 40MHZ") < 0) {
                            LOGATCI(LOG_ERR, "sprintf failed: %d", __LINE__);
                            return AT_ERROR;
                        }
                        wifiChannelBandwidth = 1;
                    }
                    break;
                case 2:
#ifdef MTK_WLAN_FEATURE
                    b = WIFI_TEST_SetBandwidthV2(WIFI_TEST_CH_BW_80MHZ);
#endif
                    if (b) {
                        if (sprintf(response,
                                "Set channel bandwidth: 80MHZ") < 0) {
                            LOGATCI(LOG_ERR, "sprintf failed: %d", __LINE__);
                            return AT_ERROR;
                        }
                        wifiChannelBandwidth = 2;
                    }
                    break;
                case 3:
#ifdef MTK_WLAN_FEATURE
                    b = WIFI_TEST_SetBandwidthV2(WIFI_TEST_CH_BW_160MHZ);
#endif
                    if (b) {
                        if (sprintf(response,
                                "Set channel bandwidth: 160MHZ") < 0) {
                            LOGATCI(LOG_ERR, "sprintf failed: %d", __LINE__);
                            return AT_ERROR;
                        }
                        wifiChannelBandwidth = 3;
                    }
                    break;
            }
            return (b ? AT_OK : AT_ERROR);
        case AT_READ_OP:
            if (wifiChannelBandwidth >= 0 && wifiChannelBandwidth < WIFI_TEST_CH_BW_NUM) {
                if (sprintf(response, "Channel bandwidth: %d",
                        wifiChannelBandwidth) < 0) {
                    LOGATCI(LOG_ERR, "sprintf failed: %d", __LINE__);
                    return AT_ERROR;
                }
            } else {
                if (sprintf(response, "No define for channel band") < 0) {
                    LOGATCI(LOG_ERR, "sprintf failed: %d", __LINE__);
                    return AT_ERROR;
                }
                return AT_OK;
            }
            break;
        default:
            break;
    }
    return AT_ERROR;
}

ATRESPONSE_t pas_widatabandwidth_handler(char* cmdline, ATOP_t opType, char* response) {
    int err = 0, cmdID = 0;
    bool b = false;

    switch (opType) {
        case AT_SET_OP:
            err = at_tok_nextint(&cmdline, &cmdID);
            if (err < 0) return -1;
            switch (cmdID) {
                case 0:
#ifdef MTK_WLAN_FEATURE
                    b = WIFI_TEST_SetTxBandwidth(WIFI_TEST_CH_BW_20MHZ);
#endif
                    if (b) {
                        if (sprintf(response, "Set data bandwidth: 20MHZ") < 0) {
                            LOGATCI(LOG_ERR, "sprintf failed: %d", __LINE__);
                            return AT_ERROR;
                        }
                        wifiDataBandwidth= 0;
                    }
                    break;
                case 1:
#ifdef MTK_WLAN_FEATURE
                    b = WIFI_TEST_SetTxBandwidth(WIFI_TEST_CH_BW_40MHZ);
#endif
                    if (b) {
                        if (sprintf(response,
                                "Set data bandwidth: 40MHZ") < 0) {
                            LOGATCI(LOG_ERR, "sprintf failed: %d", __LINE__);
                            return AT_ERROR;
                        }
                        wifiDataBandwidth = 1;
                    }
                    break;
                case 2:
#ifdef MTK_WLAN_FEATURE
                    b = WIFI_TEST_SetTxBandwidth(WIFI_TEST_CH_BW_80MHZ);
#endif
                    if (b) {
                        if (sprintf(response,
                                "Set data bandwidth: 80MHZ") < 0) {
                            LOGATCI(LOG_ERR, "sprintf failed: %d", __LINE__);
                            return AT_ERROR;
                        }
                        wifiDataBandwidth = 2;
                    }
                    break;
                case 3:
#ifdef MTK_WLAN_FEATURE
                    b = WIFI_TEST_SetTxBandwidth(WIFI_TEST_CH_BW_160MHZ);
#endif
                    if (b) {
                        if (sprintf(response,
                                "Set data bandwidth: 160MHZ") < 0) {
                            LOGATCI(LOG_ERR, "sprintf failed: %d", __LINE__);
                            return AT_ERROR;
                        }
                        wifiDataBandwidth = 3;
                    }
                    break;
            }
            return (b ? AT_OK : AT_ERROR);
        case AT_READ_OP:
            if (wifiDataBandwidth >= 0 && wifiDataBandwidth < WIFI_TEST_CH_BW_NUM) {
                if (sprintf(response, "Data bandwidth: %d",
                        wifiDataBandwidth) < 0) {
                    LOGATCI(LOG_ERR, "sprintf failed: %d", __LINE__);
                    return AT_ERROR;
                }
            } else {
                if (sprintf(response, "No define for data bandwidth") < 0) {
                    LOGATCI(LOG_ERR, "sprintf failed: %d", __LINE__);
                    return AT_ERROR;
                }
                return AT_OK;
            }
            break;
        default:
            break;
    }
    return AT_ERROR;
}


ATRESPONSE_t pas_wiprimarychset_handler(char* cmdline, ATOP_t opType, char* response) {
    int err = 0, primaryChannelOffset = 0;
    bool b = false;

    switch (opType) {
        case AT_SET_OP:
            err = at_tok_nextint(&cmdline, &primaryChannelOffset);
            if (err < 0) return -1;
#ifdef MTK_WLAN_FEATURE
            b = WIFI_TEST_SetPriChannelSetting(primaryChannelOffset);
#endif
            wifiPrimaryChannelOffset = (b ? primaryChannelOffset : -1);
            return (b ? AT_OK : AT_ERROR);
        case AT_READ_OP:
            if (sprintf(response, "%d", wifiPrimaryChannelOffset) < 0) {
                LOGATCI(LOG_ERR, "sprintf failed: %d", __LINE__);
                return AT_ERROR;
            }
            return AT_OK;
        default:
            break;
    }
    return AT_ERROR;
}

ATRESPONSE_t pas_witxdatarate_handler(char* cmdline, ATOP_t opType, char* response) {
    int err = 0, txDataRate = 0;
    bool b = false;

    switch (opType) {
        case AT_SET_OP:
            err = at_tok_nextint(&cmdline, &txDataRate);
            if (err < 0) return -1;
#ifdef MTK_WLAN_FEATURE
            b = WIFI_TEST_TxDataRate(txDataRate);
#endif
            wifiTxDataRate = (b ? txDataRate : -1);
            return (b ? AT_OK : AT_ERROR);
        case AT_READ_OP:
            if (sprintf(response, "%d", wifiTxDataRate) < 0) {
                LOGATCI(LOG_ERR, "sprintf failed: %d", __LINE__);
                return AT_ERROR;
            }
            return AT_OK;
        default:
            break;
    }
    return AT_ERROR;
}

ATRESPONSE_t pas_wirxstart_handler(char* cmdline, ATOP_t opType, char* response) {
    UNUSED(response);
    int err = 0, cmd = -1;
    bool b = false;

    switch (opType) {
        case AT_SET_OP:
            err = at_tok_nextint(&cmdline, &cmd);
            if (err < 0) return -1;
#ifdef MTK_WLAN_FEATURE
            b = (cmd == 0 ? WIFI_TEST_RxStop() : WIFI_TEST_RxStart());
#endif
            return (b ? AT_OK : AT_ERROR);
        default:
            break;
    }
    return AT_ERROR;
}

ATRESPONSE_t pas_witestset_handler(char* cmdline, ATOP_t opType, char* response) {
    int err = 0, index = -1, data = -1;
#ifdef MTK_WLAN_FEATURE
    int operation = -1;
#endif

    switch (opType) {
        case AT_SET_OP:
            err = at_tok_nextint(&cmdline, &index);
            if (err < 0) {
                return AT_ERROR;
            }

            err = at_tok_nextint(&cmdline, &data);
            if (err < 0) {
                return AT_ERROR;
            }
#ifdef MTK_WLAN_FEATURE
            operation = WIFI_TEST_set(index, data, NULL, NULL);
            if (sprintf(response, "Set index:%d to %d %s", index, data,
                        (operation == 0 ? "successful" : "failed")) < 0) {
                LOGATCI(LOG_ERR, "sprintf failed: %d", __LINE__);
                return AT_ERROR;
            }
            return (operation == 0 ? AT_OK : AT_ERROR);
#else
            LOGATCI(LOG_DEBUG,
                "AT_SET_OP failed - MTK_WLAN_FEATURE is disabled.\n");
            if (sprintf(response, "Set index:%d to %d failed", index, data) < 0)
                LOGATCI(LOG_ERR, "sprintf failed: %d", __LINE__);
            return AT_ERROR;
#endif
        case AT_READ_OP:
            err = at_tok_nextint(&cmdline, &index);
            if (err < 0) {
                return AT_ERROR;
            }
#ifdef MTK_WLAN_FEATURE
            operation = WIFI_TEST_get(index, 0, NULL, (uint32_t *)&data);
            if (sprintf(response, "Get index:%d --> data:%d %s", index, data,
                    (operation == 0 ? "successful" : "failed")) < 0) {
                LOGATCI(LOG_ERR, "sprintf failed: %d", __LINE__);
                return AT_ERROR;
            }
            return (operation == 0 ? AT_OK : AT_ERROR);
#else
            LOGATCI(LOG_DEBUG,
                "AT_READ_OP failed - MTK_WLAN_FEATURE is disabled.\n");
            if (sprintf(response, "Get index:%d --> data:%d failed",
                    index, data) < 0)
                LOGATCI(LOG_ERR, "sprintf failed: %d", __LINE__);
            return AT_ERROR;
#endif
        default:
            break;
    }
    return AT_ERROR;
}

#ifndef MTK_TC1_FEATURE
ATRESPONSE_t pas_witestmode_handler(char* cmdline, ATOP_t opType, char* response) {
    UNUSED(cmdline);
    UNUSED(opType);
    UNUSED(response);
    return AT_ERROR;
}

ATRESPONSE_t pas_witx2_handler(char* cmdline, ATOP_t opType, char* response) {
    UNUSED(cmdline);
    UNUSED(opType);
    UNUSED(response);
    return AT_ERROR;
}

ATRESPONSE_t pas_wirx2_handler(char* cmdline, ATOP_t opType, char* response) {
    UNUSED(cmdline);
    UNUSED(opType);
    UNUSED(response);
    return AT_ERROR;
}

ATRESPONSE_t pas_wimac_handler(char* cmdline, ATOP_t opType, char* response) {
    UNUSED(cmdline);
    UNUSED(opType);
    UNUSED(response);
    return AT_ERROR;
}

ATRESPONSE_t pas_wimacck_handler(char* cmdline, ATOP_t opType, char* response) {
    UNUSED(cmdline);
    UNUSED(opType);
    UNUSED(response);
    return AT_ERROR;
}
#endif
