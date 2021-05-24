/*
 *   @file  mss_main.c
 *
 *   @brief
 *      MRR TI Design  executing on the MSS
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2020 Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* BIOS/XDC Include Files. */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/family/arm/v7a/Pmu.h>
#include <ti/sysbios/family/arm/v7r/vim/Hwi.h>

/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/crc/crc.h>
#include <ti/drivers/uart/UART.h>
#include <ti/drivers/gpio/gpio.h>
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/drivers/pinmux/include/pinmux_xwr18xx.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/control/mmwavelink/mmwavelink.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/control/mmwavelink/include/rl_driver.h>

#include <ti/drivers/canfd/canfd.h>
#include <ti/drivers/pinmux/pinmux.h>
#include <ti/utils/testlogger/logger.h>

/* Application Include Files: */
#include "mss_mrr.h"
#include "../common/mrr_config_consts.h"
#include "../common/mmw_messages.h"
#include <ti/utils/cli/cli.h>
#include <ti/drivers/cbuff/cbuff.h>

/**************************************************************************
 *************************** Global Definitions ***************************
 **************************************************************************/
uint8_t                 rxData[64U];
uint32_t                txDataLength, rxDataLength;



/**
 * @brief
 *  Global Variable for tracking information required by the MRR TI Design
 */
#pragma DATA_ALIGN(gMrrMSSMCB, 16);
Mrr_MSS_MCB    gMrrMSSMCB;

/**************************************************************************
 *************************** Local Definitions ****************************
 **************************************************************************/
static void MRR_MSS_chirpIntCallback(uintptr_t arg);
static void MRR_MSS_frameStartIntCallback(uintptr_t arg);
static int32_t MRR_MSS_eventFxn(uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload);
static void MRR_MSS_cfgFxn(MMWave_CtrlCfg* ptrCtrlCfg);
static void MRR_MSS_startFxn(MMWave_CalibrationCfg* ptrCalibrationCfg);
static void MRR_MSS_stopFxn(void);
static void MRR_MSS_initTask (UArg arg0, UArg arg1);
static void MRR_MSS_mmWaveCtrlTask (UArg arg0, UArg arg1);

/**************************************************************************
 ********************** MSS MRR TI Design Functions ***********************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      This is the callback function registered with the ADC Driver which is invoked
 *      when a chirp is available. This is executed in the ISR context.
 *      
 *  @param[in]  arg
 *      Application registered argument
 *
 *  @retval
 *      Not Applicable.
 */
static void MRR_MSS_chirpIntCallback(uintptr_t arg)
{
    gMrrMSSMCB.chirpInt++;
    gMrrMSSMCB.chirpIntcumSum++;
    
    /* The different subframes in the MRR configuration have different chirp sizes. 
     * These need to be configured before the start of the next subframe, at the 
     * end of the previous subframes last chirp. */
    if (gMrrMSSMCB.chirpInt == gMrrMSSMCB.numChirpsPerSubframe[gMrrMSSMCB.subframeId]) 
    {
        gMrrMSSMCB.subframeCntFromChirpInt++;
        gMrrMSSMCB.subframeId = gMrrMSSMCB.subframeId + 1;
        if (gMrrMSSMCB.subframeId == NUM_SUBFRAMES)
        {
            gMrrMSSMCB.subframeId = 0;
        }
        gMrrMSSMCB.chirpInt = 0; 
    }
}

/**
 *  @b Description
 *  @n
 *      This is the callback function registered with the ADC Driver which is invoked
 *      when a frame is started. This is executed in the ISR context.
 *
 *  @param[in]  arg
 *      Application registered argument
 *
 *  @retval
 *      Not Applicable.
 */
static void MRR_MSS_frameStartIntCallback(uintptr_t arg)
{
    gMrrMSSMCB.frameStartToken++;
    gMrrMSSMCB.subframeCntFromFrameStart++;
    /* Check if the frames are coming correctly, and no chirps have been missed.         */
    if (gMrrMSSMCB.frameStartToken == 1) 
    {
        if (gMrrMSSMCB.chirpInt == 0)
        {
            gMrrMSSMCB.frameStartToken = 0;
        }
        else
        {
            DebugP_assert (0);
        }
    }
}

/**
 *  @b Description
 *  @n
 *      Registered event function which is invoked when an event from the
 *      BSS is received.
 *
 *  @param[in]  msgId
 *      Message Identifier
 *  @param[in]  sbId
 *      Subblock identifier
 *  @param[in]  sbLen
 *      Length of the subblock
 *  @param[in]  payload
 *      Pointer to the payload buffer
 *
 *  @retval
 *      Always returns 0 [Continue passing the event to the peer domain]
 */
static int32_t MRR_MSS_eventFxn(uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload)
{
    uint16_t asyncSB = RL_GET_SBID_FROM_UNIQ_SBID(sbId);
    // System_printf("%d,%d\r\n",msgId,asyncSB);
    /* Process the received message: */
    switch (msgId)
    {
        case RL_RF_ASYNC_EVENT_MSG:
        {
            /* Received Asychronous Message: */
            switch (asyncSB)
            {
                case RL_RF_AE_CPUFAULT_SB:
                {
                    /* Post event to datapath task notify BSS events */
                    //Event_post(gMrrMSSMCB.eventHandle, MMWDEMO_BSS_CPUFAULT_EVT);
                    break;
                }
                case RL_RF_AE_ESMFAULT_SB:
                {
                    /* Post event to datapath task notify BSS events */
                    //Event_post(gMrrMSSMCB.eventHandle, MMWDEMO_BSS_ESMFAULT_EVT);
                    break;
                }
                case RL_RF_AE_INITCALIBSTATUS_SB:
                {
                    /* This event should be handled by mmwave internally, ignore the event here */
                    break;
                }
                case RL_RF_AE_FRAME_TRIGGER_RDY_SB:
                {
                    break;
                }
                case RL_RF_AE_MON_TIMING_FAIL_REPORT_SB:
                {
                    break;
                }
                case RL_RF_AE_RUN_TIME_CALIB_REPORT_SB:
                {
                    break;
                }
                default:
                {
                    System_printf ("Error: Asynchronous Event SB Id %d not handled\n", asyncSB);
                    break;
                }
            }
            break;
        }
        default:
        {
           System_printf ("Error: Asynchronous message %d is NOT handled\n", msgId);
             break;
        }
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the application registered callback function which is invoked after
 *      the configuration has been used to configure the mmWave link and the BSS.
 *
 *  @param[in]  ptrCtrlCfg
 *      Pointer to the control configuration
 *
 *  @retval
 *      Not applicable
 */
static void MRR_MSS_cfgFxn(MMWave_CtrlCfg* ptrCtrlCfg)
{
    /* The MRR TI Design operates in MINIMAL mode and so the configuration 
     * callback function is never invoked. The assertion will ensure that 
     * this is never invoked */
    DebugP_assert (0);
}

/**
 *  @b Description
 *  @n
 *      Application registered callback function which is invoked the mmWave link on BSS
 *      has been started. This is applicable only for the XWR16xx. The BSS can be configured
 *      only by the MSS *or* DSS. The callback API is triggered on the remote execution
 *      domain (which did not configure the BSS)
 *
 *  @param[in]  ptrCalibrationCfg
 *      Pointer to the calibrat    ion configuration
 *
 *  @retval
 *      Not applicable
 */
static void MRR_MSS_startFxn(MMWave_CalibrationCfg* ptrCalibrationCfg)
{
    /* Post an event to main data path task. 
       This function in only called when mmwave_start() is called on DSS */
    gMrrMSSMCB.stats.datapathStartEvt ++;

    /* Post event to start is done */
    Event_post(gMrrMSSMCB.eventHandleNotify, MMWDEMO_DSS_START_COMPLETED_EVT);
}

/**
 *  @b Description
 *  @n
 *      This is the application registered callback function which is invoked after the
 *      has been stopped. This is applicable only for the XWR16xx. The BSS can be configured
 *      only by the MSS *or* DSS. The callback API is triggered on the remote execution
 *      mmWave link on BSS has been stopped.
 *
 *  @retval
 *      Not applicable
 */
static void MRR_MSS_stopFxn(void)
{
      /* Possible sceanarios:
       1. CLI sensorStop command triggers mmwave_stop() to be called from MSS 
       2. In case of Error, mmwave_stop() will be triggered either from MSS or DSS
     */
    gMrrMSSMCB.stats.datapathStopEvt ++;
}

/**
 *  @b Description
 *  @n
 *      Application registered callback function which is invoked the mmWave link on BSS
 *      has been opened.
 *
 *  @param[in]  ptrOpenCfg
 *      Pointer to the open configuration
 *
 *  @retval
 *      Not applicable
 */
static void MRR_MSS_openFxn(MMWave_OpenCfg* ptrOpenCfg)
{
    return;
}


/**
 *  @b Description
 *  @n
 *      Application registered callback function which is invoked the mmWave link on BSS
 *      has been closed.
 *
 *  @retval
 *      Not applicable
 */
static void MRR_MSS_closeFxn(void)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      Function to send a message to peer through Mailbox virtural channel 
 *
 *  @param[in]  message
 *      Pointer to the MMW demo message.  
 *
 *  @retval
 *      Success    - 0
 *      Fail       < -1 
 */
int32_t MmwDemo_mboxWrite(MmwDemo_message     * message)
{
    int32_t                  retVal = -1;
    
    retVal = Mailbox_write (gMrrMSSMCB.peerMailbox, (uint8_t*)message, sizeof(MmwDemo_message));
    if (retVal == sizeof(MmwDemo_message))
    {
        retVal = 0;
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The Task is used to handle  the mmw demo messages received from 
 *      Mailbox virtual channel.  
 *
 *  @param[in]  arg0
 *      arg0 of the Task. Not used
 *  @param[in]  arg1
 *      arg1 of the Task. Not used
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_mboxReadTask(UArg arg0, UArg arg1)
{
    MmwDemo_message      message;
    int32_t              retVal = 0;
    
    /* wait for new message and process all the messsages received from the peer */
    while(1)
    {
        Semaphore_pend(gMrrMSSMCB.mboxSemHandle, BIOS_WAIT_FOREVER);
        
        /* Read the message from the peer mailbox: We are not trying to protect the read
         * from the peer mailbox because this is only being invoked from a single thread */
        retVal = Mailbox_read(gMrrMSSMCB.peerMailbox, (uint8_t*)&message, sizeof(MmwDemo_message));
        if (retVal < 0)
        {
            /* Error: Unable to read the message. Setup the error code and return values */
            System_printf ("Error: Mailbox read failed [Error code %d]\n", retVal);
        }
        else if (retVal == 0)
        {
            /* We are done: There are no messages available from the peer execution domain. */
            continue;
        }
        else
        {
            /* Flush out the contents of the mailbox to indicate that we are done with the message. This will
             * allow us to receive another message in the mailbox while we process the received message. */
            Mailbox_readFlush (gMrrMSSMCB.peerMailbox);

            /* Process the received message: */
            switch (message.type)
            {
                case MMWDEMO_DSS2MSS_DETOBJ_READY:
                    /* Got detected objects , to be shipped out shipped out through UART */
                    {
                        uint32_t totalPacketLen;
                        uint32_t numPaddingBytes;
                        uint32_t itemIdx;

                        /* Send header */
                        totalPacketLen = sizeof(MmwDemo_output_message_header);
                        UART_writePolling (gMrrMSSMCB.loggingUartHandle,
                                           (uint8_t*)&message.body.detObj.header,
                                           sizeof(MmwDemo_output_message_header));
                        /* Send TLVs */
                        for (itemIdx = 0;  itemIdx < message.body.detObj.header.numTLVs; itemIdx++)
                        {
                            UART_writePolling (gMrrMSSMCB.loggingUartHandle,
                                               (uint8_t*)&message.body.detObj.tlv[itemIdx],
                                               sizeof(MmwDemo_output_message_tl));
                            UART_writePolling (gMrrMSSMCB.loggingUartHandle,
                                               (uint8_t*)SOC_translateAddress(message.body.detObj.tlv[itemIdx].address, SOC_TranslateAddr_Dir_FROM_OTHER_CPU,NULL),
                                               message.body.detObj.tlv[itemIdx].length);
                            totalPacketLen += sizeof(MmwDemo_output_message_tl) + message.body.detObj.tlv[itemIdx].length;
                        }

                        /* Send padding to make total packet length multiple of MMWDEMO_OUTPUT_MSG_SEGMENT_LEN */
                        numPaddingBytes = MMWDEMO_OUTPUT_MSG_SEGMENT_LEN - (totalPacketLen & (MMWDEMO_OUTPUT_MSG_SEGMENT_LEN-1));
                        if (numPaddingBytes<MMWDEMO_OUTPUT_MSG_SEGMENT_LEN)
                        {
                            uint8_t padding[MMWDEMO_OUTPUT_MSG_SEGMENT_LEN];
                            /*DEBUG:*/ memset(&padding, 0xf, MMWDEMO_OUTPUT_MSG_SEGMENT_LEN);
                            UART_writePolling (gMrrMSSMCB.loggingUartHandle, padding, numPaddingBytes);
                        }
                    }
                    /* Send a message to MSS to log the output data */
                    memset((void *)&message, 0, sizeof(MmwDemo_message));

                    message.type = MMWDEMO_MSS2DSS_DETOBJ_SHIPPED;
                                
                    retVal = MmwDemo_mboxWrite(&message);
                    
                    if (retVal != 0)
                    {
                        System_printf ("Error: Mailbox send message id=%d failed \n", message.type);
                    }

                    break;
                
                case MMWDEMO_DSS2MSS_ASSERT_INFO:
                    /* Send the received DSS assert info through CLI */
                    CLI_write ("DSS Exception: %s, line %d.\n", message.body.assertInfo.file,
                        message.body.assertInfo.line);
                    break;
                default:
                {
                    /* Message not support */
                    System_printf ("Error: unsupport Mailbox message id=%d\n", message.type);
                    break;
                }
            }
        }
    }
}

/**
 *  @b Description
 *  @n
 *      This function is a callback funciton that invoked when a message is received from the peer.
 *
 *  @param[in]  handle
 *      Handle to the Mailbox on which data was received
 *  @param[in]  peer
 *      Peer from which data was received
 
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_mboxCallback
(
    Mbox_Handle  handle,
    Mailbox_Type    peer
)
{
    /* Message has been received from the peer endpoint. Wakeup the mmWave thread to process
     * the received message. */
    Semaphore_post (gMrrMSSMCB.mboxSemHandle);
}


/**
 *  @b Description
 *  @n
 *      This is the task which provides an execution context for
 *      the mmWave control module.
 *
 *  @param[in]  arg0
 *      Argument0 with which the task was created
 *  @param[in]  arg1
 *      Argument1 with which the task was created
 *
 *  @retval
 *      Not Applicable.
 */
static void MRR_MSS_mmWaveCtrlTask (UArg arg0, UArg arg1)
{
    int32_t errCode;

    while (1)
    {

        /* Execute the mmWave control module: */
        if (MMWave_execute (gMrrMSSMCB.ctrlHandle, &errCode) < 0)
        {
            System_printf ("Error: mmWave control execution failed [Error code %d]\n", errCode);
        }
    }
}

/**
 *  @b Description
 *  @n
 *      MSS Initialization Task which initializes the various
 *      components in the MSS subsystem.
 *
 *  @param[in]  arg0
 *      Argument0 with which the task was created
 *  @param[in]  arg1
 *      Argument1 with which the task was created
 *
 *  @retval
 *      Not Applicable.
 */
static void MRR_MSS_initTask (UArg arg0, UArg arg1)
{
    int32_t                 errCode;
    MMWave_InitCfg          initCfg;
    UART_Params             uartParams;
    Task_Params                taskParams;
    SOC_SysIntListenerCfg   listenerCfg;
    Semaphore_Params    semParams;
    Mailbox_Config      mboxCfg;
   /* Debug Message: */
    System_printf("Debug: Launched the Initialization Task\n");

    /*****************************************************************************
     * Initialize the mmWave SDK components:
     *****************************************************************************/
    /* Pinmux setting */

    /* Setup the PINMUX to bring out the UART-1 */
    Pinmux_Set_OverrideCtrl(SOC_XWR18XX_PINN5_PADBE, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);    
    Pinmux_Set_FuncSel(SOC_XWR18XX_PINN5_PADBE, SOC_XWR18XX_PINN5_PADBE_MSS_UARTA_TX);
    Pinmux_Set_OverrideCtrl(SOC_XWR18XX_PINN4_PADBD, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);    
    Pinmux_Set_FuncSel(SOC_XWR18XX_PINN4_PADBD, SOC_XWR18XX_PINN4_PADBD_MSS_UARTA_RX);

    /* Setup the PINMUX to bring out the UART-3 */
    Pinmux_Set_OverrideCtrl(SOC_XWR18XX_PINF14_PADAJ, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR18XX_PINF14_PADAJ, SOC_XWR18XX_PINF14_PADAJ_MSS_UARTB_TX);

    /* Setup the PINMUX to bring out the DSS UART */
    Pinmux_Set_OverrideCtrl(SOC_XWR18XX_PINP8_PADBM, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR18XX_PINP8_PADBM, SOC_XWR18XX_PINP8_PADBM_DSS_UART_TX);

    /* Initialize the UART */
    UART_init();
    
    /* Initialize the GPIO */
    GPIO_init();
    
    /* Initialize the Mailbox */
    Mailbox_init(MAILBOX_TYPE_MSS);

    /* Open UART Driver: */
    UART_Params_init(&uartParams);
    uartParams.clockFrequency = MSS_SYS_VCLK;
    uartParams.baudRate       = 115200;
    uartParams.isPinMuxDone   = 1;

    /* Open the Command UART Instance */
    gMrrMSSMCB.commandUartHandle = UART_open(0, &uartParams);
    if (gMrrMSSMCB.commandUartHandle == NULL)
    {
         System_printf("Error: Unable to open the Command UART Instance\n");
        return;
    }

     /* Setup the default UART Parameters */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.clockFrequency = MSS_SYS_VCLK;
    uartParams.baudRate       = 115200*8;
    uartParams.isPinMuxDone   = 1U;

    /* Open the Logging UART Instance: */
    gMrrMSSMCB.loggingUartHandle = UART_open(1, &uartParams);
    if (gMrrMSSMCB.loggingUartHandle == NULL)
    {
        System_printf("Error: MMWDemoMSS Unable to open the Logging UART Instance\n");
        return;
    }

    /*****************************************************************************
     * Creating communication channel between MSS & DSS
     *****************************************************************************/

    /* Create a binary semaphore which is used to handle mailbox interrupt. */
    Semaphore_Params_init(&semParams);
    semParams.mode             = Semaphore_Mode_BINARY;
    gMrrMSSMCB.mboxSemHandle = Semaphore_create(0, &semParams, NULL);

    /* Setup the default mailbox configuration */
    Mailbox_Config_init(&mboxCfg);

    /* Setup the configuration: */
    mboxCfg.chType       = MAILBOX_CHTYPE_MULTI;
    mboxCfg.chId         = MAILBOX_CH_ID_0;
    mboxCfg.writeMode    = MAILBOX_MODE_BLOCKING;
    mboxCfg.readMode     = MAILBOX_MODE_CALLBACK;
    mboxCfg.readCallback = &MmwDemo_mboxCallback;

    /* Initialization of Mailbox Virtual Channel  */
    gMrrMSSMCB.peerMailbox = Mailbox_open(MAILBOX_TYPE_DSS, &mboxCfg, &errCode);
    if (gMrrMSSMCB.peerMailbox == NULL)
    {
        /* Error: Unable to open the mailbox */
        System_printf("Error: Unable to open the Mailbox to the DSS [Error code %d]\n", errCode);
        return;
    }

    /* Create task to handle mailbox messges */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 16*1024;
    Task_create(MmwDemo_mboxReadTask, &taskParams, NULL);
    
    /* Register Chirp Available Listener */
    memset ((void*)&listenerCfg, 0, sizeof(SOC_SysIntListenerCfg));
    listenerCfg.systemInterrupt   = SOC_XWR18XX_MSS_CHIRP_AVAIL_IRQ;
    listenerCfg.listenerFxn       = MRR_MSS_chirpIntCallback;
    listenerCfg.arg               = 0;
    gMrrMSSMCB.chirpIntHandle = SOC_registerSysIntListener (gMrrMSSMCB.socHandle, &listenerCfg, &errCode);
    if (gMrrMSSMCB.chirpIntHandle == NULL)
    {
        System_printf ("Error: Unable to register the Chirp Available Listener [Error code %d]\n", errCode);
        return;
    }

    /* Register Frame Start Listener */
    memset ((void*)&listenerCfg, 0, sizeof(SOC_SysIntListenerCfg));
    listenerCfg.systemInterrupt   = SOC_XWR18XX_MSS_FRAME_START_INT;
    listenerCfg.listenerFxn       = MRR_MSS_frameStartIntCallback;
    listenerCfg.arg               = 0;
    gMrrMSSMCB.frameStartIntHandle = SOC_registerSysIntListener (gMrrMSSMCB.socHandle, &listenerCfg, &errCode);

    if (gMrrMSSMCB.frameStartIntHandle == NULL)
    {
        System_printf("Error: Unable to register the Frame start Listener [Error code %d]\n", errCode);
        return ;
    }

    /*****************************************************************************
     * Initialize the mmWave module:
     *****************************************************************************/
    memset ((void *)&initCfg, 0, sizeof(MMWave_InitCfg));

    /* Populate the initialization configuration: 
     * The MMWAve is configured in minimal isolation mode. */
    initCfg.domain                      = MMWave_Domain_MSS;
    initCfg.socHandle                   = gMrrMSSMCB.socHandle;
    initCfg.eventFxn                    = &MRR_MSS_eventFxn;
    initCfg.cfgMode                     = MMWave_ConfigurationMode_MINIMAL; 
    initCfg.executionMode               = MMWave_ExecutionMode_ISOLATION;
    initCfg.linkCRCCfg.useCRCDriver     = 1U;
    initCfg.linkCRCCfg.crcChannel       = CRC_Channel_CH1;
    initCfg.cooperativeModeCfg.cfgFxn   = &MRR_MSS_cfgFxn;
    initCfg.cooperativeModeCfg.startFxn = &MRR_MSS_startFxn;
    initCfg.cooperativeModeCfg.stopFxn  = &MRR_MSS_stopFxn;
    initCfg.cooperativeModeCfg.openFxn  = &MRR_MSS_openFxn;
    initCfg.cooperativeModeCfg.closeFxn = &MRR_MSS_closeFxn;
    
    /* Initialize and setup the mmWave Control module */
    gMrrMSSMCB.ctrlHandle = MMWave_init (&initCfg, &errCode);
    if (gMrrMSSMCB.ctrlHandle == NULL)
    {
        /* Error: Unable to initialize the mmWave control module */
        System_printf ("Error: mmWave Control Initialization failed [Error code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: Initialized the mmWave module\n");

    /*****************************************************************************
     * Synchronize the mmWave module:
     *****************************************************************************/
    while (1)
    {
        int32_t syncStatus;

        /* Get the synchronization status: */
        syncStatus = MMWave_sync (gMrrMSSMCB.ctrlHandle, &errCode);
        if (syncStatus < 0)
        {
            /* Error: Unable to synchronize the mmWave control module */
            System_printf ("Error: mmWave Control Synchronization failed [Error code %d]\n", errCode);
            return;
        }
        if (syncStatus == 1)
        {
            /* Synchronization acheived: */
            break;
        }
        /* Sleep and poll again: */
        Task_sleep(1);
    }
    
    System_printf ("Debug: Synchronized the mmWave module\n");

    #ifdef SUBFRAME_CONF_MRR_USRR
        gMrrMSSMCB.numChirpsPerSubframe[0] = SUBFRAME_MRR_NUM_CHIRPS_TOTAL;
        gMrrMSSMCB.numChirpsPerSubframe[1] = SUBFRAME_USRR_NUM_CHIRPS_TOTAL;
    #else 
    #ifdef SUBFRAME_CONF_MRR
        gMrrMSSMCB.numChirpsPerSubframe[0] = SUBFRAME_MRR_NUM_CHIRPS_TOTAL;
    #endif 
    #ifdef SUBFRAME_CONF_USRR
        gMrrMSSMCB.numChirpsPerSubframe[0] = SUBFRAME_USRR_NUM_CHIRPS_TOTAL;
    #endif
    #endif

    /*****************************************************************************
     * Launch the mmWave control execution task
     * - This should have a higher priroity than any other task which uses the
     *   mmWave control API
     *****************************************************************************/
    Task_Params_init(&taskParams);
    taskParams.priority  = 6;
    taskParams.stackSize = 3*1024;
    Task_create(MRR_MSS_mmWaveCtrlTask, &taskParams, NULL);

    /*****************************************************************************
     * Setup the CLI
     *****************************************************************************/
    MRR_MSS_CLIInit ();
}

/**
 *  @b Description
 *  @n
 *      Entry point into the MSS TI Design 
 *
 *  @retval
 *      Not Applicable.
 */
int32_t main (void)
{
    Task_Params        taskParams;
    int32_t         errCode;
    SOC_Cfg         socCfg;

    /* Initialize the ESM: Dont clear errors as TI RTOS does it */
    ESM_init(0U);

    /* Initialize the global variables */
    memset ((void*)&gMrrMSSMCB, 0, sizeof(Mrr_MSS_MCB));

    /* Initialize the SOC configuration: */
    memset ((void *)&socCfg, 0, sizeof(SOC_Cfg));

    /* Populate the SOC configuration: */
    socCfg.clockCfg = SOC_SysClock_INIT;

    /* Initialize the SOC Module: This is done as soon as the application is started
     * to ensure that the MPU is correctly configured. */
    gMrrMSSMCB.socHandle = SOC_init (&socCfg, &errCode);
    if (gMrrMSSMCB.socHandle == NULL)
    {
        System_printf ("Error: SOC Module Initialization failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Check if the SOC is a secure device */
    if (SOC_isSecureDevice(gMrrMSSMCB.socHandle, &errCode))
    {
        /* Disable firewall for JTAG and LOGGER (UART) which is needed by the demo */
        SOC_controlSecureFirewall(gMrrMSSMCB.socHandle,
                                  (uint32_t)(SOC_SECURE_FIREWALL_JTAG | SOC_SECURE_FIREWALL_LOGGER),
                                  SOC_SECURE_FIREWALL_DISABLE,
                                  &errCode);
    }

    /* Initialize the Task Parameters. */
    Task_Params_init(&taskParams);
    taskParams.priority = 3;
    Task_create(MRR_MSS_initTask, &taskParams, NULL);

    /* Start BIOS */
    BIOS_start();
    return 0;
}
