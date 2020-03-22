/* Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "board.h"
#include "fsl_shell.h"
#include "fsl_debug_console.h"
#include "fsl_dmamux.h"
#include "fsl_sai_edma.h"
#include "fsl_wm8960.h"
#include "ff.h"
#include "diskio.h"
#include "fsl_sd_disk.h"

#include "pin_mux.h"
#include "clock_config.h"

#include "mainModule.hpp"
#include "pmdmini.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define LED_NUMBERS 1U
#define LED_1_INIT() USER_LED_INIT(LOGIC_LED_OFF)
#define LED_1_ON() USER_LED_ON()
#define LED_1_OFF() USER_LED_OFF()
#define SHELL_Printf PRINTF

/* SAI instance and clock */
#define DEMO_CODEC_WM8960
#define DEMO_SAI SAI1
#define DEMO_SAI_CHANNEL (0)
#define DEMO_SAI_BITWIDTH (kSAI_WordWidth16bits)
#define DEMO_SAI_IRQ SAI1_IRQn
#define SAI_TxIRQHandler SAI1_IRQHandler

/* IRQ */
#define DEMO_SAI_TX_IRQ SAI1_IRQn
#define DEMO_SAI_RX_IRQ SAI1_IRQn

/* DMA */
#define EXAMPLE_DMA DMA0
#define EXAMPLE_DMAMUX DMAMUX
#define EXAMPLE_TX_CHANNEL (0U)
#define EXAMPLE_RX_CHANNEL (1U)
#define EXAMPLE_SAI_TX_SOURCE kDmaRequestMuxSai1Tx
#define EXAMPLE_SAI_RX_SOURCE kDmaRequestMuxSai1Rx

/* Select Audio/Video PLL (786.48 MHz) as sai1 clock source */
#define DEMO_SAI1_CLOCK_SOURCE_SELECT (2U)
/* Clock pre divider for sai1 clock source */
#define DEMO_SAI1_CLOCK_SOURCE_PRE_DIVIDER (1U)
/* Clock divider for sai1 clock source */
#define DEMO_SAI1_CLOCK_SOURCE_DIVIDER (31U)
/* Get frequency of sai1 clock */
#define DEMO_SAI_CLK_FREQ (CLOCK_GetFreq(kCLOCK_AudioPllClk) / (DEMO_SAI1_CLOCK_SOURCE_DIVIDER + 1U) / (DEMO_SAI1_CLOCK_SOURCE_PRE_DIVIDER + 1U))

/* I2C instance and clock */
#define DEMO_I2C LPI2C1

/* Select USB1 PLL (480 MHz) as master lpi2c clock source */
#define DEMO_LPI2C_CLOCK_SOURCE_SELECT (0U)
/* Clock divider for master lpi2c clock source */
#define DEMO_LPI2C_CLOCK_SOURCE_DIVIDER (5U)
/* Get frequency of lpi2c clock */
#define DEMO_I2C_CLK_FREQ ((CLOCK_GetFreq(kCLOCK_Usb1PllClk) / 8) / (DEMO_LPI2C_CLOCK_SOURCE_DIVIDER + 1U))

#define OVER_SAMPLE_RATE (384U)
#define SAMPLE_RATE (kSAI_SampleRate32KHz)
#define BUFFER_SIZE (512)
#define BUFFER_NUM (4)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void Led_Init(void);

/* SHELL user send data callback */
void SHELL_SendDataCallback(uint8_t *buf, uint32_t len);

/* SHELL user receive data callback */
void SHELL_ReceiveDataCallback(uint8_t *buf, uint32_t len);

static int32_t LedControl(p_shell_context_t context, int32_t argc, char **argv);
static int32_t PmdCmd(p_shell_context_t context, int32_t argc, char **argv);
static int32_t WavCmd(p_shell_context_t context, int32_t argc, char **argv);

static void txCallback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData);
static void rxCallback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData);

/*******************************************************************************
 * Variables
 ******************************************************************************/
static const shell_command_context_t xLedCommand = {"led",
                                                    "\r\n\"led arg1 arg2\":\r\n Usage:\r\n    arg1: 1|2|3|4...         "
                                                    "   Led index\r\n    arg2: on|off                Led status\r\n",
                                                    LedControl, 2};
static const shell_command_context_t xPmdCommand = {"pmd", "\r\n\"pmd arg1\":\r\n Usage:\r\n    arg1: File Path\r\n", PmdCmd, 1};
static const shell_command_context_t xWavCommand = {"wav", "\r\n\"wav arg1\":\r\n Usage:\r\n    arg1: File Path\r\n", WavCmd, 1};

AT_NONCACHEABLE_SECTION(static FATFS g_fileSystem); /* File system object */

static const sdmmchost_detect_card_t s_sdCardDetect = {
#ifndef BOARD_SD_DETECT_TYPE
    .cdType = kSDMMCHOST_DetectCardByGpioCD,
#else
    .cdType = BOARD_SD_DETECT_TYPE,
#endif
    .cdTimeOut_ms = (~0U),
};

volatile uint32_t fullBlock = 0;
volatile uint32_t emptyBlock = BUFFER_NUM;

AT_NONCACHEABLE_SECTION_INIT(sai_edma_handle_t txHandle) = {0};
edma_handle_t dmaTxHandle = {0};
AT_NONCACHEABLE_SECTION_INIT(sai_edma_handle_t rxHandle) = {0};
edma_handle_t dmaRxHandle = {0};
static sai_transfer_format_t format = {0};
AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t audioBuff[BUFFER_SIZE * BUFFER_NUM], 4);
static uint8_t renderBuff[BUFFER_SIZE * BUFFER_NUM];
wm8960_handle_t codecHandle = {0};

#if defined(FSL_FEATURE_SOC_LPI2C_COUNT) && (FSL_FEATURE_SOC_LPI2C_COUNT)
lpi2c_master_handle_t i2cHandle = {0};
#else
i2c_master_handle_t i2cHandle = {{0, 0, kI2C_Write, 0, 0, NULL, 0}, 0, 0, NULL, NULL};
#endif
volatile bool istxFinished = false;
volatile bool isrxFinished = false;
volatile uint32_t beginCount = 0;
volatile uint32_t sendCount = 0;
volatile uint32_t receiveCount = 0;
volatile bool sdcard = false;

static void wav_header(uint8_t *header, uint32_t sampleRate, uint32_t bitsPerFrame, uint32_t fileSize)
{
    uint32_t totalDataLen = fileSize - 8U;
    uint32_t audioDataLen = fileSize - 44U;
    uint32_t byteRate = sampleRate * (bitsPerFrame / 8U) * 2U;
    header[0] = 'R';
    header[1] = 'I';
    header[2] = 'F';
    header[3] = 'F';
    header[4] = (totalDataLen & 0xff); /* file-size (equals file-size - 8) */
    header[5] = ((totalDataLen >> 8U) & 0xff);
    header[6] = ((totalDataLen >> 16U) & 0xff);
    header[7] = ((totalDataLen >> 24U) & 0xff);
    header[8] = 'W'; /* Mark it as type "WAVE" */
    header[9] = 'A';
    header[10] = 'V';
    header[11] = 'E';
    header[12] = 'f'; /* Mark the format section 'fmt ' chunk */
    header[13] = 'm';
    header[14] = 't';
    header[15] = ' ';
    header[16] = 16; /* 4 bytes: size of 'fmt ' chunk, Length of format data.  Always 16 */
    header[17] = 0;
    header[18] = 0;
    header[19] = 0;
    header[20] = 1; /* format = 1 ,Wave type PCM */
    header[21] = 0;
    header[22] = 2; /* channels */
    header[23] = 0;
    header[24] = (sampleRate & 0xff);
    header[25] = ((sampleRate >> 8U) & 0xff);
    header[26] = ((sampleRate >> 16U) & 0xff);
    header[27] = ((sampleRate >> 24U) & 0xff);
    header[28] = (byteRate & 0xff);
    header[29] = ((byteRate >> 8U) & 0xff);
    header[30] = ((byteRate >> 16U) & 0xff);
    header[31] = ((byteRate >> 24U) & 0xff);
    header[32] = (2 * bitsPerFrame / 8); /* block align */
    header[33] = 0;
    header[34] = 16; /* bits per sample */
    header[35] = 0;
    header[36] = 'd'; /*"data" marker */
    header[37] = 'a';
    header[38] = 't';
    header[39] = 'a';
    header[40] = (audioDataLen & 0xff); /* data-size (equals file-size - 44).*/
    header[41] = ((audioDataLen >> 8) & 0xff);
    header[42] = ((audioDataLen >> 16) & 0xff);
    header[43] = ((audioDataLen >> 24) & 0xff);
}
/*******************************************************************************
 * Code
 ******************************************************************************/

/*
 * AUDIO PLL setting: Frequency = Fref * (DIV_SELECT + NUM / DENOM)
 *                              = 24 * (32 + 77/100)
 *                              = 786.48 MHz
 */
const clock_audio_pll_config_t audioPllConfig = {
    .loopDivider = 30,  /* PLL loop divider. Valid range for DIV_SELECT divider value: 27~54. */
    .postDivider = 1,   /* Divider after the PLL, should only be 1, 2, 4, 8, 16. */
    .numerator = 10,    /* 30 bit numerator of fractional loop divider. */
    .denominator = 100, /* 30 bit denominator of fractional loop divider */
};
    

void BOARD_EnableSaiMclkOutput(bool enable)
{
    if (enable)
    {
        IOMUXC_GPR->GPR1 |= IOMUXC_GPR_GPR1_SAI1_MCLK_DIR_MASK;
    }
    else
    {
        IOMUXC_GPR->GPR1 &= (~IOMUXC_GPR_GPR1_SAI1_MCLK_DIR_MASK);
    }
}


static void txCallback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData)
{
    sai_transfer_t xfer = {0};

    sendCount++;
    emptyBlock++;
}

static void rxCallback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData)
{
    sai_transfer_t xfer = {0};

    receiveCount++;

    fullBlock++;
}


void Led_Init(void)
{
    LED_1_INIT();
}

void SHELL_SendDataCallback(uint8_t *buf, uint32_t len)
{
    while (len--)
    {
        PUTCHAR(*(buf++));
    }
}

void SHELL_ReceiveDataCallback(uint8_t *buf, uint32_t len)
{
    while (len--)
    {
        *(buf++) = GETCHAR();
    }
}

static int32_t LedControl(p_shell_context_t context, int32_t argc, char **argv)
{
    int32_t kLedIndex = ((int32_t)atoi(argv[1]));
    char *kLedCommand = argv[2];

    /* Check second argument to control led */
    switch (kLedIndex)
    {
#if defined(LED_NUMBERS) && LED_NUMBERS > 0
        case 1:
            if (strcmp(kLedCommand, "on") == 0)
            {
                LED_1_ON();
            }
            else if (strcmp(kLedCommand, "off") == 0)
            {
                LED_1_OFF();
            }
            else
            {
                SHELL_Printf("Control conmmand is wrong!\r\n");
            }
            break;
#endif
#if defined(LED_NUMBERS) && LED_NUMBERS > 1
        case 2:
            if (strcmp(kLedCommand, "on") == 0)
            {
                LED_2_ON();
            }
            else if (strcmp(kLedCommand, "off") == 0)
            {
                LED_2_OFF();
            }
            else
            {
                SHELL_Printf("Control conmmand is wrong!\r\n");
            }
            break;
#endif
#if defined(LED_NUMBERS) && LED_NUMBERS > 2
        case 3:
            if (strcmp(kLedCommand, "on") == 0)
            {
                LED_3_ON();
            }
            else if (strcmp(kLedCommand, "off") == 0)
            {
                LED_3_OFF();
            }
            else
            {
                SHELL_Printf("Control conmmand is wrong!\r\n");
            }
            break;
#endif
#if defined(LED_NUMBERS) && LED_NUMBERS > 3
        case 4:
            if (strcmp(kLedCommand, "on") == 0)
            {
                LED_4_ON();
            }
            else if (strcmp(kLedCommand, "off") == 0)
            {
                LED_4_OFF();
            }
            else
            {
                SHELL_Printf("Control conmmand is wrong!\r\n");
            }
            break;
#endif
        default:
            SHELL_Printf("LED index is wrong\r\n");
            break;
    }
    return 0;
}

void Delay(uint32_t x) {
  volatile int t = x;
  while (t--);
}

static int32_t PmdCmd(p_shell_context_t context, int32_t argc, char **argv) {
  char *filename = argv[1];
  
  uint32_t i = 0;
    uint32_t txindex = 0;
    uint32_t rxindex = 0;
    uint32_t sdReadCount = 0;
    uint32_t bytesWritten;
    int error;
    sai_transfer_t xfer = {0};
    FIL output;
    uint8_t header[44] = {0};
    uint32_t fileSize;

    /* Clear the status */
    isrxFinished = false;
    receiveCount = 0;
    istxFinished = false;
    sendCount = 0;
    sdcard = true;
    
    error = pmd_play(filename, "/"); 
    if (error != 0)
    {
            PRINTF("Open file failed.\r\n");
            return 1;
    }
    
    
    fileSize = pmd_length_sec() * 44100 * 2 * 2 + 44;
    
    /*
    f_open(&output, "/out.wav", (FA_CREATE_ALWAYS | FA_WRITE));
    wav_header(header, SAMPLE_RATE, 16, pmd_length_sec() * SAMPLE_RATE * 2 * 2 + 44);
    error = f_write(&output, (void *)header, 44U, (UINT *)&bytesWritten);
    PRINTF("Estimated file size %d KB.\r\n", fileSize / 1024);
    if ((error) || (bytesWritten != 44))
    {
        PRINTF("Write file failed. \r\n");
    }
    
    PRINTF("Write header successed.\r\n");
    for (i = 0; i < (fileSize / (BUFFER_SIZE * BUFFER_NUM)); i++) {
        pmd_renderer((short *)(renderBuff), BUFFER_SIZE * BUFFER_NUM / 4);
        f_write(&output, (void *)(renderBuff), BUFFER_SIZE * BUFFER_NUM, (UINT *)&bytesWritten);
    }*/
    txindex = 0;
    rxindex = 0;
    emptyBlock = 0;
    fullBlock = 0;
    memset(audioBuff, 0, BUFFER_SIZE * BUFFER_NUM);
    for (i = 0; i < BUFFER_NUM; i++)
    {
        pmd_renderer((short *)(renderBuff + i * BUFFER_SIZE), BUFFER_SIZE / 4);
        memcpy(audioBuff + i * BUFFER_SIZE, renderBuff + i * BUFFER_SIZE, BUFFER_SIZE);
        sdReadCount++;
        fullBlock++;
    }
    
    SCB_InvalidateDCache();
    SCB_EnableDCache();
    
    /* Wait for playback finished */
    while (sdReadCount < (fileSize / BUFFER_SIZE))
    {
        if (emptyBlock > 0)
        {
            pmd_renderer((short *)(renderBuff + rxindex * BUFFER_SIZE), BUFFER_SIZE / 4);
            memcpy(audioBuff + rxindex * BUFFER_SIZE, renderBuff + rxindex * BUFFER_SIZE, BUFFER_SIZE);
            rxindex = (rxindex + 1U) % BUFFER_NUM;
            emptyBlock--;
            fullBlock++;
            sdReadCount++;
        }

        if (fullBlock > 0)
        {
            xfer.dataSize = BUFFER_SIZE;
            xfer.data = audioBuff + txindex * BUFFER_SIZE;
            txindex = (txindex + 1U) % BUFFER_NUM;
            SAI_TransferSendEDMA(DEMO_SAI, &txHandle, &xfer);
            fullBlock--;
        }
    }
    
    SCB_DisableDCache();
    
    //f_close(&output);
    
    PRINTF("Finished.\r\n");
    

    return 0;
}

static int32_t WavCmd(p_shell_context_t context, int32_t argc, char **argv) {
  char *filename = argv[1];
  
    uint32_t i = 0;
    uint32_t bytesWritten = 0;
    uint32_t bytesRead = 0;
    uint32_t txindex = 0;
    uint32_t rxindex = 0;
    uint32_t sdReadCount = 0;
    uint8_t header[44] = {0};
    uint32_t fileSize;
    FIL g_fileObject;
    FRESULT error;
    sai_transfer_t xfer = {0};

    /* Clear the status */
    isrxFinished = false;
    receiveCount = 0;
    istxFinished = false;
    sendCount = 0;
    sdcard = true;

    error = f_open(&g_fileObject, filename, (FA_READ));
    if (error)
    {
            PRINTF("Open file failed.\r\n");
            return 1;
    }

    /* Playback the record file */
    PRINTF("\r\nPlayback the recorded file...");
    txindex = 0;
    rxindex = 0;
    emptyBlock = 0;
    fullBlock = 0;
    memset(audioBuff, 0, BUFFER_SIZE * BUFFER_NUM);
    if (f_lseek(&g_fileObject, 44U))
    {
        PRINTF("Set file pointer position failed. \r\n");
    }
    
    fileSize = f_size(&g_fileObject);

    for (i = 0; i < BUFFER_NUM; i++)
    {
        f_read(&g_fileObject, (void *)(audioBuff + i * BUFFER_SIZE), BUFFER_SIZE, (UINT *)&bytesRead);
        sdReadCount++;
        fullBlock++;
    }

    /* Wait for playback finished */
    while (sdReadCount < (fileSize / 512))
    {
        if (emptyBlock > 0)
        {
            f_read(&g_fileObject, (void *)(audioBuff + rxindex * BUFFER_SIZE), BUFFER_SIZE, (UINT *)&bytesRead);
            rxindex = (rxindex + 1U) % BUFFER_NUM;
            emptyBlock--;
            fullBlock++;
            sdReadCount++;
        }

        if (fullBlock > 0)
        {
            xfer.dataSize = BUFFER_SIZE;
            xfer.data = audioBuff + txindex * BUFFER_SIZE;
            txindex = (txindex + 1U) % BUFFER_NUM;
            SAI_TransferSendEDMA(DEMO_SAI, &txHandle, &xfer);
            fullBlock--;
        }
    }
    f_close(&g_fileObject);
    PRINTF("\r\nPlayback is finished!\r\n");
    return 0;
}

static void BOARD_USDHCClockConfiguration(void)
{
    /*configure system pll PFD2 fractional divider to 18*/
    CLOCK_InitSysPfd(kCLOCK_Pfd0, 0x12U);
    /* Configure USDHC clock source and divider */
    CLOCK_SetDiv(kCLOCK_Usdhc1Div, 0U);
    CLOCK_SetMux(kCLOCK_Usdhc1Mux, 1U);
}

static status_t sdcardWaitCardInsert(void)
{
    /* Save host information. */
    g_sd.host.base = SD_HOST_BASEADDR;
    g_sd.host.sourceClock_Hz = SD_HOST_CLK_FREQ;
    /* card detect type */
    g_sd.usrParam.cd = &s_sdCardDetect;
    /* SD host init function */
    if (SD_HostInit(&g_sd) != kStatus_Success)
    {
        PRINTF("\r\nSD host init fail\r\n");
        return kStatus_Fail;
    }
    /* power off card */
    SD_PowerOffCard(g_sd.host.base, g_sd.usrParam.pwr);
    /* wait card insert */
    if (SD_WaitCardDetectStatus(SD_HOST_BASEADDR, &s_sdCardDetect, true) == kStatus_Success)
    {
        PRINTF("\r\nCard inserted.\r\n");
        /* power on the card */
        SD_PowerOnCard(g_sd.host.base, g_sd.usrParam.pwr);
    }
    else
    {
        PRINTF("\r\nCard detect fail.\r\n");
        return kStatus_Fail;
    }

    return kStatus_Success;
}

static int MOUNT_SDCard(void)
{
    FRESULT error;
    const TCHAR driverName[3U] = {SDDISK + '0', ':', '/'};

    // clear FATFS manually
    memset((void *)&g_fileSystem, 0, sizeof(g_fileSystem));

    /* Wait for the card insert. */
    if (sdcardWaitCardInsert() != kStatus_Success)
    {
        PRINTF("Card not inserted.\r\n");
        return -1;
    }

    // Mount the driver
    if (f_mount(&g_fileSystem, driverName, 0))
    {
        PRINTF("Mount volume failed.\r\n");
        return -2;
    }

#if (_FS_RPATH >= 2U)
    if (f_chdrive((char const *)&driverName[0U]))
    {
        PRINTF("Change drive failed.\r\n");
        return -3;
    }
#endif

    return 0;
}

/*! @brief Main function */
int main(void)
{
    shell_context_struct user_context;
    sai_config_t config;
    uint32_t mclkSourceClockHz = 0U;
    edma_config_t dmaConfig = {0};
#if defined(FSL_FEATURE_SOC_LPI2C_COUNT) && (FSL_FEATURE_SOC_LPI2C_COUNT)
    lpi2c_master_config_t i2cConfig = {0};
#else
    i2c_master_config_t i2cConfig = {0};
#endif
    uint32_t i2cSourceClock;

    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    CLOCK_InitAudioPll(&audioPllConfig);
    BOARD_USDHCClockConfiguration();
    BOARD_InitDebugConsole();
    
    /*Clock setting for LPI2C*/
    CLOCK_SetMux(kCLOCK_Lpi2cMux, DEMO_LPI2C_CLOCK_SOURCE_SELECT);
    CLOCK_SetDiv(kCLOCK_Lpi2cDiv, DEMO_LPI2C_CLOCK_SOURCE_DIVIDER);
    
    /*Clock setting for SAI1*/
    CLOCK_SetMux(kCLOCK_Sai1Mux, DEMO_SAI1_CLOCK_SOURCE_SELECT);
    CLOCK_SetDiv(kCLOCK_Sai1PreDiv, DEMO_SAI1_CLOCK_SOURCE_PRE_DIVIDER);  
    CLOCK_SetDiv(kCLOCK_Sai1Div, DEMO_SAI1_CLOCK_SOURCE_DIVIDER);

    /* Enable clock gate for GPIO1 */
    CLOCK_EnableClock(kCLOCK_Gpio1);    
    
    /*Enable MCLK clock*/
    BOARD_EnableSaiMclkOutput(true);
    
    /* Create EDMA handle */
    /*
     * dmaConfig.enableRoundRobinArbitration = false;
     * dmaConfig.enableHaltOnError = true;
     * dmaConfig.enableContinuousLinkMode = false;
     * dmaConfig.enableDebugMode = false;
     */
    EDMA_GetDefaultConfig(&dmaConfig);
    EDMA_Init(EXAMPLE_DMA, &dmaConfig);
    EDMA_CreateHandle(&dmaTxHandle, EXAMPLE_DMA, EXAMPLE_TX_CHANNEL);
    EDMA_CreateHandle(&dmaRxHandle, EXAMPLE_DMA, EXAMPLE_RX_CHANNEL);

    DMAMUX_Init(EXAMPLE_DMAMUX);
    DMAMUX_SetSource(EXAMPLE_DMAMUX, EXAMPLE_TX_CHANNEL, (uint8_t)EXAMPLE_SAI_TX_SOURCE);
    DMAMUX_EnableChannel(EXAMPLE_DMAMUX, EXAMPLE_TX_CHANNEL);
    DMAMUX_SetSource(EXAMPLE_DMAMUX, EXAMPLE_RX_CHANNEL, (uint8_t)EXAMPLE_SAI_RX_SOURCE);
    DMAMUX_EnableChannel(EXAMPLE_DMAMUX, EXAMPLE_RX_CHANNEL);

    /* Init SAI module */
    /*
     * config.masterSlave = kSAI_Master;
     * config.mclkSource = kSAI_MclkSourceSysclk;
     * config.protocol = kSAI_BusLeftJustified;
     * config.syncMode = kSAI_ModeAsync;
     * config.mclkOutputEnable = true;
     */
    SAI_TxGetDefaultConfig(&config);
    SAI_TxInit(DEMO_SAI, &config);

    /* Initialize SAI Rx */
    SAI_RxGetDefaultConfig(&config);
    SAI_RxInit(DEMO_SAI, &config);

    /* Configure the audio format */
    format.bitWidth = kSAI_WordWidth16bits;
    format.channel = 0U;
    format.sampleRate_Hz = SAMPLE_RATE;
    format.masterClockHz = DEMO_SAI_CLK_FREQ;
    format.protocol = config.protocol;
    format.stereo = kSAI_Stereo;
#if defined(FSL_FEATURE_SAI_FIFO_COUNT) && (FSL_FEATURE_SAI_FIFO_COUNT > 1)
    format.watermark = FSL_FEATURE_SAI_FIFO_COUNT / 2U;
#endif

    /* Configure Sgtl5000 I2C */
    codecHandle.base = DEMO_I2C;
    codecHandle.i2cHandle = &i2cHandle;
    i2cSourceClock = DEMO_I2C_CLK_FREQ;

#if defined(FSL_FEATURE_SOC_LPI2C_COUNT) && (FSL_FEATURE_SOC_LPI2C_COUNT)
    /*
     * i2cConfig.debugEnable = false;
     * i2cConfig.ignoreAck = false;
     * i2cConfig.pinConfig = kLPI2C_2PinOpenDrain;
     * i2cConfig.baudRate_Hz = 100000U;
     * i2cConfig.busIdleTimeout_ns = 0;
     * i2cConfig.pinLowTimeout_ns = 0;
     * i2cConfig.sdaGlitchFilterWidth_ns = 0;
     * i2cConfig.sclGlitchFilterWidth_ns = 0;
     */
    LPI2C_MasterGetDefaultConfig(&i2cConfig);
    LPI2C_MasterInit(DEMO_I2C, &i2cConfig, i2cSourceClock);
    LPI2C_MasterTransferCreateHandle(DEMO_I2C, &i2cHandle, NULL, NULL);
#else
    /*
     * i2cConfig.baudRate_Bps = 100000U;
     * i2cConfig.enableStopHold = false;
     * i2cConfig.glitchFilterWidth = 0U;
     * i2cConfig.enableMaster = true;
     */
    I2C_MasterGetDefaultConfig(&i2cConfig);
    I2C_MasterInit(DEMO_I2C, &i2cConfig, i2cSourceClock);
    I2C_MasterTransferCreateHandle(DEMO_I2C, &i2cHandle, NULL, NULL);
#endif

    WM8960_Init(&codecHandle, NULL);
    WM8960_ConfigDataFormat(&codecHandle, format.masterClockHz, format.sampleRate_Hz, format.bitWidth);
    WM8960_SetVolume(&codecHandle, kWM8960_ModuleHP, 0x5F);
    
    SAI_TransferTxCreateHandleEDMA(DEMO_SAI, &txHandle, txCallback, NULL, &dmaTxHandle);
    SAI_TransferRxCreateHandleEDMA(DEMO_SAI, &rxHandle, rxCallback, NULL, &dmaRxHandle);

    mclkSourceClockHz = DEMO_SAI_CLK_FREQ;
    SAI_TransferTxSetFormatEDMA(DEMO_SAI, &txHandle, &format, mclkSourceClockHz, format.masterClockHz);
    SAI_TransferRxSetFormatEDMA(DEMO_SAI, &rxHandle, &format, mclkSourceClockHz, format.masterClockHz);

    /* Enable interrupt to handle FIFO error */
    SAI_TxEnableInterrupts(DEMO_SAI, kSAI_FIFOErrorInterruptEnable);
    SAI_RxEnableInterrupts(DEMO_SAI, kSAI_FIFOErrorInterruptEnable);
    //EnableIRQ(DEMO_SAI_TX_IRQ);
    //EnableIRQ(DEMO_SAI_RX_IRQ);

    /* Init led */
    Led_Init();
    
    // Init the SD card
    if (0 != MOUNT_SDCard())
    {
        PRINTF("SD card mount error. Demo stopped!");
        return -1;
    }
    
    pmd_init();
    
    if (pmd_play("/th04_10.m", "/") != 0)
    {
            return -1;
    }
    
    uint32_t bytesWritten = 0;
    uint32_t fileSize;
    FIL fp;
    unsigned char *fileBuf;
    
    fileSize = pmd_length_sec() * 44100 * 2 * 2 + 44;
    
    fileBuf = malloc(fileSize);
    
    SCB_InvalidateDCache();
    SCB_EnableDCache();
    
    LED_1_ON();
    
    wav_header(fileBuf, 44100, 16, fileSize);
    for (int i = 0; i < (fileSize/2048); i++) {
      pmd_renderer((short *)(fileBuf + i*2048 + 44), 2048 / 4);
    }
    
    LED_1_OFF();
    
    SCB_DisableDCache();
    
    f_open(&fp, "/output_imx.wav", (FA_CREATE_ALWAYS | FA_WRITE));
    f_write(&fp, (void *)(fileBuf), fileSize, (UINT *)&bytesWritten);
    f_close(&fp);
    
    /* Init SHELL */
    SHELL_Init(&user_context, SHELL_SendDataCallback, SHELL_ReceiveDataCallback, SHELL_Printf, "ZPR>> ");

    /* Add new command to commands list */
    SHELL_RegisterCommand(&xLedCommand);
    SHELL_RegisterCommand(&xPmdCommand);
    SHELL_RegisterCommand(&xWavCommand);

    SHELL_Main(&user_context);

    while (1)
    {
    }
}
