#include "uart.h"

#include <stdarg.h>

#include "ti_msp_dl_config.h"

#define UART_RX_PACKET_CAPACITY 100u

extern volatile uint32_t system_ms;

volatile char UART_RxPacket[UART_RX_PACKET_CAPACITY];
volatile uint8_t UART_RxFlag;
volatile uint32_t UART_RxPacketTimeMs;
volatile uint32_t UART_RxOverwriteCount;

static char g_uartRxBuilding[UART_RX_PACKET_CAPACITY];
static uint8_t g_uartRxState;
static uint8_t g_uartRxLength;
static uint8_t g_uartRxOverflow;
static uint8_t g_uartHasPreviousArrival;
static uint32_t g_uartPreviousArrivalMs;
static volatile uint32_t g_uartPendingMaxGapMs;

static uint32_t UART_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1u;
    while (Y-- != 0u) {
        Result *= X;
    }
    return Result;
}

void UART_SendByte(uint8_t Byte)
{
    DL_UART_Main_transmitDataBlocking(Serial_INST, Byte);
}

void UART_SendArray(uint8_t *Array, uint16_t Length)
{
    uint16_t i;
    for (i = 0u; i < Length; ++i) {
        UART_SendByte(Array[i]);
    }
}

void UART_SendString(char *String)
{
    uint16_t i;
    for (i = 0u; String[i] != '\0'; ++i) {
        UART_SendByte((uint8_t)String[i]);
    }
}

void UART_SendNumber(uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0u; i < Length; ++i) {
        UART_SendByte((uint8_t)((Number /
            UART_Pow(10u, (uint32_t)(Length - i - 1u)) % 10u) + '0'));
    }
}

#ifndef UART_HOST_TEST_NO_STDIO_REDIRECT
int fputc(int ch, FILE *f)
{
    (void)f;
    UART_SendByte((uint8_t)ch);
    return ch;
}
#endif

void UART_Printf(char *format, ...)
{
    char String[100];
    va_list arg;
    va_start(arg, format);
    vsprintf(String, format, arg);
    va_end(arg);
    UART_SendString(String);
}

uint8_t UART_TakeLatestPacket(char *packet,
                              uint16_t capacity,
                              uint32_t *arrival_ms,
                              uint32_t *max_interarrival_gap_ms)
{
    uint16_t i = 0u;

    if ((packet == NULL) || (capacity == 0u)) {
        return 0u;
    }

    NVIC_DisableIRQ(Serial_INST_INT_IRQN);
    if (UART_RxFlag == 0u) {
        NVIC_EnableIRQ(Serial_INST_INT_IRQN);
        return 0u;
    }

    while ((i < (capacity - 1u)) && (UART_RxPacket[i] != '\0')) {
        packet[i] = UART_RxPacket[i];
        ++i;
    }
    packet[i] = '\0';
    if (arrival_ms != NULL) {
        *arrival_ms = UART_RxPacketTimeMs;
    }
    if (max_interarrival_gap_ms != NULL) {
        *max_interarrival_gap_ms = g_uartPendingMaxGapMs;
    }
    UART_RxFlag = 0u;
    g_uartPendingMaxGapMs = 0u;
    NVIC_EnableIRQ(Serial_INST_INT_IRQN);
    return 1u;
}

void UART_ResetRxMailbox(void)
{
    NVIC_DisableIRQ(Serial_INST_INT_IRQN);
    UART_RxFlag = 0u;
    UART_RxPacket[0] = '\0';
    UART_RxPacketTimeMs = system_ms;
    UART_RxOverwriteCount = 0u;
    g_uartPendingMaxGapMs = 0u;
    g_uartHasPreviousArrival = 0u;
    g_uartPreviousArrivalMs = system_ms;
    g_uartRxState = 0u;
    g_uartRxLength = 0u;
    g_uartRxOverflow = 0u;
    NVIC_EnableIRQ(Serial_INST_INT_IRQN);
}

static void UART_PublishRxPacket(uint32_t arrival_ms)
{
    uint8_t i;

    if (UART_RxFlag != 0u) {
        UART_RxOverwriteCount++;
    }
    for (i = 0u; i < g_uartRxLength; ++i) {
        UART_RxPacket[i] = g_uartRxBuilding[i];
    }
    UART_RxPacket[g_uartRxLength] = '\0';

    if (g_uartHasPreviousArrival != 0u) {
        uint32_t gap_ms = (uint32_t)(arrival_ms - g_uartPreviousArrivalMs);
        if (gap_ms > g_uartPendingMaxGapMs) {
            g_uartPendingMaxGapMs = gap_ms;
        }
    } else {
        g_uartHasPreviousArrival = 1u;
    }
    g_uartPreviousArrivalMs = arrival_ms;
    UART_RxPacketTimeMs = arrival_ms;
    UART_RxFlag = 1u;
}

void Serial_INST_IRQHandler(void)
{
    if (DL_UART_Main_getPendingInterrupt(Serial_INST) == DL_UART_IIDX_RX) {
        uint8_t RxData = DL_UART_Main_receiveData(Serial_INST);

        /* A new header always resynchronizes a damaged/incomplete packet. */
        if (RxData == '@') {
            g_uartRxState = 1u;
            g_uartRxLength = 0u;
            g_uartRxOverflow = 0u;
            return;
        }

        if (g_uartRxState == 1u) {
            if (RxData == '\r') {
                g_uartRxState = 2u;
            } else if (RxData == '\n') {
                g_uartRxState = 0u;
                g_uartRxLength = 0u;
                g_uartRxOverflow = 0u;
            } else if (g_uartRxLength < (UART_RX_PACKET_CAPACITY - 1u)) {
                g_uartRxBuilding[g_uartRxLength] = (char)RxData;
                g_uartRxLength++;
            } else {
                g_uartRxOverflow = 1u;
            }
        } else if (g_uartRxState == 2u) {
            if ((RxData == '\n') && (g_uartRxOverflow == 0u)) {
                UART_PublishRxPacket(system_ms);
            }
            g_uartRxState = 0u;
            g_uartRxLength = 0u;
            g_uartRxOverflow = 0u;
        }
    }
}
