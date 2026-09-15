#ifndef __UART_H
#define __UART_H

#include <stdint.h>
#include <stdio.h>

#include "ti_msp_dl_config.h"

/* Compatibility exports: the latest complete packet is published here. */
extern volatile char UART_RxPacket[];
extern volatile uint8_t UART_RxFlag;
extern volatile uint32_t UART_RxPacketTimeMs;
extern volatile uint32_t UART_RxOverwriteCount;

void UART_SendByte(uint8_t Byte);
void UART_SendArray(uint8_t *Array, uint16_t Length);
void UART_SendString(char *String);
void UART_SendNumber(uint32_t Number, uint8_t Length);
void UART_Printf(char *format, ...);

/*
 * Atomically takes the newest complete @...\r\n packet.
 * arrival_ms is captured in the UART ISR, not when the main loop wakes.
 * max_interarrival_gap_ms covers every complete packet since the prior take,
 * including packets overwritten while OLED or other foreground work ran.
 */
uint8_t UART_TakeLatestPacket(char *packet,
                              uint16_t capacity,
                              uint32_t *arrival_ms,
                              uint32_t *max_interarrival_gap_ms);
void UART_ResetRxMailbox(void);
void Serial_INST_IRQHandler(void);

#endif
