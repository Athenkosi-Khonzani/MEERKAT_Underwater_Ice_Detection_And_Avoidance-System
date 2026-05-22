/**
 ******************************************************************************
 * @file    mission_rx.c
 * @brief   Receive mission commands from ESP32 over UART3 (interrupt-driven)
 *
 *  Protocol from ESP32: ASCII lines terminated with '\n'
 *      "M:<value>\n"   mission command, e.g. "M:1\n"
 *      "C:<value>\n"   generic command (ignored for now)
 *
 *  On receipt of "M:1" we set missionInit = 1.
 ******************************************************************************
 */

#include "mission_rx.h"
#include <string.h>
#include <stdlib.h>

#define RX_LINE_MAX  32

volatile uint8_t missionInit = 0;

static UART_HandleTypeDef *s_huart = NULL;
static uint8_t  s_rx_byte;             /* single-byte ISR landing slot */
static char     s_line[RX_LINE_MAX];   /* line accumulator */
static uint16_t s_line_len = 0;

/* Forward declaration */
static void handle_line(const char *line);

void MissionRx_Init(UART_HandleTypeDef *huart)
{
    s_huart    = huart;
    s_line_len = 0;

    /* Arm the first interrupt-driven receive of one byte */
    HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1);
}

/* ---------------------------------------------------------------------------
 *  HAL RX-complete callback
 *
 *  Called from the UART3 ISR each time a single byte has been received.
 *  We accumulate bytes until '\n', then dispatch the line and re-arm.
 *
 *  Keep this function short — it runs in interrupt context.
 * ------------------------------------------------------------------------- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        char c = (char)s_rx_byte;

        if (c == '\r') {
            /* swallow CR */
        }
        else if (c == '\n') {
            s_line[s_line_len] = '\0';
            if (s_line_len > 0) {
                handle_line(s_line);
            }
            s_line_len = 0;
        }
        else if (s_line_len < RX_LINE_MAX - 1) {
            s_line[s_line_len++] = c;
        }
        else {
            /* overflow — drop the line */
            s_line_len = 0;
        }

        /* Re-arm for the next byte. Must do this every callback for IT mode. */
        HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1);
    }
}

/* ---------------------------------------------------------------------------
 *  Line dispatcher — runs in ISR context, keep it light.
 * ------------------------------------------------------------------------- */
static void handle_line(const char *line)
{
    /* "M:<value>"  -> mission command */
    if (line[0] == 'M' && line[1] == ':') {
        int value = atoi(&line[2]);
        if (value == 1) {
            missionInit = 1;
        }
        /* Add more mission values here as your protocol grows:
         *   else if (value == 2) { missionPause = 1; }
         *   else if (value == 0) { missionInit  = 0; }
         */
    }
    /* "C:<value>"  -> reserved for future generic commands */
}
