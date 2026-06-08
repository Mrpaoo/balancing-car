#include "remote_ctrl.h"
#include "uart_drv.h"
#include "usart.h"
#include "global_def.h"

#define HEADER     0xAA
#define END        0x55
#define TYPE_TRACK 0x01

static uint8_t     dma_buf[VISION_BUF_SIZE];
static uint8_t     frame_buf[VISION_BUF_SIZE];
static UartDma_Rx  g_vision_uart;

/*
 * OpenMV track packet:
 * HEADER | TYPE | X | Y | CHK | END
 * 0xAA   | 0x01 | X | Y | CHK | 0x55
 *
 * X/Y are int8 command values.
 * Only X is used for turn control.
 * Y is kept for checksum compatibility but ignored by control logic.
 *
 * CHK = TYPE ^ X ^ Y
 */
static void vision_parse_one_frame(uint8_t *data)
{
    if (data[0] != HEADER || data[5] != END)
        return;

    if (data[1] != TYPE_TRACK)
        return;

    uint8_t chk = TYPE_TRACK ^ data[2] ^ data[3];
    if (data[4] != chk)
        return;

    int8_t x = (int8_t)data[2];

    g_vision_cmd.turn = (int32_t)x * RC_SPEED_SCALE;

    /*
     * Y is intentionally ignored.
     * Do not update g_vision_cmd.v here.
     */
}

static void vision_on_frame(uint8_t *data, uint16_t len)
{
    /*
     * UART is a byte stream, so one ReceiveToIdle callback may contain:
     * - exactly one frame
     * - multiple frames stuck together
     * - noise before a valid frame
     *
     * Therefore we scan the received buffer and parse every valid frame.
     * If multiple valid frames exist, the later one naturally overwrites
     * the previous command, leaving g_vision_cmd.turn with the newest value.
     */
    for (uint16_t i = 0; i + 5 < len; i++) {
        if (data[i] != HEADER)
            continue;

        if (data[i + 1] != TYPE_TRACK)
            continue;

        if (data[i + 5] != END)
            continue;

        uint8_t chk = TYPE_TRACK ^ data[i + 2] ^ data[i + 3];
        if (data[i + 4] != chk)
            continue;

        vision_parse_one_frame(&data[i]);

        /*
         * Skip this complete 6-byte frame.
         * The for-loop will increment i once more, so use i += 5 here.
         */
        i += 5;
    }
}

void Vision_Init(void)
{
    UartDma_Init(&g_vision_uart, &huart6,
                 dma_buf, frame_buf, VISION_BUF_SIZE,
                 vision_on_frame);
}
