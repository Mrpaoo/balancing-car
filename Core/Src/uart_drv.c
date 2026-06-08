#include"uart_drv.h"
#include<string.h>

#define MAX_UART_RX 4

static UartDma_Rx *g_rx[MAX_UART_RX];
static uint8_t     g_rx_cnt;

void UartDma_Init(UartDma_Rx *u, UART_HandleTypeDef *huart,
                  uint8_t *dma_buf, uint8_t *frame_buf, uint16_t buf_size,
                  UartDma_Callback cb)
{
	u->huart     = huart;
	u->dma_buf   = dma_buf;
	u->frame_buf = frame_buf;
	u->buf_size  = buf_size;
	u->cb        = cb;

	if (g_rx_cnt < MAX_UART_RX)
		g_rx[g_rx_cnt++] = u;

	/* Clear stale FE/NE before starting DMA to avoid IDLE being lost */
	{
		uint32_t sr = huart->Instance->SR;
		if (sr & (USART_SR_FE | USART_SR_NE))
			(void)huart->Instance->DR;
	}

	HAL_StatusTypeDef rc = HAL_UARTEx_ReceiveToIdle_DMA(huart, dma_buf, buf_size);
	if (rc != HAL_OK) {
		__disable_irq();
		while (1) {}
	}
}

static void UartDma_OnRxEvent(UART_HandleTypeDef *huart, uint16_t len)
{
	for (uint8_t i = 0; i < g_rx_cnt; i++) {
		UartDma_Rx *u = g_rx[i];
		if (u->huart != huart) continue;

		if (len > 0 && len <= u->buf_size) {
			memcpy(u->frame_buf, u->dma_buf, len);
			if (u->cb)
				u->cb(u->frame_buf, len);
		}

		HAL_UARTEx_ReceiveToIdle_DMA(huart, u->dma_buf, u->buf_size);
		break;
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	UartDma_OnRxEvent(huart, Size);
}
