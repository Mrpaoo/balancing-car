#include"remote_ctrl.h"
#include"uart_drv.h"
#include"usart.h"
#include"global_def.h"

#define HEADER  0xAA
#define END  0x55
#define TYPE_MOVE 0x01
#define TYPE_MODE_SHIFT 0x02

static uint8_t     dma_buf[WIFI_BUF_SIZE];
static uint8_t     frame_buf[WIFI_BUF_SIZE];
static UartDma_Rx  g_wifi_uart;

static void wifi_on_frame(uint8_t *data, uint16_t len)
{
	/* MOVE: HEADER | TYPE | X | Y | CHK | END  (6 bytes) */
	/* MODE_SHIFT: HEADER | TYPE | MODE | CHK | END (5 bytes)*/
	if(len!=6 && len!=5)
		return;
	if (data[0] != HEADER || data[len - 1] != END)
		return;
	switch(data[1]){
		case TYPE_MOVE: {
			uint8_t chk = TYPE_MOVE ^ data[2] ^ data[3];
			if(data[4]!=chk)
				return;
			g_wifi_cmd.turn = (int32_t)data[2] * RC_SPEED_SCALE;
			g_wifi_cmd.v    = (int32_t)data[3] * RC_SPEED_SCALE;
			break;
		}
		case TYPE_MODE_SHIFT: {
			uint8_t chk = TYPE_MODE_SHIFT ^ data[2];
			if(data[3]!=chk)
				return;
			g_wifi_cmd.mode_request = (int32_t)data[2];
			break;
		}
	}

}

void Wifi_Init(void)
{
	/* PC11 pull-up for USART3 RX */
	GPIOC->PUPDR = (GPIOC->PUPDR & ~(3UL << 22)) | (1UL << 22);

	/* ESP8266 boot: IO high, RST pulse */
	HAL_GPIO_WritePin(ESP_IO_GPIO_Port, ESP_IO_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ESP8266_RST_GPIO_Port, ESP8266_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(ESP8266_RST_GPIO_Port, ESP8266_RST_Pin, GPIO_PIN_SET);    /* RST high */
	HAL_Delay(100);   /* wait for ESP8266 Wi-Fi connect + DHCP */

	/* Start DMA after ESP is booted to avoid baud mismatch noise */
	UartDma_Init(&g_wifi_uart, &huart3,
	             dma_buf, frame_buf, WIFI_BUF_SIZE,
	             wifi_on_frame);
}
