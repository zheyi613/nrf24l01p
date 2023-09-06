# nrf24l01p
NRF24L01+ driver

# Interface define 

- User can define interface according to your platform. It's easy to port.

```c = 8
#include "nrf24l01p.h"
#include "spi.h"
#include "gpio.h"
#include "main.h"
#include "dwt_delay.h"

/* Define user spi function */
#define spi_txrx(pTxData,pRxData)      \
        HAL_SPI_TransmitReceive(&hspi2, pTxData, pRxData, 1, 1);
#define spi_rx(pData, size)   \
        HAL_SPI_Receive(&hspi2, pData, size, 1)
#define spi_tx(pData, size)   \
        HAL_SPI_Transmit(&hspi2, pData, size, 1)
#define cs_high()       \
        HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET)
#define cs_low()        \
        HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET)
#define ce_high()       \
        HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_SET)
#define ce_low()        \
        HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET)
#define delay_us(val)   DWT_Delay(val)
```

# Example
Just present important parts

## PRX mode

```c
uint8_t payload[8];

int main(void)
{
  int result = 0;
  result = nrf24l01p_init(&nrf24l01p.param);
  if (result) /* If result equal to 1, device maybe lost */
    while (1)
      ;
  nrf24l01p_start_rx();

  while (1) {
    /* don't care about here */
    HAL_Delay(100);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  nrf24l01p_receive(payload);
}
```

## PTX mode

```c
int max_retransmit_flag = 0; /* check if max retransmission count or not */

int main(void)
{
  int result = 0;
  uint8_t payload[8] = {7, 6, 5, 4, 3, 2, 1, 0};
  result = nrf24l01p_init(&nrf24l01p.param);
  if (result) /* If result equal to 1, device maybe lost */
    while (1)
      ;

  while (1) {
    nrf24l01p_transmit(payload);
    HAL_Delay(100);
  }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  max_retransmit_flag = nrf24l01p_tx_irq();
  if (max_retransmit_flag) {
    /* failed */
  }
}

```

## PRX mode with ACK payload

```c
uint8_t payload[8];
uint8_t ack_payload[8] = {0, 1, 2, 3, 4, 5, 6, 7};

int main(void)
{
  int result = 0;
  result = nrf24l01p_init(&nrf24l01p.param);
  if (result) /* If result equal to 1, device maybe lost */
    while (1)
      ;
  nrf24l01p_start_rx();

  while (1) {
    /* don't care about here */
    HAL_Delay(100);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  nrf24l01p_write_ack_payload(ack_payload, 3);
  nrf24l01p_receive(payload);
}
```

## PTX mode with ACK payload

```c
int max_retransmit_flag = 0; /* check up to max retransmission count or not */
uint8_t ack_payload[8];

int main(void)
{
  int result = 0;
  uint8_t payload[8] = {7, 6, 5, 4, 3, 2, 1, 0};
  result = nrf24l01p_init(&nrf24l01p.param);
  if (result) /* If result equal to 1, device maybe lost */
    while (1)
      ;

  while (1) {
    nrf24l01p_transmit(payload, 8);
    HAL_Delay(100);
  }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  max_retransmit_flag = nrf24l01p_tx_irq(ack_payload);
  if (max_retransmit_flag) {
    /* failed */
  }
}

```
