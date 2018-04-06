#ifndef __SPI_COMM_H
#define __SPI_COMM_H
#include "stm32f1xx_hal.h"


void SPI_COMM_Function(uint8_t spi_order,uint8_t spi_tx_hig,uint8_t spi_tx_mid,uint8_t spi_tx_low);
void SYNC_COMM_TEST(void);

//void SPI_COMM_OPERATION(void);




#endif


