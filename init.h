
#include <stm32l1xx.h>
#include <system_stm32l1xx.h>
#include <stm32l1xx_gpio.h>
#include <stm32l1xx_spi.h>
#include <stm32l1xx_dma.h>
#include <stm32l1xx_rcc.h>


static uint16_t data[6]={0x54, 0xF8, 0x26, 0x10, 0x3B, 0x48};  //array of data to transmit via SPI

//function of configuration of ports, spi and tim modules
void gpio_init(void);
void spi_init(void);
void dma_init(void);
void tim_init(void);
void rcc_init(void);
void delay (void);