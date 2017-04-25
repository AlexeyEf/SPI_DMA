#include <init.h>
  
int main()
{  
  //call function of configuration of ports, spi and timer modules
  rcc_init();
  gpio_init();
  spi_init();
  dma_init();
  tim_init();  
  
  while(1);
  
   
} 