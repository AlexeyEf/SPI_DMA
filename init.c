
#include <init.h>

//configuration of HSI oscillator 16 MHz for system clock
void rcc_init(void)
  {
		RCC->CFGR |= RCC_CFGR_SW_HSI; //use HSI oscillator for system clock
		RCC->CR |= RCC_CR_HSION;  //turn on oscillator HSI for clocking ADC1
    while(!(RCC->CR&RCC_CR_HSIRDY)); //waiting for stabilization of HSI
	}
	
	//ports configuration
void gpio_init(void)
  {

  GPIO_InitTypeDef port;  //definition of variable of GPIO_InitTypeDef type to access to elements of structure GPIO_InitTypeDef to configure port GPIOA & GPIOC
    
  //clock configuration
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOC,ENABLE); //enable clock of ports A & C
    
  //Ports configuration
  //configuration of port A5 & A12 for SPI outputs SCK & MOSI
  port.GPIO_Pin=GPIO_Pin_5|GPIO_Pin_12;  //change pins
  port.GPIO_Mode=GPIO_Mode_AF;  //this ports use for alternative function (SPI)
  port.GPIO_Speed=GPIO_Speed_40MHz; //set max rate via this pins
  port.GPIO_OType=GPIO_OType_PP;
  //call function to configure GPIOA Pin 4, GPIO_Pin_5 and GPIO_Pin_12
  GPIO_Init(GPIOA,&port);
	
	//configuration of port C1 & C2 for master outputs SlaveSelect for transmitting to device 1 & device 2	
	port.GPIO_Pin=GPIO_Pin_1|GPIO_Pin_2; //change pins
  port.GPIO_Mode=GPIO_Mode_OUT; //this ports use as outputs
  port.GPIO_Speed=GPIO_Speed_40MHz; //set max rate via this pins
  port.GPIO_OType=GPIO_OType_PP;
  //call function to configure GPIOC Pin 1 & GPIOC Pin 2
  GPIO_Init(GPIOC,&port);
  
  }

	//spi configuration
void spi_init(void)
  {
    SPI_InitTypeDef spi; //definition of variable of SPI_InitTypeDef type to access to elements of structure GPIO_InitTypeDef to configure SPI

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE); //enable clock of SPI
    
    
		spi.SPI_Direction = SPI_Direction_1Line_Tx;  // specifies SPI direction
		spi.SPI_DataSize = SPI_DataSize_16b;  //specifies size of transmitting data - 16 bits
		spi.SPI_CPOL = SPI_CPOL_Low;  //specifies the serial clock steady state - low state
		spi.SPI_CPHA = SPI_CPHA_1Edge; // specifies the clock active edge for the bit capture - active 1 edge
		spi.SPI_NSS = SPI_NSS_Hard; //specifies whether the NSS signal is managed by hardware or by software - by hardware                                       
		spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;//spi clock frequency=16 MHz/8=2 MHz
		spi.SPI_FirstBit = SPI_FirstBit_MSB; //first bit-MSB
 
		spi.SPI_Mode = SPI_Mode_Master;// change master mode
		SPI_Init(SPI1, &spi);
		
		SPI_SSOutputCmd(SPI1, ENABLE); // configure output NSS as output  
		SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE); // dma request from spi after end of transmitting 16 bits
  
		SPI_Cmd(SPI1, ENABLE);// start DMA
  
  }

	//dma configuration
void dma_init(void)
  {
    DMA_InitTypeDef dma; //definition of variable of DMA_InitTypeDef type to access to elements of structure GPIO_InitTypeDef to configure DMA
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE); //enable clock of DMA
    
    dma.DMA_PeripheralBaseAddr=(uint32_t) (&(SPI1->DR)); //Specifies the peripheral base address of data spi register SPI1->DR for DMA1 Channel3

    dma.DMA_MemoryBaseAddr=(uint32_t) (&(data));     //Specifies the memory base address of first element of array data[] for DMA1 Channel3

    dma.DMA_DIR=DMA_DIR_PeripheralDST;      //Specifies if the peripheral is destination
                                       

    dma.DMA_BufferSize=6;         //Specifies the buffer size=6 of 16 bits words
                                        

    dma.DMA_PeripheralInc=DMA_PeripheralInc_Disable;      //Specifies whether the Peripheral address register is incremented or not
                                        

    dma.DMA_MemoryInc=DMA_MemoryInc_Enable;          // Specifies whether the memory address register is incremented or not
                                        

    dma.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord;           // Specifies the Peripheral data width - 16 bit
                                        

    dma.DMA_MemoryDataSize=DMA_MemoryDataSize_HalfWord;     // Specifies the Memory data width - 16 bit
                                        

    dma.DMA_Mode=DMA_Mode_Circular;               // Specifies the operation mode of the DMA1 Channel3 - Circular mode
                                        

    dma.DMA_Priority=DMA_Priority_High;           // Specifies the software priority for the DMA1 Channel3 - high priority
                                        

    dma.DMA_M2M=DMA_M2M_Disable;         //Specifies if the DMA1 Channel3 will be used in memory-to-memory transfer - disable
                                        
      
    DMA_Init( DMA1_Channel3, &dma);
  
    DMA_ITConfig(DMA1_Channel3, DMA_IT_HT|DMA_IT_TC, ENABLE);//enable interrupt if half and full data was transmitted
		
    DMA_Cmd(DMA1_Channel3, ENABLE); //start dma
  
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);//enable interrupt from DMA in NVIC controller
  }

	//timer configuration
void tim_init(void)
  {

  TIM_TimeBaseInitTypeDef timer; //definition of variable of TimeBaseInitTypeDef type to access to elements of structure TimeBaseInitTypeDef to configure timer TIM2
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE); //enable clock of basic timer TIM2
      
  //Timer configuration
  //configuration of elements of structure TimeBaseInitTypeDef
  timer.TIM_Prescaler=15999; //prescaller of system clock 16 MHz = 16000
  timer.TIM_Period=100; // period=100 ms
  //call function to configure timer TIM6
  TIM_TimeBaseInit(TIM6,&timer);
      
  TIM6->DIER |= TIM_DIER_UIE;  //enable interrupt
  NVIC_EnableIRQ(TIM6_IRQn);    //enable interrupt in NVIC controller
  }

	void TIM6_IRQHandler(void)
 {
   TIM_Cmd(TIM6, DISABLE); //stop timer
   TIM_ClearITPendingBit(TIM6, TIM_EventSource_Update);//Clear ITPending Bit
 }
	

void DMA1_Channel3_IRQHandler(void) 
  {
		  
      if(DMA_GetITStatus(DMA1_IT_HT3)==SET)  //end of transmitting of first half data to device 1
			  
			{				
				// waiting for end of transmitting
				while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE) == SET);
				while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_BSY) == RESET);
			
				  ////set portC1
				  //end of transmitting to device 1
          GPIO_SetBits(GPIOC, GPIO_Pin_1);
				
					//delay of 100 ms
				  delay();
				
					//start transmitting to device 2
          GPIO_ResetBits(GPIOC, GPIO_Pin_2);   //reset portC2, which use to control transmitting to device 2, to transmit second half data to device 2 
			}
			
			if(DMA_GetITStatus(DMA_IT_TC)==SET)   //end of transmitting of second half data to device 1
				
			{
				// waiting for end of transmitting
				while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE) == SET);
				while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_BSY) == RESET);
				
				//set portC2 
				//end of transmitting to device 2
				GPIO_SetBits(GPIOC, GPIO_Pin_2);
				
				//delay 100 ms
				  delay();
							
				//start of transmitting to device 1
          GPIO_ResetBits(GPIOC, GPIO_Pin_1);
				
			}
			
				DMA_ClearITPendingBit(DMA_IT_HT|DMA_IT_TC);  //reset ITPendingBits
   
	}
	
	void delay (void)
 {
   TIM_Cmd(TIM6, ENABLE);//start counting of timer TIM6
 }