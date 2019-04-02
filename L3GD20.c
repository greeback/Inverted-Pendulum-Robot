#include "stm32f4xx.h"
#include "L3GD20.h"

uint16_t timeout;

void SPI_slaveSelect_ctrl(uint8_t state){
if(state){
GPIOE->BSRRH=0x8;
}
else {
while(SPI1->SR & SPI_SR_BSY);
GPIOE->BSRRL=0x8;
}
}

uint8_t Spi_sendrecv (uint8_t data)
{
  timeout = TIMEOUT_TIME;
  while( !(SPI1->SR & SPI_SR_TXE) &(timeout != 0))
    {
	timeout--;
    }	
  SPI1->DR = data;
  timeout = TIMEOUT_TIME;
  while( !(SPI1->SR & SPI_SR_RXNE)&(timeout != 0) )
    {
	timeout--;
    }	
  data = SPI1->DR;
  return data;
}

void L3GD20_write_reg(uint8_t addr, uint8_t value)
{
 SPI_slaveSelect_ctrl(select);
 Spi_sendrecv(addr);
 Spi_sendrecv(value);
SPI_slaveSelect_ctrl(deselect);
}

uint8_t L3GD20_read_reg(uint8_t addr)
{
 SPI_slaveSelect_ctrl(select);
 Spi_sendrecv(addr);
 uint8_t value = Spi_sendrecv(0xff);
 SPI_slaveSelect_ctrl(deselect);
 return value;
}
void L3GD20_read_reg_multi(uint8_t addr, void* data, int size)
{
 int i;
 uint8_t* buffer = (uint8_t*)data;

 SPI_slaveSelect_ctrl(select);
 Spi_sendrecv(addr);
 
 for (i = 0; i < size - 1; i++) 
     buffer[i] = Spi_sendrecv(0xff);
   SPI_slaveSelect_ctrl(deselect);
}