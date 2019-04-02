#include "stm32f4xx.h"
#include "LSM303DLHC.h"

void I2C_Write (unsigned char c) 
{

I2C1->DR = c;
while (!(I2C1->SR1 & (1<<7)));	// czekaj na TxE 

} 

void I2C_Start() 
{
uint32_t dummy;
I2C1->CR1 |= I2C_CR1_START;				// request a start
while((I2C1->SR1 & I2C_SR1_SB) == RESET);		// wait for start to finish
dummy = I2C1->SR1;
}	

void I2C_Stop (void) 
{
I2C1->CR1 |= I2C_CR1_STOP;//I2C Stop
while (I2C1->SR2 & 0x0002);//czekaj na flage BUSY
}	

unsigned char I2C_Read (int ack) 
{
if (ack) I2C1->CR1 |= 0x0400;//jak chce wczytac wiecej bitów to trzeba ustawic flage ACK
else I2C1->CR1 &= ~0x0400;//a jak nie to wyczyscic ACK
while (!(I2C1->SR1 & 0x00000040));// Czekaj dopóki RxNE sie wyzeruje
return (I2C1->DR);
}

void I2C_Addr (unsigned char adr) 
{
char res;
I2C1->DR = adr | 0;//wyslanie adresu urzadzenia
while (!(I2C1->SR1 & 0x0002));//Czekaj na flage wyslania adresu ADDR
res = (I2C1->SR1);
res = (I2C1->SR2);//odczytanie z rejestru czysci flage ADDR
}

void lsm_set_reg(uint8_t reg)
{
 I2C_Start();
 I2C_Addr (LSM303D_ADDR);
 I2C_Write (reg);
 }

void lsm_write(uint8_t reg, uint8_t data)
{
 lsm_set_reg(reg);
 I2C_Write (data);
 I2C_Stop();
}

void lsm_read(uint8_t reg, void* data, int size)
{
 int i;
 uint8_t* buffer = (uint8_t*)data;
 
 lsm_set_reg(reg);
 
 I2C_Start(); 
 I2C_Addr (LSM303D_ADDR|1);
 for (i = 0; i < size - 1; i++) 
     buffer[i] = I2C_Read (1);
  buffer[i] = I2C_Read (0);
  I2C_Stop();
}
 