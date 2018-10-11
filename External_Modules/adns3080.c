#include "adns3080.h"
#define ADNS_NCS_DESELECT GPIO_SetBits(GPIOE, GPIO_Pin_7)
#define ADNS_NCS_SELECT GPIO_ResetBits(GPIOE, GPIO_Pin_7)

unsigned char frame[ADNS3080_PIXELS_X * ADNS3080_PIXELS_Y];
int pid;
void adns3080_reset(void);
void adns3080_spi_config(void) {
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructt;
    SPI_InitTypeDef   SPI_InitStructure;
	// PB1 -> RST
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	  GPIO_InitStructt.GPIO_Pin =GPIO_Pin_1;
    GPIO_InitStructt.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructt.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructt.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructt.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructt);
	// PE7 -> NCS
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
		  GPIO_InitStructt.GPIO_Pin =GPIO_Pin_7;
    GPIO_InitStructt.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructt.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructt.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructt.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOE, &GPIO_InitStructt);
	
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
//	  GPIO_InitStructt.GPIO_Pin =GPIO_Pin_4;
//    GPIO_InitStructt.GPIO_Mode = GPIO_Mode_OUT;
//    GPIO_InitStructt.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructt.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructt.GPIO_PuPd = GPIO_PuPd_NOPULL;
//    GPIO_Init(GPIOC, &GPIO_InitStructt);
  /* SPI_MASTER configuration ------------------------------------------------*/
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  GPIO_InitStructt.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructt.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOA, &GPIO_InitStructt);
  
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1); // SCK
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1); // MISO
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1); // MOSI
  
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	// spi mode 3.
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_Init(SPI1, &SPI_InitStructure);
  SPI_Cmd(SPI1, ENABLE);
}

int8_t mousecam_init(void) {
	ADNS_NCS_DESELECT;
	adns3080_reset();
	pid = mousecam_read_reg(ADNS3080_PRODUCT_ID);
  if(pid != ADNS3080_PRODUCT_ID_VAL)
    return -1;
  // turn on sensitive mode
  mousecam_write_reg(ADNS3080_CONFIGURATION_BITS, 0x19);
  return 0;
}

void adns3080_reset(void)
{
	GPIO_SetBits(GPIOB, GPIO_Pin_1);
  delay_ms(1); // reset pulse >10us
	GPIO_ResetBits(GPIOB, GPIO_Pin_1);
  delay_ms(35); // 35ms from reset to functional
}

int mousecam_read_reg(int Address)
{
  int val;
	ADNS_NCS_SELECT;
	delay_us(2);
	SPI_transfer(Address);
	delay_us(83);
	val = SPI_transfer(0xff);
	ADNS_NCS_DESELECT;
	delay_us(1);
	return val;
}

// pdata must point to an array of size ADNS3080_PIXELS_X x ADNS3080_PIXELS_Y
// you must call mousecam_reset() after this if you want to go back to normal operation
int mousecam_frame_capture(unsigned char *pdata)
{
  mousecam_write_reg(ADNS3080_FRAME_CAPTURE,0x83);
  ADNS_NCS_SELECT;
  SPI_transfer(ADNS3080_PIXEL_BURST);
  delay_us(50);
  
  int pix;
  unsigned char started = 0;
  int count;
  int timeout = 0;
  int ret = 0;
  for(count = 0; count < ADNS3080_PIXELS_X * ADNS3080_PIXELS_Y; )
  {
    pix = SPI_transfer(0xff);
    delay_us(10);
    if(started==0)
    {
      if(pix&0x40)
        started = 1;
      else
      {
        timeout++;
        if(timeout==100)
        {
          ret = -1;
          break;
        }
      }
    }
    if(started==1)
    {
      pdata[count++] = (pix & 0x3f)<<2; // scale to normal grayscale byte range
    }
  }
	ADNS_NCS_DESELECT;
  delay_us(14);
  
  return ret;
}

void mousecam_write_reg(int reg, int val)
{  
	ADNS_NCS_SELECT;
	delay_us(5);
	SPI_transfer(reg| 0x80);
	SPI_transfer(val);
	ADNS_NCS_DESELECT;
	delay_us(5);
}
unsigned char SPI_transfer(unsigned char data){

	uint32_t val;
	//This function is just exactly the same as void 'SPI_I2S_SendData(SPI_TypeDef* SPIx, uint16_t Data)'
	SPI1->DR = data; // write data to be transmitted to the SPI data register
	while( !(SPI1->SR & SPI_I2S_FLAG_TXE) ); // wait until transmit complete
	while( !(SPI1->SR & SPI_I2S_FLAG_RXNE) ); // wait until receive complete
	while( SPI1->SR & SPI_I2S_FLAG_BSY ); // wait until SPI is not busy anymore
	return SPI1->DR; // return received data from SPI data register
}

void mousecam_read_motion(struct MD *p)
{
	ADNS_NCS_SELECT;
  SPI_transfer(ADNS3080_MOTION_BURST);
  delay_us(75);
  p->motion =  SPI_transfer(0xff);
  p->dx =  SPI_transfer(0xff);
  p->dy =  SPI_transfer(0xff);
  p->squal =  SPI_transfer(0xff);
  p->shutter =  SPI_transfer(0xff)<<8;
  p->shutter |=  SPI_transfer(0xff);
  p->max_pix =  SPI_transfer(0xff);
	ADNS_NCS_DESELECT;
  delay_us(5);
}

