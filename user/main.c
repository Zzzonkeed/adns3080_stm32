#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "my_delay.h"
#include "adns3080.h"
//#include "my_delay.c"

//void adns_spi_config(void);
void usart_init(uint32_t BaudRate);
// 0.1ms delay
void delay_01ms(uint16_t period){
	
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
  	TIM6->PSC = 8399;		// clk = SystemCoreClock /2 /(PSC+1) = 10KHz
  	TIM6->ARR = period-1;
  	TIM6->CNT = 0;
  	TIM6->EGR = 1;		// update registers;
  	TIM6->SR  = 0;		// clear overflow flag
  	TIM6->CR1 = 1;		// enable Timer6
  	while (!TIM6->SR);
  	TIM6->CR1 = 0;		// stop Timer6
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, DISABLE);
}

NVIC_InitTypeDef  NVIC_InitStruct;
GPIO_InitTypeDef GPIO_InitStruct;
USART_InitTypeDef USART_InitStruct;
GPIO_InitTypeDef  GPIO_InitStruct;
//uint32_t count;
struct MD adns_md;
int adns_val;
int adns_status=5, adns_init_cnt = 0;
int main(void)
{
 //   usart_init(19200);
//    delay_01ms(1000);
	   delay_init(168);
	
//    printf("--New Reset--\r\n");
    GPIO_InitTypeDef  GPIO_InitStruct;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    /* Configure PD12, PD13 in output pushpull mode */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15 ;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStruct);
    adns3080_spi_config();
	  delay_ms(500);
	  while (adns_status != 0) {
    adns_status =	mousecam_init();
			adns_init_cnt++;
			if (adns_init_cnt==10) {
				NVIC_SystemReset();
			}
			delay_ms(200);
		}
	 // delay_ms(500);
    while (1) {
	//	printf("Hello worlddd %d \r\n", count);
//		count+=5;
			 
//	delay_ms(100);
//	GPIO_SetBits(GPIOD,GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
//	delay_ms(100);
//	GPIO_ResetBits(GPIOD,GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
			
			  adns_val = mousecam_read_reg(ADNS3080_PIXEL_SUM);
////  
  mousecam_read_motion(&adns_md);
			delay_ms(4);
			adns_md.x+=adns_md.dx;
			adns_md.y+=adns_md.dy;
			
			
//  adns_val = mousecam_read_reg(ADNS3080_PIXEL_SUM);
//  
//  mousecam_read_motion(&md);
  // md.squal is ******** squirrel number = surface quality
 // for(int i=0; i<md.squal/4; i++)
//    Serial.print('*');
//  Serial.print(' ');
  // avarage brightness value
//  Serial.print((adns_val*100)/351);
////  Serial.print(' ');
//  Serial.print(md.shutter); Serial.print(" (");
//  // dx dy are motion vector
//  Serial.print((int)md.dx); Serial.print(',');
//  Serial.print((int)md.dy); Serial.println(')');
  // Serial.println(md.max_pix);
 // delay_ms(100);
  
	
    }
}
void usart_init(uint32_t BaudRate) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Pin=GPIO_Pin_2|GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);//UART Tx pin
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);//UART Rx pin

    USART_InitStruct.USART_BaudRate=BaudRate;
    USART_InitStruct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;
    USART_InitStruct.USART_Parity=USART_Parity_No;
    USART_InitStruct.USART_StopBits=USART_StopBits_1;
    USART_InitStruct.USART_WordLength=USART_WordLength_8b;
    USART_Init(USART2, &USART_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 10;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART2, ENABLE);
}
