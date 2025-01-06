#include "usart.h"


static void NVIC_Configuration(void)
{
	NVIC_InitTypeDef  NVIC_InitStructure;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel =DEBUG_USART_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd =ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void USART_Config2(void)
{
 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	// 打开串口GPIO的时钟
	DEBUG_USART_GPIO_APBxClkCmd(DEBUG_USART_GPIO_CLK, ENABLE);
	
	// 打开串口外设的时钟
	DEBUG_USART_APBxClkCmd(DEBUG_USART_CLK, ENABLE);

	// 将USART Tx的GPIO配置为推挽复用模式
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  // 将USART Rx的GPIO配置为浮空输入模式
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);
	
	// 配置串口的工作参数
	// 配置波特率
	USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE;
	// 配置 针数据字长
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// 配置停止位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// 配置校验位
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	// 配置硬件流控制
	USART_InitStructure.USART_HardwareFlowControl = 
	USART_HardwareFlowControl_None;
	// 配置工作模式，收发一起
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// 完成串口的初始化配置
	USART_Init(DEBUG_USARTx, &USART_InitStructure);
	//中断配置
	NVIC_Configuration();
	//开启串口接收中断
	USART_ITConfig(DEBUG_USARTx,USART_IT_RXNE, ENABLE);
	//使能串口
	USART_Cmd(DEBUG_USARTx, ENABLE);

}
//发送一个字节
void Usart_SendByte(USART_TypeDef * pUSARTx,uint8_t date)
{
	 USART_SendData(pUSARTx,date);
	 while( USART_GetFlagStatus(pUSARTx,USART_FLAG_TXE)== RESET);
}

//发送一个16位的数据
void Usart_SendHalfWord(USART_TypeDef * pUSARTx,uint16_t date)
{
	uint16_t tmp_h;
	uint16_t tmp_l;
	tmp_h =date>>0x08;
	tmp_l =date & 0xff;
	Usart_SendByte(pUSARTx,tmp_h);
	Usart_SendByte(pUSARTx,tmp_l);
}

//发送一个8位的数组
void Usart_SendArray(USART_TypeDef * pUSARTx,uint8_t *arr,uint16_t num)
{ 
   while(num--)
	 {
		 Usart_SendByte( pUSARTx ,*arr++);
	 }
 while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TC)== RESET);
}

//发送字符串
void Usart_SendString( USART_TypeDef * pUSARTx, char *str)
{
 while( *str!='\0' )
 {
	 Usart_SendByte( pUSARTx, *str++); 
 }

 while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TC)== RESET);
}



//中断服务函数
void DEBUG_USART_IRQHandler(void)
{
    static  u8 i;
	u8 res,Frame_End_sta,Frame_Head_sta,str[10],len_r;
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) 
	{
		res=USART_ReceiveData(USART2);
        
        if((res==0xa5)&&(Frame_End_sta==0))//如果接收到数据是帧头并且帧尾标志位为0
            Frame_Head_sta=1;//此时接收到为帧头，帧头标志位置1
        if(res==0x5a&&Frame_Head_sta==1) //如果已经接收到帧头，且此时接收到数据为0x5A
            Frame_End_sta=1;//判定此次为帧尾
        
                
        if(Frame_Head_sta==1)//已经接收到帧头
        {
		str[i]=res;//把接收到的数据写进数组
		printf("%d/r",res);
        i++;
     
		if(res!=0x5a)//如果最后一个不是帧尾，说明接收错误
		{
			Frame_Head_sta=0;//帧头帧尾标志位全部置0，重新接收
			Frame_End_sta=0;
			i=0;
		}
		
			
        }
				
		if(str[0]!=0xa5)//如果第一个数据不是帧头，说明接收错误
		{
			i=0;
			Frame_Head_sta=0;//帧头帧尾标志位全部置0，重新接收
			Frame_End_sta=0;
		}
		
            
	}



	
	

}


