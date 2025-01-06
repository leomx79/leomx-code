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

	// �򿪴���GPIO��ʱ��
	DEBUG_USART_GPIO_APBxClkCmd(DEBUG_USART_GPIO_CLK, ENABLE);
	
	// �򿪴��������ʱ��
	DEBUG_USART_APBxClkCmd(DEBUG_USART_CLK, ENABLE);

	// ��USART Tx��GPIO����Ϊ���츴��ģʽ
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  // ��USART Rx��GPIO����Ϊ��������ģʽ
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);
	
	// ���ô��ڵĹ�������
	// ���ò�����
	USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE;
	// ���� �������ֳ�
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// ����ֹͣλ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// ����У��λ
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	// ����Ӳ��������
	USART_InitStructure.USART_HardwareFlowControl = 
	USART_HardwareFlowControl_None;
	// ���ù���ģʽ���շ�һ��
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// ��ɴ��ڵĳ�ʼ������
	USART_Init(DEBUG_USARTx, &USART_InitStructure);
	//�ж�����
	NVIC_Configuration();
	//�������ڽ����ж�
	USART_ITConfig(DEBUG_USARTx,USART_IT_RXNE, ENABLE);
	//ʹ�ܴ���
	USART_Cmd(DEBUG_USARTx, ENABLE);

}
//����һ���ֽ�
void Usart_SendByte(USART_TypeDef * pUSARTx,uint8_t date)
{
	 USART_SendData(pUSARTx,date);
	 while( USART_GetFlagStatus(pUSARTx,USART_FLAG_TXE)== RESET);
}

//����һ��16λ������
void Usart_SendHalfWord(USART_TypeDef * pUSARTx,uint16_t date)
{
	uint16_t tmp_h;
	uint16_t tmp_l;
	tmp_h =date>>0x08;
	tmp_l =date & 0xff;
	Usart_SendByte(pUSARTx,tmp_h);
	Usart_SendByte(pUSARTx,tmp_l);
}

//����һ��8λ������
void Usart_SendArray(USART_TypeDef * pUSARTx,uint8_t *arr,uint16_t num)
{ 
   while(num--)
	 {
		 Usart_SendByte( pUSARTx ,*arr++);
	 }
 while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TC)== RESET);
}

//�����ַ���
void Usart_SendString( USART_TypeDef * pUSARTx, char *str)
{
 while( *str!='\0' )
 {
	 Usart_SendByte( pUSARTx, *str++); 
 }

 while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TC)== RESET);
}



//�жϷ�����
void DEBUG_USART_IRQHandler(void)
{
    static  u8 i;
	u8 res,Frame_End_sta,Frame_Head_sta,str[10],len_r;
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) 
	{
		res=USART_ReceiveData(USART2);
        
        if((res==0xa5)&&(Frame_End_sta==0))//������յ�������֡ͷ����֡β��־λΪ0
            Frame_Head_sta=1;//��ʱ���յ�Ϊ֡ͷ��֡ͷ��־λ��1
        if(res==0x5a&&Frame_Head_sta==1) //����Ѿ����յ�֡ͷ���Ҵ�ʱ���յ�����Ϊ0x5A
            Frame_End_sta=1;//�ж��˴�Ϊ֡β
        
                
        if(Frame_Head_sta==1)//�Ѿ����յ�֡ͷ
        {
		str[i]=res;//�ѽ��յ�������д������
		printf("%d/r",res);
        i++;
     
		if(res!=0x5a)//������һ������֡β��˵�����մ���
		{
			Frame_Head_sta=0;//֡ͷ֡β��־λȫ����0�����½���
			Frame_End_sta=0;
			i=0;
		}
		
			
        }
				
		if(str[0]!=0xa5)//�����һ�����ݲ���֡ͷ��˵�����մ���
		{
			i=0;
			Frame_Head_sta=0;//֡ͷ֡β��־λȫ����0�����½���
			Frame_End_sta=0;
		}
		
            
	}



	
	

}


