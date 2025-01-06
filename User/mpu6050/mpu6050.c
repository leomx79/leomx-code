/**
  ******************************************************************************
  * @file    bsp_led.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   MPU6050����
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� F103-MINI STM32 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */


#include "./mpu6050/mpu6050.h"
#include "./usart/bsp_usart.h"




#ifdef   soft_IIC
#include "./iic/bsp_soft_i2c.h"
static void I2Cx_Error(uint8_t Addr);
int MPU6050_ReadData(unsigned char slave_addr, unsigned char reg_addr, unsigned short len,
                             unsigned char *data_ptr)
{
	unsigned char i;

    slave_addr <<= 1;
	
	i2c_Start();
	i2c_SendByte(slave_addr);
    
	if(i2c_WaitAck())
        I2Cx_Error(slave_addr);
    
	i2c_SendByte(reg_addr);
	
    if(i2c_WaitAck())
        I2Cx_Error(slave_addr);
	
	i2c_Start();
	i2c_SendByte(slave_addr+1);
	
    if(i2c_WaitAck())
        I2Cx_Error(slave_addr);
	
	for(i=0;i<(len-1);i++){
		*data_ptr=i2c_ReadByte(1);
		data_ptr++;
	}
	*data_ptr=i2c_ReadByte(0);
	i2c_Stop();
    return 0;
}

/**
 * @brief  д�Ĵ����������ṩ���ϲ�Ľӿ�
 * @param  slave_addr: �ӻ���ַ
 * @param 	reg_addr:�Ĵ�����ַ
 * @param len��д��ĳ���
 *	@param data_ptr:ָ��Ҫд�������
 * @retval ����Ϊ0��������Ϊ��0
 */
int MPU6050_WriteReg(unsigned char slave_addr, unsigned char reg_addr, unsigned short len,
                              unsigned char *data_ptr)
{
    unsigned short i = 0;
    slave_addr <<= 1;
    i2c_Start();
    i2c_SendByte(slave_addr);
    
    if(i2c_WaitAck())
        I2Cx_Error(slave_addr);
    
    i2c_SendByte(reg_addr);
    
    if(i2c_WaitAck())
        I2Cx_Error(slave_addr);
    
    for(i=0;i<len;i++)
    {
        i2c_SendByte(data_ptr[i]);
        if(i2c_WaitAck())
            I2Cx_Error(slave_addr);
    }
    i2c_Stop();
    return 0;
}

/**
 * @brief  Manages error callback by re-initializing I2C.
 * @param  Addr: I2C Address
 * @retval None
 */
static void I2Cx_Error(uint8_t Addr)
{
    i2c_GPIO_Config();
    i2c_Stop();
    MPU6050_Init();
}

#else
#include "./iic/bsp_hard_i2c.h"



/**
 * @brief  ���Ĵ����������ṩ���ϲ�Ľӿ�
 * @param  slave_addr: �ӻ���ַ
 * @param 	reg_addr:�Ĵ�����ַ
 * @param len��Ҫ��ȡ�ĳ���
 *	@param data_ptr:ָ��Ҫ�洢���ݵ�ָ��
 * @retval ����Ϊ0��������Ϊ��0
 */
int MPU6050_ReadData(unsigned char slave_addr, unsigned char reg_addr, unsigned short len,
                             unsigned char *data_ptr)
{
    I2C_BufferRead(data_ptr,reg_addr,len);
    return 0;
}


/**
 * @brief  д�Ĵ����������ṩ���ϲ�Ľӿ�
 * @param  slave_addr: �ӻ���ַ
 * @param 	reg_addr:�Ĵ�����ַ
 * @param len��д��ĳ���
 *	@param data_ptr:ָ��Ҫд�������
 * @retval ����Ϊ0��������Ϊ��0
 */
 
int MPU6050_WriteReg(unsigned char slave_addr, unsigned char reg_addr, unsigned short len,
                              unsigned char *data_ptr)
{
    I2C_ByteWrite(data_ptr,reg_addr,len);
    return 0;
}



#endif



static void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    
    NVIC_InitStructure.NVIC_IRQChannel = MPU6050_INT_EXTI_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    
    NVIC_Init(&NVIC_InitStructure);
}

void MPU_INT_Init(void)
{
    GPIO_InitTypeDef Init;
    EXTI_InitTypeDef EXTI_InitStructure;
    
    NVIC_Configuration();
    
    RCC_APB2PeriphClockCmd(RCC_INT_GPIO_PORT, ENABLE);   //����GPIOAʱ��
    
    Init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    Init.GPIO_Pin  = MPU6050_INT_Pin;
    Init.GPIO_Speed = GPIO_Speed_50MHz;
    
    GPIO_Init(MPU6050_INT_GPIO_Port,&Init);
    
    /* ѡ��EXTI���ź�Դ */
    GPIO_EXTILineConfig(MPU6050_INT_EXTI_PORTSOURCE,MPU6050_INT_EXTI_PINSOURCE);
    
    EXTI_InitStructure.EXTI_Line = MPU6050_INT_EXTI_LINE;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    
    EXTI_Init(&EXTI_InitStructure);
    
}


/**
  * @brief   ��ʼ��MPU6050оƬ
  * @param   
  * @retval  
  */
void MPU6050_Init(void)
{
  int i=0,j=0;
  uint8_t reg[5]={0x00,0x07,0x06,0x01,0x18};
  
  
  //�ڳ�ʼ��֮ǰҪ��ʱһ��ʱ�䣬��û����ʱ����ϵ�����ϵ����ݿ��ܻ����
  for(i=0;i<1000;i++)
  {
    for(j=0;j<1000;j++)
    {
      ;
    }
  }
	MPU6050_WriteReg(MPU6050_SLAVE_ADDRESS,MPU6050_RA_PWR_MGMT_1,1, &reg[0]);	     //�������״̬
	MPU6050_WriteReg(MPU6050_SLAVE_ADDRESS,MPU6050_RA_SMPLRT_DIV ,1, &reg[1]);	    //�����ǲ�����
	MPU6050_WriteReg(MPU6050_SLAVE_ADDRESS,MPU6050_RA_CONFIG , 1,&reg[2]);	
	MPU6050_WriteReg(MPU6050_SLAVE_ADDRESS,MPU6050_RA_ACCEL_CONFIG ,1, &reg[3]);	  //���ü��ٶȴ�����������4Gģʽ
	MPU6050_WriteReg(MPU6050_SLAVE_ADDRESS,MPU6050_RA_GYRO_CONFIG,1, &reg[4]);     //�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
}

/**
  * @brief   ��ȡMPU6050��ID
  * @param   
  * @retval  
  */
uint8_t MPU6050ReadID(void)
{
	unsigned char Re = 0;
    MPU6050_ReadData(MPU6050_SLAVE_ADDRESS,MPU6050_RA_WHO_AM_I,1,&Re);
	if(Re != 0x68)
	{
		printf("MPU6050 dectected error!\r\n��ⲻ��MPU6050ģ�飬����ģ���뿪����Ľ���");
		return 0;
	}
	else
	{
		printf("MPU6050 ID = %d\r\n",Re);
		return 1;
	}
		
}
/**
  * @brief   ��ȡMPU6050�ļ��ٶ�����
  * @param   
  * @retval  
  */
void MPU6050ReadAcc(short *accData)
{
    u8 buf[6];
    MPU6050_ReadData(MPU6050_SLAVE_ADDRESS,MPU6050_ACC_OUT,6,buf);
    accData[0] = (buf[0] << 8) | buf[1];
    accData[1] = (buf[2] << 8) | buf[3];
    accData[2] = (buf[4] << 8) | buf[5];
}

/**
  * @brief   ��ȡMPU6050�ĽǼ��ٶ�����
  * @param   
  * @retval  
  */
void MPU6050ReadGyro(short *gyroData)
{
    u8 buf[6];
    MPU6050_ReadData(MPU6050_SLAVE_ADDRESS,MPU6050_GYRO_OUT,6,buf);
    gyroData[0] = (buf[0] << 8) | buf[1];
    gyroData[1] = (buf[2] << 8) | buf[3];
    gyroData[2] = (buf[4] << 8) | buf[5];
}


/**
  * @brief   ��ȡMPU6050��ԭʼ�¶�����
  * @param   
  * @retval  
  */
void MPU6050ReadTemp(short *tempData)
{
	u8 buf[2];
    MPU6050_ReadData(MPU6050_SLAVE_ADDRESS,MPU6050_RA_TEMP_OUT_H,2,buf);
    *tempData = (buf[0] << 8) | buf[1];
}


/**
  * @brief   ��ȡMPU6050���¶����ݣ�ת�������϶�
  * @param   
  * @retval  
  */
void MPU6050_ReturnTemp(float *Temperature)
{
	short temp3;
	u8 buf[2];
	
    MPU6050_ReadData(MPU6050_SLAVE_ADDRESS,MPU6050_RA_TEMP_OUT_H,2,buf);
  temp3= (buf[0] << 8) | buf[1];	
	*Temperature=((double) temp3/340.0)+36.53;

}
