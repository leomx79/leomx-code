#ifndef __BSP_IIC_H
#define	__BSP_IIC_H

#include "stm32f10x.h"

#define  MPU_I2C    I2C2

    /**I2C2 GPIO Configuration
    PB10     ------> I2C1_SCL
    PB11     ------> I2C1_SDA
    */
//�궨��IIC��GPIO
#define  RCC_HARD_GPIO_PORT      RCC_APB2Periph_GPIOB
#define  RCC_HARD_I2C_PORT       RCC_APB1Periph_I2C2
#define  I2C_SCL_PIN             GPIO_Pin_10
#define  I2C_SDA_PIN             GPIO_Pin_11
#define  I2C_GPIO_PORT           GPIOB

/* �����ַֻҪ��STM32��ҵ�I2C������ַ��һ������ */
#define I2Cx_OWN_ADDRESS7      0X0A 




void MPU_I2C_Config(void);
void I2C_ByteWrite(uint8_t* pBuffer, uint8_t WriteAddr,uint8_t len);
void I2C_BufferRead(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);
#endif /* __BSP_IIC_H */
