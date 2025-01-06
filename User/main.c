/**
 ******************************************************************************
 * @file    main.c
 * @author  fire
 * @version V1.0
 * @date    2024-xx-xx
 * @brief   MPU6050
 ******************************************************************************
 * @attention
 *
 * 实验平台:野火小智 STM32F103C8 核心板 
 * 论坛    :http://www.firebbs.cn
 * 淘宝    :https://fire-stm32.taobao.com
 *
 ******************************************************************************
 */

#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "systick/bsp_SysTick.h"
#include "led/bsp_led.h"
#include "usart/bsp_usart.h"
#include "mpu6050/mpu6050.h"
#include "pwm.h"
#include "usart.h"

/* mpu6050 DMP库  begin */
#include "eMPL_outputs.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "log.h"
#include "mltypes.h"
#include "mpu.h"
#include "packet.h"
#include "mpu6050_SL.h"
#include "pid.h"


#ifdef   soft_IIC
#include "iic/bsp_soft_i2c.h"
#else
#include "iic/bsp_hard_i2c.h"
#endif

/* MPU6050数据 */
short Accel[3];
short Gyro[3];
float Temp;
int goat_angle,roll,pitch,yaw;
static int first=1000000,second=1000000,three=1000000,four=1000000;

//微秒级的延时
void delay_us(uint32_t delay_us)
{    
  volatile unsigned int num;
  volatile unsigned int t;
 
  
  for (num = 0; num < delay_us; num++)
  {
    t = 11;
    while (t != 0)
    {
      t--;
    }
  }
}
//毫秒级的延时
void delay_ms(uint32_t delay_ms)
{    
  volatile unsigned int num;
  for (num = 0; num < delay_ms; num++)
  {
    delay_us(1000);
  }
}




void  getdata(void);
/**
  * @brief  主函数
  * @param  无  
  * @retval 无
  */
int main(void)
{
  
    
    /* 配置SysTick定时器和中断 */
    SysTick_Init(); //配置 SysTick 为 1ms 中断一次，在中断里读取传感器数据
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; //启动定时器
    PWM_Init();
    /* LED 端口初始化 */
    LED_GPIO_Config();
    /* 串口1通信初始化 */
    USART_Config();
	USART_Config2();
	printf("\r\n 欢迎使用野火  STM32 开发板 \r\n");
    printf("\r\n 这是一个MPU6050串口显示欧拉角测试例程\r\n");
    
    //I2C初始化
    #ifdef soft_IIC
        i2c_GPIO_Config();
    #else
        MPU_I2C_Config();
    #endif
    
    //初始化INT引脚
    MPU_INT_Init();
    
    //初始化MPU6050
    MPU6050_mpu_init();
    MPU6050_mpl_init();
    MPU6050_config();

	CascadePID cascade_pid;
	PID sudupid;
	CascadePID_Init(&cascade_pid, 1.0, 0.1, 0.01, 1.5, 0.2, 0.02);
	PID_Init(&sudupid, 1, 0.1, 0.01);
	
	double output_roll,output_pitch;
	
	double primary_setpoint = 10.0;
	double secondary_setpoint = 0;
	double measured_value_roll = roll;
	double measured_value_pitch = pitch;
	
	int measured_value_sudu_first = first;
	int measured_value_sudu_second = second;
	int measured_value_sudu_three = three;
	int measured_value_sudu_four = four;
	
	
	double setpoint_sudu_three = measured_value_sudu_three+output_roll;
	
	
	double setpoint_sudu_first = measured_value_sudu_first-output_roll;
	
	
	double setpoint_sudu_second = measured_value_sudu_second+output_pitch;
	
	
	double setpoint_sudu_four = measured_value_sudu_four-output_pitch;
	
	
	
	
    //检测MPU6050
    if(MPU6050ReadID() == 0)
    {
        printf("\r\n没有检测到MPU6050传感器！\r\n");
        LED1_ON; 
        while(1);	
    }
    
    mdelay(500);
    while (1)
    {
        unsigned long sensor_timestamp;
        if (!hal.sensors || !hal.new_gyro)
        {
            continue;
        }
		
//	    getdata();
		//roll的PID
	    output_roll = CascadePID_Compute(&cascade_pid, primary_setpoint, secondary_setpoint, measured_value_roll);
		
		//速度环PID
		measured_value_sudu_three = PID_Compute(&sudupid,setpoint_sudu_three,measured_value_sudu_three);
		measured_value_sudu_first = PID_Compute(&sudupid,setpoint_sudu_first,measured_value_sudu_first);
		
		//电机输出
		TIM_SetCompare1(TIM3,measured_value_sudu_first);
		TIM_SetCompare3(TIM3,measured_value_sudu_three);
		
		
		
		//Pitch的PID
		output_pitch = CascadePID_Compute(&cascade_pid, primary_setpoint, secondary_setpoint, measured_value_pitch);
		
		//速度环的PID
		measured_value_sudu_second = PID_Compute(&sudupid,setpoint_sudu_second,measured_value_sudu_second);
		measured_value_sudu_four = PID_Compute(&sudupid,setpoint_sudu_four,measured_value_sudu_four);
		
		//电机输出
		TIM_SetCompare2(TIM3,measured_value_sudu_second);
		TIM_SetCompare4(TIM3,measured_value_sudu_four);



    }
	
}


void  getdata(void)
{
        unsigned long sensor_timestamp;
				unsigned char new_temp = 0;
				unsigned long timestamp;
				int new_data = 0;
        /* 接收到INT传来的中断信息后继续往下 */
       
        /* 每过500ms读取一次温度 */
        get_tick_count(&timestamp);
        if (timestamp > hal.next_temp_ms)
        {
            hal.next_temp_ms = timestamp + TEMP_READ_MS;
            new_temp = 1;
        }

        /* 接收到新数据 并且 开启DMP */
        if (hal.new_gyro && hal.dmp_on)
        {
            short gyro[3], accel_short[3], sensors;
            unsigned char more;
            long accel[3], quat[4], temperature;

            dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);

            if (!more)
                hal.new_gyro = 0;

            if (sensors & INV_WXYZ_QUAT)
            {
                inv_build_quat(quat, 0, sensor_timestamp);
                new_data = 1;
            }

            if (sensors & INV_XYZ_GYRO)
            {
                inv_build_gyro(gyro, sensor_timestamp);
                new_data = 1;
                if (new_temp)
                {
                    new_temp = 0;

                    mpu_get_temperature(&temperature, &sensor_timestamp);
                    inv_build_temp(temperature, sensor_timestamp);
                }
            }
            

            if (sensors & INV_XYZ_ACCEL)
            {
                accel[0] = (long)accel_short[0];
                accel[1] = (long)accel_short[1];
                accel[2] = (long)accel_short[2];
                inv_build_accel(accel, 0, sensor_timestamp);
                new_data = 1;
            }
        }

        if (new_data)
        {
            long data[9];
            int8_t accuracy;

            if (inv_execute_on_data())
            {
                printf("数据错误\n");
            }

            float float_data[3];
            if (inv_get_sensor_type_euler(data, &accuracy, (inv_time_t *)&timestamp))
            {
                pitch=float_data[0] = data[0] * 1.0 / (1 << 16);//俯仰角
                roll=float_data[1] = data[1] * 1.0 / (1 << 16);//横滚角
                yaw=float_data[2] = data[2] * 1.0 / (1 << 16);//航向角
                printf("\r\n欧拉角(rad)\t\t: %7.5f\t %7.5f\t %7.5f\t", float_data[0], float_data[1], float_data[2]);
				
            }

        }

  

}





/*********************************************END OF FILE**********************/
