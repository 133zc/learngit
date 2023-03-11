/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "pid.h"
#include "Remote_Control.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
PID_TypeDef motor_pid[4];
int32_t set_spd = 0;
int32_t set_angle=0;
static int key_sta = 0;
int speed_step_sign = +1;
float    pos_output[4];
uint16_t TIM_COUNT[2];
#define SpeedStep 500

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void Key_Scan(){
		
		if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin) == GPIO_PIN_RESET){
						
			if(key_sta == 0){
					
				key_sta = 1;
				
				set_spd += SpeedStep*speed_step_sign;
				
				if(set_spd>8000)
				{
					speed_step_sign = -1;
				}
				
				if(set_spd<=0){
						
					set_spd = 0;
					speed_step_sign = 1;
					
				}
					
			}
			
		}else{
			
			key_sta = 0;
		
		}
	
}





/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */
  my_can_filter_init_recv_all(&hcan1);     //ÅäÖÃCAN¹ýÂËÆ÷
  HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);   //Æô¶¯CAN½ÓÊÕÖÐ¶Ï
  HAL_UART_Receive_IT_IDLE(&huart1,UART_Buffer,100);   //Æô¶¯´®¿Ú½ÓÊÕ

  HAL_TIM_IC_Start_DMA(&htim1,TIM_CHANNEL_2,(uint32_t *)TIM_COUNT,2);
	
	/*< ³õÊ¼»¯PID²ÎÊý >*/
//  for(int i=0; i<4; i++)
//  {	

//    pid_init(&motor_pid[i]);
//    motor_pid[i].f_param_init(&motor_pid[i],PID_Speed,16384,5000,10,0,8000,0,1.5,1.1,0);
//    
//  }
//set_spd =4000;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {	
		
    if(HAL_GetTick() - Latest_Remote_Control_Pack_Time >500){   //Èç¹û500ms¶¼Ã»ÓÐÊÕµ½Ò£¿ØÆ÷Êý¾Ý£¬Ö¤Ã÷Ò£¿ØÆ÷¿ÉÄÜÒÑ¾­ÀëÏß£¬ÇÐ»»µ½°´¼ü¿ØÖÆÄ£Ê½¡£
      Key_Scan();                                                                                                                                                                                                                                                                                                                                                                                                                                                           
    }else{		
			
				if(remote_control.switch_right!=2)//½øÈëËÙ¶È¿ØÖÆÄ£Ê½
			
      {			
					  for(int i=0; i<4; i++)
					{	
					pid_init(&motor_pid[i]);
					motor_pid[i].f_param_init(&motor_pid[i],PID_Speed,16384,5000,10,0,8000,0,1.5,1.1,0);
					}
					
					set_spd = remote_control.ch4*8000/660;
					for(int i=0; i<4; i++)
					{	
							motor_pid[i].target = set_spd; 																							
							motor_pid[i].f_cal_pid(&motor_pid[i],moto_chassis[i].speed_rpm);    //¸ù¾ÝÉè¶¨Öµ½øÐÐPID¼ÆËã¡£
					}

					
			if(remote_control.switch_left==1&&remote_control.switch_right==1)//×óÓÒ²¦¸Ë¿ª¹ØÎ»ÖÃÄ£Ê½Ñ¡Ôñ,ÉÏ¡¢ÖÐ¡¢ÏÂ·Ö±ð¶ÔÓ¦1¡¢3¡¢2
			{
            set_moto_current(&hcan1, 
												motor_pid[0].output,   //½«PIDµÄ¼ÆËã½á¹ûÍ¨¹ýCAN·¢ËÍµ½µç»ú
                        motor_pid[1].output,
                        motor_pid[2].output,
                        motor_pid[3].output);				
						HAL_Delay(10); 
			}
			if(remote_control.switch_left==3&&remote_control.switch_right==1)
			{
            set_moto_current(&hcan1, motor_pid[0].output,0,0,0);//µç»ú1	
						HAL_Delay(10); 		//½«PIDµÄ¼ÆËã½á¹ûÍ¨¹ýCAN·¢ËÍµ½µç»ú                                     			
			}

			if(remote_control.switch_left==2&&remote_control.switch_right==1)
			{
            set_moto_current(&hcan1, 0,motor_pid[1].output,0,0);//µç»ú2	 
						HAL_Delay(10); 		//½«PIDµÄ¼ÆËã½á¹ûÍ¨¹ýCAN·¢ËÍµ½µç»
			}


			if(remote_control.switch_right==3&&remote_control.switch_left==2)
			{
            set_moto_current(&hcan1, 0,0,motor_pid[2].output,0);//µç»ú3	 
						HAL_Delay(10); 		//½«PIDµÄ¼ÆËã½á¹ûÍ¨¹ýCAN·¢ËÍµ½µç»
			}

						if(remote_control.switch_right==3&&remote_control.switch_left==3)
       {    
						set_moto_current(&hcan1, 0,0,0,motor_pid[3].output);//µç»ú4	
						HAL_Delay(10); 		//½«PIDµÄ¼ÆËã½á¹ûÍ¨¹ýCAN·¢ËÍµ½µç»
			 }
		 }
			      if(remote_control.switch_right==2)//½øÈë2006¡¢3508Î»ÖÃ»·¿ØÖÆÄ£Ê½
			 {
						HAL_Delay(1);
						if(remote_control.switch_right==2&&remote_control.switch_left==1)
						{
								set_angle=8192*40*1;//ÉèÖÃÐý×ª½Ç¶È£»×ª×Ó½Ç¶È*¼õËÙ±È*×ªÖáÐý×ªÈ¦Êý 
						}else{set_angle=8192*40*0;}							
								for(int i=0; i<4; i++)
									{	
									pid_init(&motor_pid[i]);
									motor_pid[i].f_param_init(&motor_pid[i],PID_Position,10000,0,50,0,0,0,0.25,0,0.1);//Î»ÖÃ»·pid¸³Öµ
									}
								for(int i=0; i<4; i++)
									{	
									motor_pid[i].target = set_angle; 																							
									motor_pid[i].f_cal_pid(&motor_pid[i],moto_chassis[i].total_angle);
									pos_output[i]=motor_pid[i].f_cal_pid(&motor_pid[i],moto_chassis[i].total_angle);//¸ù¾ÝÉè¶¨Öµ½øÐÐPID¼ÆËã¡£
									}                                    
								for(int i=0; i<4; i++)
									{	
									pid_init(&motor_pid[i]);
									motor_pid[i].f_param_init(&motor_pid[i],PID_Speed,8000,5000,1000,0,800,0,2,0.1,0);//ËÙ¶È»·pid¸³Öµ
									}
								for(int i=0; i<4; i++)
									{	
									motor_pid[i].target = pos_output[i];																				
									motor_pid[i].f_cal_pid(&motor_pid[i],moto_chassis[i].speed_rpm);//¸ù¾ÝÉè¶¨Öµ½øÐÐPID¼ÆËã¡£
									} 
									set_moto_current(&hcan1, 
												motor_pid[0].output,   //½«PIDµÄ¼ÆËã½á¹ûÍ¨¹ýCAN·¢ËÍµ½µç»ú
                        motor_pid[1].output,
                        motor_pid[2].output,
                        motor_pid[3].output);				
						      HAL_Delay(10); 
						}


			 }
		}

//    for(int i=0; i<4; i++)
//    {	
//      motor_pid[i].target = set_spd; 																							
//      motor_pid[i].f_cal_pid(&motor_pid[i],moto_chassis[i].speed_rpm);    //¸ù¾ÝÉè¶¨Öµ½øÐÐPID¼ÆËã¡£
//    }
//		 for(int i=2; i<4; i++)
//    {	
//      motor_pid[i].target = -set_spd; 																							
//      motor_pid[i].f_cal_pid(&motor_pid[i],moto_chassis[i].speed_rpm);    //¸ù¾ÝÉè¶¨Öµ½øÐÐPID¼ÆËã¡£
//    }
//    set_moto_current(&hcan1, motor_pid[0].output,   //½«PIDµÄ¼ÆËã½á¹ûÍ¨¹ýCAN·¢ËÍµ½µç»ú
//                        motor_pid[1].output,
//                            motor_pid[2].output,
//                        motor_pid[3].output);
    
//    HAL_Delay(10);      //PID¿ØÖÆÆµÂÊ100HZ  
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */


/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
