/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"

#include "robot.h"
uint8_t flag=0;
uint8_t material_type=0;
uint8_t base_position_now = 0;
uint16_t AD_RES=0;
uint32_t AD1=0;
uint32_t AD2=0;
uint32_t AD3=0;
uint32_t AD4=0;
uint8_t i=0;
uint16_t j=0;
uint8_t AD[4];
uint8_t pwm1 = 0,pwm2 = 0,pwm3 = 0,pwm4 = 0;
uint8_t series = 10;
float time_now=0,time_previous=0;
float Ts=0.0;
float out1,out2=0.0,out3=0.0,out4=0.0;
float old1=0.0,old2=0.0,old3=0.0,old4=0.0;
float error1=0.0,error2=0.0,error3=0.0,error4=0.0;
float theta1=0.0,theta2=0.0,theta3=0.0,theta4=0.0;
float velocity1=0.0,velocity2=0.0,velocity3=0.0,velocity4=0.0;
float velocity_ref1=0.0,velocity_ref2=0.0,velocity_ref3=0.0,velocity_ref4=0.0;
char direc[4];
char conveyor[1];
char end_effector[1];




uint8_t x =0; // counter as a timer

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void arm_up(void)
{
	error2 = theta2_up - theta2;
	velocity_ref2 = P(error2,kp12);
	if(velocity_ref2 != 0)
	{
		out2 = PI(velocity_ref2 - velocity2, kp2, ki2, Ts);

	      if(out2 < 0)
	      {
	    	  direc[1] = 'f';
	    	  pwm2 = (uint8_t)(-out2);
	      }
	      else
	      {
	    	  direc[1] = 'b';
	    	  pwm2 = (uint8_t)out2;
	      }
	}
	else // velocity_ref2 = 0  (motor2 off)
	{
		//motor2 off
		direc[1] = 's';
		pwm2 = 0;


	}
	error3 = theta3_up - theta3;
			error4 = theta4_up - theta4;
			velocity_ref3 = P(error3, kp13);
			velocity_ref4 = P(error4, kp14);
			if(velocity_ref3 != 0)
			{
				out3 = PI(velocity_ref3 - velocity3, kp3, ki3, Ts);
			      if(out3 < 0)
			      {
			    	  direc[2] = 'f';
			    	  pwm3 = (uint8_t)(-out3);
			      }
			      else
			      {
			    	  direc[2] = 'b';
			    	  pwm3 = (uint8_t)out3;
			      }

			}
			else // velocity_ref3 = 0 (motor3 off)
			{
				direc[2] = 's';
				pwm3 = 0;
			}

			if(velocity_ref4 != 0)
			{
				 out4 = PI(velocity_ref4 - velocity4, kp4, ki4, Ts);
				  if(out4 < 0)
				  {
				      direc[3] = 'f';
				      pwm4 = (uint8_t)(-out4);
				  }
				  else
				  {
				      direc[3] = 'b';
				      pwm4 = (uint8_t)out4;
				  }
			}
			else // velocity_ref4 = 0 (motor4 off)
			{
				direc[3] = 's';
				pwm4 = 0;
			}

			if(velocity_ref3 == 0 && velocity_ref4 == 0 && velocity_ref2 == 0)
			{
				flag = arm_is_up;
			}



	// saturation limits
	if(pwm2>100)
		pwm2=100;
	else if(pwm2<15 && pwm2>2 && direc[1] == 'b')//up
		pwm2=15;
	else if(pwm2<10 && pwm2>2 && direc[1] == 'f')// down
		pwm2=5;
	else if(pwm2 <= 2) {
		direc[1] = 's';
		pwm2=0;
	}

	if(pwm3>50 && direc[2] == 'b'){
		pwm3=50;
	}
	else if(pwm3 > 15 && direc[2] == 'f'){
		pwm3=5;
	}
	else if(pwm3<10 && pwm3>2 && direc[2] == 'f')
	{
		pwm3=5;
	}
	else if(pwm3<18 && pwm3>2 && direc[2] == 'b')
	{
		pwm3=18;
	}
	else if(pwm3 <= 2) {
		direc[2] = 's';
		pwm3=0;
	}

	if(pwm4>40 && direc[3] == 'b') // max speed up
		pwm4=40;
	if(pwm4>10 && direc[3] == 'f')// max speed down
		pwm4=10;
	else if(pwm4<18 && pwm4>3 && direc[3] == 'b')// min  speed up
		pwm4=18;
	else if(pwm4<5 && pwm4>3 && direc[3] == 'f')// min speed down
		pwm4=5;
	else if(pwm4 <= 3)
	{
		direc[3] = 's';
		pwm4=0;
	}

}


void arm_down(void)
{
	error3 = theta3_down - theta3; // من اجل ارتياب قراءة الADC
	error4 = theta4_down - theta4 ;
	velocity_ref3 = P(error3, kp13);
	velocity_ref4 = P(error4, kp14);
	if(velocity_ref3 != 0)
	{
	    out3 = PI(velocity_ref3 - velocity3, kp3, ki3, Ts);
	      if(out3 < 0)
	      {
	          direc[2] = 'f';
	          pwm3 = (uint8_t)(-out3);
	      }
	      else
	      {
	    	  direc[2] = 'b';
	    	  pwm3 = (uint8_t)out3;
	      }

	 }
	 else // velocity_ref3 = 0 (motor3 off)
	 {
		  direc[2] = 's';
		  pwm3 = 0;
	 }

	 if(velocity_ref4 != 0)
	 {
		  out4 = PI(velocity_ref4 - velocity4, kp4, ki4, Ts);
		  if(out4 < 0)
		  {
			  direc[3] = 'f';
			  pwm4 = (uint8_t)(-out4);
		  }
		  else
		  {
			  direc[3] = 'b';
		      pwm4 = (uint8_t)out4;
		  }
	 }
	 else  // velocity_ref4 = 0 (motor4 off)
	 {
		  direc[3] = 's';
		  pwm4 = 0;
	 }
	 error2 = theta2_down - theta2;
	 	      velocity_ref2 = P(error2,kp12);
	 	   	  if(velocity_ref2 != 0)
	 		  {
	 			  out2 = PI(velocity_ref2 - velocity2, kp2, ki2, Ts);

	 			  if(out2 < 0)
	 			  {
	 			      direc[1] = 'f';
	 			      pwm2 = (uint8_t)(-out2);
	 			  }
	 			  else
	 			  {
	 			      direc[1] = 'b';
	 			      pwm2 = (uint8_t)out2;
	 			  }
	 		  }
	 	   	  else
	 	   	  {
	 			  direc[1] = 's';
	 			  pwm2 = 0;

	 	   	  }
	 if(velocity_ref3 == 0 && velocity_ref4 == 0 && velocity_ref2 ==0)
	 {
		 flag = arm_is_down;
	 }

	// saturation limits
	if(pwm2>100)
		pwm2=100;
	else if(pwm2<15 && pwm2>2 && direc[1] == 'b')//up
		pwm2=15;
	else if(pwm2<15 && pwm2>2 && direc[1] == 'f')// down
		pwm2=5;
	else if(pwm2 <= 2) {
		direc[1] = 's';
		pwm2=0;
	}

	if(pwm3>9 && direc[2] == 'f'){
		pwm3 = 9;
	}
	else if(pwm3>50 && direc[2] == 'b')
	{
		pwm3 = 50;
	}
	else if(pwm3<6 && pwm3>3 && direc[2] == 'f')
	{
		pwm3=6;
	}
	else if(pwm3<15 && pwm3>3 && direc[2] == 'b')
	{
		pwm3=15;
	}
	else if(pwm3 <= 3) {
		direc[2] = 's';
		pwm3=0;
	}

	if(pwm4>30 && direc[3] == 'b') // max speed up
		pwm4=30;
	if(pwm4>8 && direc[3] == 'f')// max speed down
		pwm4=8;
	else if(pwm4<25 && pwm4>3 && direc[3] == 'b')// min  speed up
		pwm4=25;
	else if(pwm4<5 && pwm4>3 && direc[3] == 'f')// min speed down
		pwm4=5;
	else if(pwm4 <= 3)
	{
		direc[3] = 's';
		pwm4=0;
	}
}
void rot_base(float teta)
{
	error1 = teta - theta1;
	velocity_ref1=P(error1,kp11);
	if(velocity_ref1 != 0)
	{
	      out1 = PI(velocity_ref1 - velocity1, kp1, ki1, Ts);

	      if(out1 > 0)
	      {
	    	  direc[0] = 'b';
	    	  pwm1 = (uint8_t)out1;
	      }
	      else
	      {
	    	  direc[0] = 'f';
	    	  pwm1 = (uint8_t)(-out1);
	      }

	}
	else
	{
		  direc[0] = 's';
		  pwm1 = 0;
		  base_position_now = teta;
	}


	// Saturation Limits...
	  if(pwm1>100)
		  pwm1=100;
	  else if(pwm1<15 && pwm1>1)
		  pwm1 = 12;
	  else if(pwm1 <= 1)
	  {
		  direc[0] = 's';
		  pwm1 = 0;
		  base_position_now = (uint8_t)teta;
	  }
}

void arm_down2(void){
	error4 = theta4_down - theta4 ;
	velocity_ref4 = P(error4, kp14);
	if(velocity_ref4 != 0)
		 {
			  out4 = PI(velocity_ref4 - velocity4, kp4, ki4, Ts);
			  if(out4 < 0)
			  {
				  direc[3] = 'f';
				  pwm4 = (uint8_t)(-out4);
			  }
			  else
			  {
				  direc[3] = 'b';
			      pwm4 = (uint8_t)out4;
			  }
		 }
		 else  // velocity_ref4 = 0 (motor4 off)
		 {
			  direc[3] = 's';
			  pwm4 = 0;
			  error2 = theta2_down - theta2;
			  	      velocity_ref2 = P(error2,kp12);
			  	   	  if(velocity_ref2 != 0)
			  		  {
			  			  out2 = PI(velocity_ref2 - velocity2, kp2, ki2, Ts);

			  			  if(out2 < 0)
			  			  {
			  			      direc[1] = 'f';
			  			      pwm2 = (uint8_t)(-out2);
			  			  }
			  			  else
			  			  {
			  			      direc[1] = 'b';
			  			      pwm2 = (uint8_t)out2;
			  			  }
			  		  }
			  	   	  else
			  	   	  {
			  			  direc[1] = 's';
			  			  pwm2 = 0;
			  			  flag = arm_is_down;
			  	   	  }
		 }
	//saturation limits
	if(pwm2>100)
		pwm2=100;
	else if(pwm2<15 && pwm2>2 && direc[1] == 'b')//up
		pwm2=15;
	else if(pwm2<15 && pwm2>2 && direc[1] == 'f')// down
		pwm2=5;
	else if(pwm2 <= 2) {
		direc[1] = 's';
		pwm2=0;
	}
	if(pwm4>30 && direc[3] == 'b') // max speed up
			pwm4=30;
		if(pwm4>8 && direc[3] == 'f')// max speed down
			pwm4=8;
		else if(pwm4<18 && pwm4>3 && direc[3] == 'b')// min  speed up
			pwm4=18;
		else if(pwm4<5 && pwm4>3 && direc[3] == 'f')// min speed down
			pwm4=5;
		else if(pwm4 <= 3)
		{
			direc[3] = 's';
			pwm4=0;
		}

}

void arm_up2(void)
{
	error2 = theta2_up - theta2;
	velocity_ref2 = P(error2,kp12);
	if(velocity_ref2 != 0)
	{
		out2 = PI(velocity_ref2 - velocity2, kp2, ki2, Ts);

	      if(out2 < 0)
	      {
	    	  direc[1] = 'f';
	    	  pwm2 = (uint8_t)(-out2);
	      }
	      else
	      {
	    	  direc[1] = 'b';
	    	  pwm2 = (uint8_t)out2;
	      }
	}
	else // velocity_ref2 = 0  (motor2 off)
	{
		//motor2 off
		direc[1] = 's';
		pwm2 = 0;


		error4 = theta4_up - theta4;
		velocity_ref4 = P(error4, kp14);

		if(velocity_ref4 != 0)
		{
			 out4 = PI(velocity_ref4 - velocity4, kp4, ki4, Ts);
			  if(out4 < 0)
			  {
			      direc[3] = 'f';
			      pwm4 = (uint8_t)(-out4);
			  }
			  else
			  {
			      direc[3] = 'b';
			      pwm4 = (uint8_t)out4;
			  }
		}
		else // velocity_ref4 = 0 (motor4 off)
		{
			direc[3] = 's';
			pwm4 = 0;
			flag = arm_is_up;
		}
	}
		// saturation limits
		if(pwm2>100)
			pwm2=100;
		else if(pwm2<15 && pwm2>2 && direc[1] == 'b')//up
			pwm2=15;
		else if(pwm2<15 && pwm2>2 && direc[1] == 'f')// down
			pwm2=5;
		else if(pwm2 <= 2) {
			direc[1] = 's';
			pwm2=0;
		}


		if(pwm4>40 && direc[3] == 'b') // max speed up
			pwm4=40;
		if(pwm4>8 && direc[3] == 'f')// max speed down
			pwm4=8;
		else if(pwm4<18 && pwm4>3 && direc[3] == 'b')// min  speed up
			pwm4=18;
		else if(pwm4<5 && pwm4>3 && direc[3] == 'f')// min speed down
			pwm4=5;
		else if(pwm4 <= 3)
		{
			direc[3] = 's';
			pwm4=0;
		}
}


void arm_down44(void)
{
	error3 = theta3_down - theta3; // من اجل ارتياب قراءة الADC
	error4 = theta4_down - theta4 ;
	velocity_ref3 = P(error3, kp13);
	velocity_ref4 = P(error4, kp14);
	if(velocity_ref3 != 0)
	{
	    out3 = PI(velocity_ref3 - velocity3, kp3, ki3, Ts);
	      if(out3 < 0)
	      {
	          direc[2] = 'f';
	          pwm3 = (uint8_t)(-out3);
	      }
	      else
	      {
	    	  direc[2] = 'b';
	    	  pwm3 = (uint8_t)out3;
	      }

	 }
	 else // velocity_ref3 = 0 (motor3 off)
	 {
		  direc[2] = 's';
		  pwm3 = 0;
	 }

	 if(velocity_ref4 != 0)
	 {
		  out4 = PI(velocity_ref4 - velocity4, kp4, ki4, Ts);
		  if(out4 < 0)
		  {
			  direc[3] = 'f';
			  pwm4 = (uint8_t)(-out4);
		  }
		  else
		  {
			  direc[3] = 'b';
		      pwm4 = (uint8_t)out4;
		  }
	 }
	 else  // velocity_ref4 = 0 (motor4 off)
	 {
		  direc[3] = 's';
		  pwm4 = 0;
	 }
	 error2 = theta2_down2 - theta2;
	 	      velocity_ref2 = P(error2,kp12);
	 	   	  if(velocity_ref2 != 0)
	 		  {
	 			  out2 = PI(velocity_ref2 - velocity2, kp2, ki2, Ts);

	 			  if(out2 < 0)
	 			  {
	 			      direc[1] = 'f';
	 			      pwm2 = (uint8_t)(-out2);
	 			  }
	 			  else
	 			  {
	 			      direc[1] = 'b';
	 			      pwm2 = (uint8_t)out2;
	 			  }
	 		  }
	 	   	  else
	 	   	  {
	 			  direc[1] = 's';
	 			  pwm2 = 0;

	 	   	  }
	 if(velocity_ref3 == 0 && velocity_ref4 == 0 && velocity_ref2 ==0)
	 {
		 flag = arm_is_down;
	 }

	// saturation limits
	if(pwm2>100)
		pwm2=100;
	else if(pwm2<15 && pwm2>2 && direc[1] == 'b')//up
		pwm2=15;
	else if(pwm2<15 && pwm2>2 && direc[1] == 'f')// down
		pwm2=5;
	else if(pwm2 <= 2) {
		direc[1] = 's';
		pwm2=0;
	}

	if(pwm3>9 && direc[2] == 'f'){
		pwm3 = 9;
	}
	else if(pwm3>50 && direc[2] == 'b')
	{
		pwm3 = 50;
	}
	else if(pwm3<6 && pwm3>3 && direc[2] == 'f')
	{
		pwm3=6;
	}
	else if(pwm3<15 && pwm3>3 && direc[2] == 'b')
	{
		pwm3=15;
	}
	else if(pwm3 <= 3) {
		direc[2] = 's';
		pwm3=0;
	}

	if(pwm4>30 && direc[3] == 'b') // max speed up
		pwm4=30;
	if(pwm4>8 && direc[3] == 'f')// max speed down
		pwm4=8;
	else if(pwm4<25 && pwm4>3 && direc[3] == 'b')// min  speed up
		pwm4=25;
	else if(pwm4<5 && pwm4>3 && direc[3] == 'f')// min speed down
		pwm4=5;
	else if(pwm4 <= 3)
	{
		direc[3] = 's';
		pwm4=0;
	}
}

void arm_up44(void)
{
	error2 = theta2_up - theta2;
	velocity_ref2 = P(error2,kp12);
	if(velocity_ref2 != 0)
	{
		out2 = PI(velocity_ref2 - velocity2, kp2, ki2, Ts);

	      if(out2 < 0)
	      {
	    	  direc[1] = 'f';
	    	  pwm2 = (uint8_t)(-out2);
	      }
	      else
	      {
	    	  direc[1] = 'b';
	    	  pwm2 = (uint8_t)out2;
	      }
	}
	else // velocity_ref2 = 0  (motor2 off)
	{
		//motor2 off
		direc[1] = 's';
		pwm2 = 0;


	}
	error3 = theta3_up - theta3;
			error4 = theta4_up2 - theta4;
			velocity_ref3 = P(error3, kp13);
			velocity_ref4 = P(error4, kp14);
			if(velocity_ref3 != 0)
			{
				out3 = PI(velocity_ref3 - velocity3, kp3, ki3, Ts);
			      if(out3 < 0)
			      {
			    	  direc[2] = 'f';
			    	  pwm3 = (uint8_t)(-out3);
			      }
			      else
			      {
			    	  direc[2] = 'b';
			    	  pwm3 = (uint8_t)out3;
			      }

			}
			else // velocity_ref3 = 0 (motor3 off)
			{
				direc[2] = 's';
				pwm3 = 0;
			}

			if(velocity_ref4 != 0)
			{
				 out4 = PI(velocity_ref4 - velocity4, kp4, ki4, Ts);
				  if(out4 < 0)
				  {
				      direc[3] = 'f';
				      pwm4 = (uint8_t)(-out4);
				  }
				  else
				  {
				      direc[3] = 'b';
				      pwm4 = (uint8_t)out4;
				  }
			}
			else // velocity_ref4 = 0 (motor4 off)
			{
				direc[3] = 's';
				pwm4 = 0;
			}

			if(velocity_ref3 == 0 && velocity_ref4 == 0 && velocity_ref2 == 0)
			{
				flag = arm_is_up;
			}



	// saturation limits
	if(pwm2>100)
		pwm2=100;
	else if(pwm2<15 && pwm2>2 && direc[1] == 'b')//up
		pwm2=15;
	else if(pwm2<10 && pwm2>2 && direc[1] == 'f')// down
		pwm2=5;
	else if(pwm2 <= 2) {
		direc[1] = 's';
		pwm2=0;
	}

	if(pwm3>50 && direc[2] == 'b'){
		pwm3=50;
	}
	else if(pwm3 > 15 && direc[2] == 'f'){
		pwm3=5;
	}
	else if(pwm3<10 && pwm3>2 && direc[2] == 'f')
	{
		pwm3=5;
	}
	else if(pwm3<20 && pwm3>2 && direc[2] == 'b')
	{
		pwm3=20;
	}
	else if(pwm3 <= 2) {
		direc[2] = 's';
		pwm3=0;
	}

	if(pwm4>50 && direc[3] == 'b') // max speed up
		pwm4=50;
	if(pwm4>8 && direc[3] == 'f')// max speed down
		pwm4=8;
	else if(pwm4<28 && pwm4>3 && direc[3] == 'b')// min  speed up
		pwm4=28;
	else if(pwm4<5 && pwm4>3 && direc[3] == 'f')// min speed down
		pwm4=5;
	else if(pwm4 <= 3)
	{
		direc[3] = 's';
		pwm4=0;
	}

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  conveyor[0]='n';
  /* USER CODE END Init */


  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	    /* USER CODE END WHILE */
//
          old1 = theta1;
          old2 = theta2;
          old3 = theta3;
          old4 = theta4;
          AD1=0;
          AD2=0;
          AD3=0;
          AD4=0;

          for(j=0;j<100;j++)
          {
        	  for(i=0; i<4; i++)
        	  	      {

        	  	      HAL_ADC_Start(&hadc1); // Start ADC Conversion
        	  	      HAL_ADC_PollForConversion(&hadc1, 1); // Poll ADC1 Peripheral & TimeOut = 1mSec
        	  	      AD_RES = HAL_ADC_GetValue(&hadc1); // Read ADC Conversion Result
        	  	      switch(i)
        	  	      {
        	  	        case 0:
        	  	          AD1 += AD_RES>>3; // Map The ADC_RES To PWM DutyCycle
        	  	          break;
        	  	        case 1:
        	  	          AD2 += AD_RES>>3; // Map The ADC_RES To PWM DutyCycle
        	  	          break;
        	  	        case 2:
        	  	          AD3 += AD_RES>>3; // Map The ADC_RES To PWM DutyCycle
        	  	          break;
        	  	        case 3:
        	  	          AD4 += AD_RES>>3; // Map The ADC_RES To PWM DutyCycle
        	  	          break;
        	  	       }
        	        }
          }

	      theta1=((AD1/155.555)-87)*(pi/180.0);   //  AD/100 (100 number of reads) degree into rad (need to shift 3 bits to left )
	      theta2=((AD2/155.555)-87)*(pi/180.0);
	      theta3=((AD3/155.555)-87)*(pi/180.0);
	      theta4=((AD4/155.555)-87)*(pi/180.0);
//
	      time_previous = time_now;
	      time_now = HAL_GetTick();
	      Ts = (time_now-time_previous)/1000.0;
	      velocity1=(theta1-old1)/Ts;
	      velocity2=(theta2-old2)/Ts;
	      velocity3=(theta3-old3)/Ts;
	      velocity4=(theta4-old4)/Ts;

//          arm_down2();

	      if(series == 10 )
	      {

	    	  conveyor[0] = 'n';
		      if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)==1)// B15 inductive Proximity Sensor
		      {

		    	  material_type = metal;

		      }
		      if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)==1)// A8 capacity proximity sensor
		      {
		    	  conveyor[0] = 's';  // Stop Conveyor belt
		    	  if(material_type != metal)
		    	  {
		    		  material_type = not_metal;
		    	  }
		    	  series = 1;  // next step

		      }


	      }
	      else if(series == 1)
	      {
	    	  arm_up();
	    	  if(flag == arm_is_up)
	    	  {
	    		   series = 2;
	    	  }


	      }
	      else if(series == 2 )
	      {
	    	  rot_base(pi/2);
	    	  if(base_position_now == (uint8_t)(pi/2))
	    	  {
	    		  series = 3;
	    	  }

	      }


	      else if( series == 3 )
	      {
	    	  arm_down();
	    	  if(flag == arm_is_down)
	    	  {
	    	  	   end_effector[0]='c';
	    	  	   x += 1;
	    	  }
	    	  if(x == 100)
	    	  {
	    		  series = 4;
	    		  x = 0;
	    	  }

	      }
	      else if(series == 4)
	      {
	    	  arm_up44();
	    	  if(flag == arm_is_up)
	    	  {
	    		  series = 5;
	    	  }

	      }
	      else if(series == 5)
	      {
	    	  if(material_type == metal)
	    	  {
	    		  rot_base(pi);
	    		  if(base_position_now == (uint8_t)(pi))
	    		  {
	    			  series = 6;
	    		  }

	    	  }
	    	  else if(material_type == not_metal)
	    	  {
	    		  rot_base(0.0);
	    		  if(base_position_now == (uint8_t)(0.0))
	    		  {
	    			  series = 6;
	    		  }
	           }
	      }
	      else if(series == 6)
	      {
	    	  arm_down44();
	    	  if(flag == arm_is_down)
	    	  {
	    		  series = 7;
	    	  }
	      }
	      else if(series == 7)
	      {
	    	  end_effector[0] = 'o';
	    	  x += 1;                // as a timer
	    	  if(x == 100)
	    	  {
	    		  x = 0;
	    		  series = 10;
	    		  material_type = 0;
	    	  }
	      }





	      memcpy(&AD[0],&pwm1,1);
	      memcpy(&AD[1],&pwm2,1);
	      memcpy(&AD[2],&pwm3,1);
	      memcpy(&AD[3],&pwm4,1);
	      HAL_UART_Transmit(&huart2, end_effector, 1, 100);
	      HAL_UART_Transmit(&huart2, conveyor, 1, 100);
	      HAL_UART_Transmit(&huart2, direc, 4, 100);
	      HAL_UART_Transmit(&huart2, AD, 4, 100);

	      HAL_Delay(15);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /*Configure GPIO pins : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
