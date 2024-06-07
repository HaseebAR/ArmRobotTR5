/*
 BitzOS (BOS) V0.3.2 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Private variables ---------------------------------------------------------*/
uint8_t index=0;
uint8_t index_now=0;
uint8_t DataRecieve[4];
uint8_t i=0;
uint8_t j=0;
uint8_t pwm1 = 0, pwm2 = 0, pwm3 = 0, pwm4 = 0;
char direction[4];
char end_effector[1];
char conveyor[1];




/* Private function prototypes -----------------------------------------------*/

/* Main function ------------------------------------------------------------*/

int main(void){

	Module_Init();		//Initialize Module &  BitzOS

	//Don't place your code here.




	for(;;){}
}

/*-----------------------------------------------------------*/

/* User Task */
void UserTask(void *argument){


	while(1){



		index = MSG_RX_BUF_SIZE-(DMA1_Channel3->CNDTR);
		if(index == index_now)
			continue;
		index_now = index-10; // 10 is number of sent bytes
		if (index_now < 0)
		    index_now =0;

		end_effector[0]=(char)(UARTRxBuf[2][index_now]);
		index_now += 1;

		conveyor[0]=(char)(UARTRxBuf[2][index_now]);
		index_now += 1;

		for(j=0; j<4; j++)
		{
			direction[j]=(char)(UARTRxBuf[2][index_now]);
			index_now += 1;
		}
		for(i=0; i<4; i++)
		{
			DataRecieve[i]=(uint8_t)(UARTRxBuf[2][index_now]);
			index_now += 1;
		}



		pwm1 = (uint8_t)DataRecieve[0];
		pwm2 = (uint8_t)DataRecieve[1];
		pwm3 = (uint8_t)DataRecieve[2];
		pwm4 = (uint8_t)DataRecieve[3];

		// Saturation limits...

//		// MotorA1.. => Base
//		if(pwm1>100)
//			pwm1=100;
//		else if(pwm1<7 && pwm1>1)
//			pwm1=7;
//		// MotorB1.. => Shoulder
//		if(pwm2>100)
//			pwm2=100;
//		else if(pwm2<10 && pwm2>1 && direction[1]=='b') // up shoulder
//			pwm2=10;
//		else if(pwm2<6 && pwm2>1 && direction[1]=='f') // down shoulder
//			pwm2=6;
//		// MotorA2.. => Forearm
//		if(pwm3>100)
//			pwm3=100;
//		else if(pwm3<10 && pwm3>1 && direction[2]=='b') // up Forearm
//			pwm3=10;
//		else if(pwm3<5 && pwm3>1 && direction[2]=='f') // down Forearm
//			pwm3=5;
//		// MotorB2.. => Wrist
//		if(pwm4>100)
//			pwm4=100;
//		else if(pwm4<6 && pwm4>1)
//			pwm4=6;

       // MotorA1 ON / OFF
		if (direction[0] == 's' || pwm1 < 2){
			Turn_OFF(MotorA);
		}
		else if (direction[0] == 'f'){
			Turn_PWM(forward,pwm1,MotorA);
		}
		else if (direction[0] == 'b'){
			Turn_PWM(backward,pwm1,MotorA);
		}
	    // MotorB1 ON / OFF
		if (direction[1] == 's' || pwm2 < 2){
			Turn_OFF(MotorB);
		}
		else if (direction[1] == 'f'){
			Turn_PWM(forward,pwm2,MotorB);
		}
		else if (direction[1] == 'b'){
		    Turn_PWM(backward,pwm2,MotorB);
		}
		// MotorA2 ON / OFF
		if (direction[2] == 's' || pwm3 < 2){
			messageParams[0]=1; //MotorA Module_2
		    SendMessageToModule(2,CODE_H18R1_Turn_OFF,1);
		}
		else if (direction[2] == 'f'){
			messageParams[0] = 1; // forward
			messageParams[1] = pwm3; // Duty cycle
			messageParams[2] = 1; //MotorA
			SendMessageToModule(2, CODE_H18R1_Turn_PWM,3);
		}
		else if (direction[2] == 'b'){
			messageParams[0] = 2; // backward
			messageParams[1] = pwm3; // Duty cycle
			messageParams[2] = 1; //MotorA
			SendMessageToModule(2, CODE_H18R1_Turn_PWM,3);
		}
		// MotorB2 ON / OFF
		if (direction[3] == 's' || pwm4 < 2){
			messageParams[0]=2; //MotorB Module_2
		    SendMessageToModule(2,CODE_H18R1_Turn_OFF,1);
		}
		else if (direction[3] == 'f'){
			messageParams[0] = 1; // forward
			messageParams[1] = pwm4; // Duty cycle
			messageParams[2] = 2; //MotorB
			SendMessageToModule(2, CODE_H18R1_Turn_PWM,3);
		}
		else if (direction[3] == 'b'){
			messageParams[0] = 2; // backward
			messageParams[1] = pwm4; // Duty cycle
			messageParams[2] = 2; //MotorB
			SendMessageToModule(2, CODE_H18R1_Turn_PWM,3);
		}

		// End_effector (Module 3 , MotorA)
		if(end_effector[0] == 'c'){
			messageParams[0] = 1; // forward
			messageParams[1] = 50; // Duty cycle
			messageParams[2] = 1; //MotorA
			SendMessageToModule(3, CODE_H18R1_Turn_PWM,3);
		}
		else if (end_effector[0]== 'o'){
			messageParams[0]=1; //MotorA Module_3
			SendMessageToModule(3,CODE_H18R1_Turn_OFF,1);
		}

		// Conveyor belt (Module 3 , MotorB)
		if(conveyor[0] == 'n'){
			messageParams[0]=1; //forward
			messageParams[1]=2; //MotorB Module_3
			SendMessageToModule(3, CODE_H18R1_Turn_ON, 2);
		}
		else if (conveyor[0] == 's'){
			messageParams[0]=2; //MotorB Module_3
			SendMessageToModule(3,CODE_H18R1_Turn_OFF,1);
		}


		Delay_ms(10);


	}

	}


/*-----------------------------------------------------------*/
