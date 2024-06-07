#include "robot.h"

// must define Kp,Ki in "robot.h"
float PI_one(float error,float Ts)
{
	 static float error_previous, sum;
//	 static uint8_t x;
	 float  out_PI;
//	 x += 1;
//	 if(x == 5)
//	 {
//		 sum=0;
//	 }
	 sum += error;
	 out_PI = Kp*error + Ki*(error+error_previous)*Ts;
	 error_previous = error;
	 if(out_PI > 100.0)
		 out_PI = 100.0;
	 else if(out_PI < -100.0)
		 out_PI = -100.0;


	 return out_PI;

}

float P(float error , float KP)
{

	 float out_P;
	 out_P = KP*error ;
	 if(out_P > pi/2)   // Max speed = pi/2 (rad/s)
		 out_P = pi/2;
	 else if(out_P < -pi/2)
		 out_P = -pi/2;

	 if(error < 0.05 && error > -0.05)
		 out_P = 0.0;


	 return out_P;
}


float PI(float error, float KP, float KI, float Ts)
{
	 static float error_previous,sum ;
	 float probortional, integral, out_PI ;

//	 if(x == 5)//////////////////////////
//		 sum = 0;
//	 sum += error;
	 integral = KI*(error+error_previous)*Ts;
	 if(integral > 5.0)
		 integral = 5.0;
	 else if(integral < -5.0)
		 integral = -5.0;
	 probortional = KP*error;
	 out_PI = probortional + integral;
	 error_previous = error;
	 if(out_PI > 100.0)
		 out_PI = 100.0;
	 else if(out_PI < -100.0)
		 out_PI = -100.0;

	 return out_PI;
}
