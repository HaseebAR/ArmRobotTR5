/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ROBOT_H
#define __ROBOT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g0xx_hal.h"


#define metal                                       1 // The material is metal
#define not_metal                                   2 // The material is not metal

#define pi                                       3.14
#define theta1_0                                   0 // 0 degree for base
#define theta1_90                                1.57 // 90 degree for base
#define theta1_180                               3.14 // 180 degree for base
#define theta2_down                              1.48 // down for shoulder (pi/2)
#define theta2_up                                2.155 // up for shoulder  (3pi/4)
#define theta3_down                              1.80 // down for shoulder
#define theta3_up                                2.20  // up for shoulder
#define theta4_down                              2.0  // down for shoulder
#define theta4_up                                1.40  // up for shoulder
#define theta2_down2                             1.15 // down for shoulder (pi/2)
#define theta4_up2                               1.25  // up for shoulder

#define min_velocity_1                              7 //  7 % PWM
#define min_velocity_2_up                          10 // 10 % PWM
#define min_velocity_2_down                         6 //  6 % PWM
#define min_velocity_3_up                          10 // 10 % PWM
#define min_velocity_3_down                         5 //  5 % PWM
#define min_velocity_4_up                           6 //  6 % PWM
#define min_velocity_4_down                         6 //  6 % PWM

#define done_classification                         0 // fished the operation
#define arm_is_down                                 1 // Arm is down
#define arm_is_up                                   2 // Arm is down
#define catch                                       3 // end_effector is closed (catch)



#define Kp1                                        4.0  // Kp for P controller
#define Kp                                         20.0 // Kp for PI controller
#define Ki                                         5.0  // Ki for PI controller



// parameter for P controller
#define kp11                                        6.0 // kp for P controller motor1
#define kp12                                        6.0 // kp for P controller motor2
#define kp13                                        6.5 // kp for P controller motor3
#define kp14                                        6.0 // kp for P controller motor4

// parameter for PI controller

#define kp1                                        17.0 // kp for PI controller motor1
#define kp2                                        15.0 // kp for PI controller motor2
#define kp3                                        15.0 // kp for PI controller motor3
#define kp4                                        10.0 // kp for PI controller motor4

#define ki1                                        10.0 // ki for PI controller motor1
#define ki2                                        5.0 // ki for PI controller motor2
#define ki3                                        5.0 // ki for PI controller motor3
#define ki4                                        20.0 // ki for PI controller motor4  //25






float PI_one(float error, float Ts);
float P(float error ,float KP);
float PI(float error, float KP, float KI, float Ts);













#ifdef __cplusplus
}
#endif

#endif /* ROBOT_H */
