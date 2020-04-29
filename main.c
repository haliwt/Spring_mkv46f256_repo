/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause  MKV46F256VLH16
 */

/* This is programm for spring pole be control DATA:2020-04-27*/


/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include <stdio.h>
#include "fsl_debug_console.h"

#include "fsl_pwm.h"
#include "pin_mux.h"
#include "fsl_xbara.h"
#include "bsp_led.h"
#include "bsp_key.h"
#include "bsp_bldc.h"
#include "adc.h"
#include "pollingusart.h"
#include "output.h"
#include "input.h"
#include "hall.h"
#include "bsp_encoder.h"
#include "fsl_xbara.h"
#include "fsl_enc.h"
#include "bsp_algorithm.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
 #define AUTOMATICA  0 //1 
__IO int32_t HALL_Pulse;  

output_t  motor_ref;
encoder_t en_t;
int32_t PID_Result;
PID_TypeDef  sPID;
__IO int32_t  PID_PWM_Duty;
BLDC_Typedef BLDCMotor;
reference_t greference_t ;


 
/*******************************************************************************
 *
 * Code
 *
 * @brief Main function
 *
******************************************************************************/
int main(void)
{
    
     uint8_t ucKeyCode=0;
     uint8_t RxBuffer[5],i,k0,j=0;
     
     volatile int16_t DectBuf[6];
     volatile uint16_t Time_CNT,EnBuf[2]={0,0};
	 volatile int32_t mCurPosValue=0,mHoldPos=0;
	 int16_t lkeydir,lstn=0;
   
    XBARA_Init(XBARA);
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
 
    
     LED_Init();
     KEY_Init();
     DelayInit();
     HALL_Init();
     Capture_Input_Init();

   
    OUTPUT_Fucntion_Init();
    ADC_CADC_Init();
   
	
    /* Set the PWM Fault inputs to a low value */
    PWM_BLDC_Init();
   // USART_POLLING_Init();
     #if 0  //no interrupt
    /* Initialize the ENC module. */
    ENC_GetDefaultConfig(&mEncConfigStruct);
    ENC_Init(DEMO_ENC_BASEADDR, &mEncConfigStruct);
    ENC_DoSoftwareLoadInitialPositionValue(DEMO_ENC_BASEADDR); /* Update the position counter with initial value. */
    #endif
	Dir=3;
	en_t.HorizonStop_flag=0;
	
   while(1)
   {
    ucKeyCode = KEY_Scan(0);
	
    /***********Position :Home and End*****************/
        if(en_t.eInit_n !=1){
          
		   PWM_Duty =90;
		   Spring_Itself_Check();
           //mHoldPos = HALL_Pulse;
		  // printf("en_mHoldPos = %d \r\n",mHoldPos);
		 
            if(Dir==0){/*Horizon Position*/

			     Search_Start_HorizonPos();
			   
            }
			else if(Dir == 1){

				Search_Start_VerticalPos();
          
           }
            else{
               LED2 =0;
               DelayMs(50);
               LED2=1;
               DelayMs(50);
               LED2 =0;
            }
		 
        }//end en_t.eIn_n == 0
    /********************************************************************************/     
    /***********motor run main*********************/
     if((motor_ref.motor_run == 1)&&(en_t.HorizonStop_flag !=2)&&(en_t.eInit_n !=0))
     {
   		  if(en_t.eInit_n ==1){
	          if(en_t.DIR_flag ==1)PWM_Duty =90;

			  else PWM_Duty = PID_PWM_Duty;
   		  }
			   
   		  
         /* horizon decelerate region*/
		 if(en_t.HorizonStop_flag ==1 && en_t.eInit_n ==1){

				Balance_HorizonRegion();
					
		 }
		/* motor run */
		if(Dir==0)
	  	{
			  uwStep = HallSensor_GetPinState();
	          HALLSensor_Detected_BLDC(PWM_Duty);
	  	}
	   else{
	  	     PWM_Duty =90;
             uwStep = HallSensor_GetPinState();
	         HALLSensor_Detected_BLDC(PWM_Duty);
	  	}
		mCurPosValue = HALL_Pulse ; /* current read hall pulse   */
		
    
#if 0
         if(Dir == 0){
	          
	         iPrintf();
		 }
#endif
		
         Time_CNT++;
#if 1    
        /* 100ms arithmetic PID */
    	if((Time_CNT % 25 == 0)&&(en_t.eInit_n == 1)&&(en_t.HorizonStop_flag !=2)&&(en_t.HorizonStop_flag !=1)){
   
           
			if(Dir == 0)//CCW  Horizion Direction
			{
						en_t.DIR_flag =0;
					    Spring_Horizon_Decelerate();
						printf("PID_PWM_H 25 = %d\r\n",PID_PWM_Duty);
				
			}
			else{  //Vertical Position judge is boundary
					  
					printf("verHALL = %d \r\n",HALL_Pulse);
					Spring_Vertical_Decelerate();
			    
			}
			
		}
	   if(Time_CNT >=25){
			Time_CNT = 0;
			
	   	}
	   #endif 
	 } 
     else if(en_t.HorizonStop_flag==2){
          
		  en_t.DIR_flag = 1;
          Dir =1;
          PWM_Duty =30; /*real be test*/
          uwStep = HallSensor_GetPinState();
          HALLSensor_Detected_BLDC(PWM_Duty);
 		  
          //Dir =0;
		  PRINTF("flag=2 stop CurrPos !!!!!!!\n");
		  for(lstn=0;lstn < 20000;lstn ++){
		      Dir =1;
	          PWM_Duty =30; /*real be test*/
	          uwStep = HallSensor_GetPinState();
	          HALLSensor_Detected_BLDC(PWM_Duty);
	 		 // Dir =0;
		  }
		 if(greference_t.key_automatic_flag==1){

		      for(lstn=0;lstn < 20000;lstn ++){
		      Dir =1;
	          PWM_Duty =30; /*real be test*/
	          uwStep = HallSensor_GetPinState();
	          HALLSensor_Detected_BLDC(PWM_Duty);
	 		 // Dir =0;
		      }
			 en_t.HorizonStop_flag= 0;
             motor_ref.motor_run =1;
		     en_t.DIR_flag = 1;
		     for(lkeydir=0;lkeydir< 1000;lkeydir++){
				  Dir =1;
				  PWM_Duty =50; /*real be test*/
		          uwStep = HallSensor_GetPinState();
		          HALLSensor_Detected_BLDC(PWM_Duty);
				}
		 }

	}
    else if(en_t.eInit_n !=0){ 
            
		Stop_Fun();
		HALL_Pulse = 0;
         if(motor_ref.motor_run == 3){
				  UART_ReadBlocking(DEMO_UART, RxBuffer, 5);
				  PRINTF("PID input referece \n");
                  for(i=0;i<8;i++){
				  	    
						  if(RxBuffer[0]==0xff){
							  pid_r.KD_H=RxBuffer[1];
		                      pid_r.KI_H=RxBuffer[2];
							  pid_r.KD_H=RxBuffer[3];
							 
		                      motor_ref.motor_run =RxBuffer[4];
		                     PRINTF("KP KI KD = %d %d %d \n\r",pid_r.KP_H,pid_r.KI_H,pid_r.KD_H);
							 
							 
					  	  }
						  else{
								 k0=RxBuffer[0];
								 PRINTF("USART Error !!!!!!!!\n\r");
								 PRINTF("k0 = %d \n\r",k0);
								 PRINTF("KP KI KD = %d %d %d \n\r",pid_r.KP_H,pid_r.KI_H,pid_r.KD_H);
								
								  motor_ref.motor_run =RxBuffer[4];
						      }
                        }
		          
					}
         
		
		 if(greference_t.key_automatic_flag==1){
		   
		   Dir = 0;
		   en_t.HorizonStop_flag= 0;
           motor_ref.motor_run =1;
		   en_t.DIR_flag = 1;
		 }
		 
          
		
    }
            
    
   
   /**********8Key process********************/  
   if(ucKeyCode !=KEY_UP){

	  switch(ucKeyCode){ 
                 
                  case DIR_CW_PRES ://Dir =1 ,PTE29-CW,KEY1,motor to Vertical 

	                Dir =  1;  
	                if(en_t.eInit_n !=1)HALL_Pulse =0;
                   
                    en_t.DIR_flag = 1;
#if 0
	  				  if(en_t.HorizonStop_flag==2){
							
						  for(lkeydir=0;lkeydir< 100;lkeydir++){
							  Dir =1;
							  PWM_Duty =90; /*real be test*/
					          uwStep = HallSensor_GetPinState();
					          HALLSensor_Detected_BLDC(PWM_Duty);
						  }
						
						  en_t.HorizonStop_flag= 0;
                          motor_ref.motor_run =1;
						  en_t.DIR_flag = 1;
                          
	  				  }
                      else {
					  	      Dir =1;
							 for(lkeydir=0;lkeydir< 100;lkeydir++){
							  Dir =1;
							  PWM_Duty =90; /*real be test*/
					          uwStep = HallSensor_GetPinState();
					          HALLSensor_Detected_BLDC(PWM_Duty);
						  }
							  motor_ref.motor_run =1;
                             en_t.DIR_flag =1;

                      	}
#endif         
					  PRINTF("Dir 11111111111111111\n");
                      
				  	break;
        		
                 case START_PRES:
				   motor_ref.motor_run =1;
				  // if(en_t.eInit_n !=1)HALL_Pulse =0;
                   HALL_Pulse = 0;
                   en_t.DIR_flag = 1;
                   en_t.HorizonStop_flag =0;
				   en_t.HorizonStop_flag=0;
                   PRINTF("START_KEY @@@@@@@@@@@@@@@@@@@@@@@@@\n\r");
              
				  break;
		
				 case DIR_CCW_PRES: //Dir = 0;PTE24 = CCW,KEY3,水平方向
				    Dir = 0 ; //
					en_t.DIR_flag = 1;
				    en_t.CCW_flag = 1;
				 	if(en_t.eInit_n !=1)HALL_Pulse =0;
				
				   PRINTF("DIR =000000000\r\n");
	  			   
			    break;
				case MOTOR_STOP_PRES:
					 
                     en_t.HorizonStop_flag=0;
					 motor_ref.motor_run = 3;
				     BLDCMotor.Lock_Time=0;
                     PRINTF("motor stop = %d \n\r",motor_ref.motor_run);
                 break;
				case USART_RT_PRES:/*automaitc be test */

				   j++;
				   if(j==1){
				      greference_t.key_automatic_flag = 1;
					  printf("AUTO BE TEST @@@@@@@@@@@@@@@@@@@@@@@@\n");
				   	}
				   else{
				   	    j=0;
						greference_t.key_automatic_flag = 0;
				   	}
				   
					 
				break;
            default :
              
      
              break;
			
        }
        
	}

  }//end while(1)
}
/******************************************************************************
 *
 * Function Name:BARKE_KEY_IRQ_HANDLER(void)
 * Function Active: Interrpt brake input key 
 * @brief Interrupt service fuction of switch.
 *
 * This function toggles the LED
 *
******************************************************************************/
void BARKE_KEY_IRQ_HANDLER(void )//void BOARD_BRAKE_IRQ_HANDLER(void)
{
  
    /* Clear external interrupt flag. */
    GPIO_PortClearInterruptFlags(BRAKE_KEY_GPIO, 1U << BRAKE_KEY_GPIO_PIN );
    /* Change state of button. */
   
	BLDCMotor.Lock_Time=0;
    motor_ref.motor_run = 0;
    en_t.HorizonStop_flag=0;
	PMW_AllClose_ABC_Channel();
	HALL_Pulse =0;
	#ifdef DRV8302
    GPIO_PinWrite(DRV8302_EN_GATE_GPIO,DRV8302_EN_GATE_GPIO_PIN,0);
    #endif 
	PRINTF("interrupte has happed  \r\n");
	                  
/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

/*****************************************END**********************************************/


