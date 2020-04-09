/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause  MKV46F256VLH16
 */

/*System includes.*/


/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include <stdio.h>
#include "fsl_debug_console.h"

#include "fsl_pwm.h"
#include "pin_mux.h"
#include "fsl_xbara.h"
#include "led.h"
#include "key.h"
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
__IO int32_t HALL_Pulse;  

output_t  motor_ref;
encoder_t en_t;
int32_t PID_Result;
PID_TypeDef  sPID;
__IO int32_t  PID_PWM_Duty;
BLDC_Typedef BLDCMotor;

struct _pid_reference pid_r={1.0f,1.0f,1.0,1.0f,1.0f,1.0f};
/*******************************************************************************
 *
 * Code
 *
 * @brief Main function
 *
******************************************************************************/
int main(void)
{
    
     enc_config_t mEncConfigStruct;
	 volatile int32_t dError_sum;
	 volatile int32_t iError,last_iError ;
     uint8_t printx1[]="Key Dir = 1 is CW !!!! CW \r\n";
     uint8_t printx2[]="Key Dir = 0 is CCW \r\n";
 
     uint8_t ucKeyCode=0,z=0,m=0;
     uint8_t RxBuffer[8],i,k0,judge_n;

	
     volatile uint16_t Time_CNT,EnBuf[2]={0,0};
	 volatile int32_t mCurPosValue,mHoldPos,HDff,VDff,Dff;
	 uint32_t eIn_n= 0;
	 int16_t j=0;
   

	 
    
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
     #if 1  //no interrupt
    /* Initialize the ENC module. */
    ENC_GetDefaultConfig(&mEncConfigStruct);
    ENC_Init(DEMO_ENC_BASEADDR, &mEncConfigStruct);
    ENC_DoSoftwareLoadInitialPositionValue(DEMO_ENC_BASEADDR); /* Update the position counter with initial value. */
    #endif
	Dir=3;
   
	
   while(1)
   {
       ucKeyCode = KEY_Scan(0);
    mHoldPos = ENC_GetHoldPositionValue(DEMO_ENC_BASEADDR);
       
	//#ifdef DEBUG_PRINT 
		   PRINTF("CPHod: %d\r\n", mHoldPos);
           PRINTF("en_t.eInit_n: %d\r\n", en_t.eInit_n);
	//#endif
			
		
#if 1
	 /***********Position :Home and End*****************/
        if(en_t.eInit_n ==0)
        {
            PWM_Duty=50 ;
			 if(eIn_n > 2 ){
                   
				   j++;
                   if(j==1){
                            EnBuf[0]=mHoldPos;
                            
							   }
				   else if(j==2){
				   			EnBuf[1]= mCurPosValue;
                           }

                   if(j>=2){  /*judge and setup this Home and End Position */
					
			               if((EnBuf[0]==EnBuf[1])){
                     			judge_n++; /*judge home and end position twice*/
								PRINTF("Z==2 ############# \n\r");
                                PMW_AllClose_ABC_Channel();
                                motor_ref.motor_run = 0;
								PRINTF("judge_n =%d \n\r",judge_n);
								/*To judge  Home position and End position*/

								 if(Dir ==0){/*honrizon Position*/
							    		PRINTF("HALL > 0\n\r");
								 		if(judge_n==1){
										        en_t.Home_flag = 1;
												en_t.Horizon_Position = ENC_GetHoldPositionValue(DEMO_ENC_BASEADDR);
												en_t.First_H_dec = 1;
												PRINTF("HorizPos_1 = %d\r\n",en_t.Horizon_Position);
												PRINTF("Hor_HALL_1 = %d\r\n",en_t.Horizon_HALL_Pulse);
								 		}
										else{
												if((judge_n==2)&&(en_t.End_V_flag !=1)){

												        /* */
													    if(en_t.First_H_dec==1){
													        en_t.End_H_flag = 1;
															en_t.Vertical_Position = ENC_GetHoldPositionValue(DEMO_ENC_BASEADDR);
															
															PRINTF("VerPos_2 = %d\r\n",en_t.Vertical_Position);
														}
														else{
															/**/
						
													        en_t.End_H_flag = 1;
															en_t.Horizon_Position = ENC_GetHoldPositionValue(DEMO_ENC_BASEADDR);
														
															PRINTF("HoPos_2 = %d\r\n",en_t.Horizon_Position);
														
														}
											 
										        }
										 
					                     }
								 }
								else{
										/*HALL_Pulse < 0*/
										PRINTF("judge_n = %d\n\r",judge_n);
										PRINTF("End_H_flag = %d \r\n",en_t.End_H_flag);
										PRINTF("First_H_dec = %d \r\n",en_t.First_H_dec);
								
										if(judge_n==1 ){
								
											 en_t.First_V_dec =1;
											 en_t.Vertical_Position = ENC_GetHoldPositionValue(DEMO_ENC_BASEADDR);
											
											 PRINTF("--VerPos_1 = %d\r\n",en_t.Vertical_Position);
											
											 HALL_Pulse =0;

										
										}
										else if((judge_n==2)&& (en_t.End_H_flag !=1)){ 

											
											/*��һ�μ�⵽��ֱλ�ã��ڶ����Ǽ�⵽ˮƽλ��,hall <0*/
											if(en_t.First_V_dec == 1){
												
												 en_t.Horizon_HALL_Pulse = HALL_Pulse;
												 en_t.End_V_flag = 1;
												 en_t.Horizon_Position = ENC_GetHoldPositionValue(DEMO_ENC_BASEADDR);
												
												 PRINTF("--HorPos_2 = %d\r\n",en_t.Horizon_Position);
												
											}
											else{
												
												/*�ڶ��δ�ֱλ��*/
											
										        en_t.End_V_flag = 1;
												en_t.Vertical_Position = ENC_GetHoldPositionValue(DEMO_ENC_BASEADDR);
												
												 PRINTF("--VerPos_2= %d\r\n",en_t.Vertical_Position);
											}
										}	
								
												
							      }
						   }
						if(judge_n==2){
						 	      en_t.eInit_n++;
								  HALL_Pulse =0;
						         
                                  iError =0;
                                  PID_PWM_Duty=50;
								  /*判断最小值*/
								  if(en_t.Horizon_Position > en_t.Vertical_Position)
								  	en_t.mini_value = en_t.Vertical_Position;
								  else en_t.mini_value = en_t.Horizon_Position ;
								  en_t.VH_Total_Dis = abs(en_t.Horizon_Position -en_t.Vertical_Position);//Toatal= horizonPos-verticalPos
						    }
						
					}
                    if(j>=2)j=0;
			}
            if(eIn_n > 2)eIn_n =0;
        }
#endif 
         
    /***********motor run main*********************/
     if((motor_ref.motor_run == 1)&&(en_t.HorizonStop_flag !=2))
     {
   		  if(en_t.eInit_n == 1){
			   PWM_Duty = PID_PWM_Duty;
			   PRINTF("motor run pwm: %d\r\n",PID_PWM_Duty);
   		  	}
		  else if(en_t.eInit_n == 0)
		  {
			if(Dir ==0){
			
            PWM_Duty = 50 - m;
			if(PWM_Duty > 0) 
            {
			    if(Time_CNT % 100 == 0)
				{
				   m=m+5;
                   
				}
            }
            else  PWM_Duty =0;
			if(Time_CNT >=100)Time_CNT =0;
			PWM_Duty = abs(PWM_Duty);
            PRINTF("start pwm : %d\r\n", PWM_Duty);
			}  
		  }
   		   
		   #ifdef DRV8302 
            GPIO_PinWrite(DRV8302_EN_GATE_GPIO,DRV8302_EN_GATE_GPIO_PIN,1);
		   #endif 
		  uwStep = HallSensor_GetPinState();
          HALLSensor_Detected_BLDC(PWM_Duty);
          
	      mCurPosValue = ENC_GetPositionValue(DEMO_ENC_BASEADDR); /*read current position of value*/
	       // #ifdef DEBUG_PRINT 
	       //  PRINTF("CurrPos : %d\r\n", mCurPosValue);
	      //  #endif
               
		   eIn_n ++; 
		
           Time_CNT++;
#if 1    
        /* 100ms arithmetic PID */
    	if((Time_CNT % 50== 0)&&(en_t.eInit_n == 1)){
   
            PRINTF("Time_CNT : %d\r\n", Time_CNT);
			mCurPosValue = ENC_GetPositionValue(DEMO_ENC_BASEADDR); /*read current position of value*/
			if(Dir == 0)//CCW HB0 = Horizion
			{
				        iError =mCurPosValue - en_t.Horizon_Position ; //����?
					      PRINTF("mCurPosValue= %d \n\r",mCurPosValue);
						
						  if(iError > 1)
						  	printf("hor_iError > 1 = %d \n\r",iError);
						  else if(iError <1)
						   	printf("hor_iEerro < 1 = %d \n\r",iError );
						   else  printf("iHerror == 1 = %d \n\r",iError );
						//#endif
				       	HDff = iError;
					   	HDff = abs(iError);
				        if(HDff <= 500 ){
						  
						    en_t.HorizonStop_flag =2;
                            for(z=0;z<250;z++){
                            Dir =1;
                            PWM_Duty =50;
                            uwStep = HallSensor_GetPinState();
                            HALLSensor_Detected_BLDC(PWM_Duty);
							PRINTF("Stop500 CurrPos : %d\r\n", mCurPosValue);
                            Dir =0;
                           }
												
						}
					    else if(en_t.HorizonStop_flag ==2){
						     PWM_Duty = 0;
					    }
					    else{
							dError_sum += iError; 
							
						    if(dError_sum > 1)dError_sum =1; /*error accumulate */
							if(dError_sum < -1)dError_sum = -1; 
		                    PID_PWM_Duty = (int32_t)(iError *pid_r.KP_H + dError_sum * pid_r.KI_H + (iError - last_iError)*pid_r.KD_H);//proportion + itegral + differential
					       // #ifdef DEBUG_PRINT
								if(PID_PWM_Duty > 0)
									PRINTF("hpid_pwm= %d\r \n",PID_PWM_Duty);
								else if(PID_PWM_Duty == 0)PRINTF("hpid_pwm= %d\r \n",PID_PWM_Duty);
								else PRINTF("-hpid_pwm= -%d\r \n",PID_PWM_Duty);
							//#endif
							PID_PWM_Duty = abs(PID_PWM_Duty);
							if(PID_PWM_Duty >=50)PID_PWM_Duty=50;

		                 	last_iError = iError;
							PWM_Duty = PID_PWM_Duty;
							
					    }
				
				
			}
            /***************************************************/
			else{  //Vertical Position judge is boundary

				       iError = mCurPosValue - en_t.Vertical_Position ; //
				 		//if(iError <= 10 && iError >= -10)iError =0;
						#ifdef DEBUG_PRINT
                			PRINTF("HB0= %d \n\r",en_t.Vertical_Position);
					      	PRINTF("mCurPosValue= %d \n\r",mCurPosValue);
						    PRINTF("currHALL= %d \n\r",HALL_Pulse);
						#endif
				       	VDff = iError;
						#ifdef DEBUG_PRINT
						   	if(VDff>=0)
						      	PRINTF("VDff = %d \n\r",VDff);
							 else
							   	PRINTF("-VDff = - %d \n\r",VDff);
                        #endif 
						
             			VDff = abs(VDff);
					   HALL_Pulse = abs(HALL_Pulse);
					   PRINTF("VDff= %d  \r\n",VDff);
					   PRINTF("VcurrHALL= %d \n\r",HALL_Pulse);
                      if( VDff<=50 ){
					  	      BLDCMotor.Lock_Time ++;
							  BLDCMotor.Position = VDff;
					          HALL_Pulse = 0;
							 
                    	     PRINTF("VDff= %d \r\n",VDff);
	                         PMW_AllClose_ABC_Channel();
	                         motor_ref.motor_run =0;
							 PRINTF("VVVVVVVV\r\n");
							 iError =0;
							 last_iError=0;
							 PRINTF("Vpwm VDFF =50 = %d \r\n",PID_PWM_Duty);
	                            
				   	     
			   	   	        
                      }
					  else {
					        dError_sum += iError; 
					
						    if(dError_sum > 1000)dError_sum =1000; 
							if(dError_sum < -1000)dError_sum = -1000; 
		                    PID_PWM_Duty = (int32_t)(iError *pid_r.KP_V + dError_sum * pid_r.KI_V + (iError - last_iError)*pid_r.KD_V);//proportion + itegral + differential
					       // #ifdef DEBUG_PRINT
								if(PID_PWM_Duty > 0)
									PRINTF("PID pwm= %d\r \n",PID_PWM_Duty);
								else if(PID_PWM_Duty == 0)PRINTF("PID pwm= %d\r \n",PID_PWM_Duty);
								else PRINTF("-PID pwm= -%d\r \n",PID_PWM_Duty);
							//#endif
							PID_PWM_Duty = abs(PID_PWM_Duty);
						    
							if(PID_PWM_Duty >=50)
						    if(PID_PWM_Duty >=50)PID_PWM_Duty=50;

		                 	last_iError = iError;
							PWM_Duty = PID_PWM_Duty;
							
							HALL_Pulse =0;
					  	}
			    
			}
			HALL_Pulse =0;
		}
	   if(Time_CNT >=50){
			Time_CNT = 0;
			HALL_Pulse =0;
	   	}
	   #endif 
	} 
     else if(en_t.HorizonStop_flag==2){
       
          Dir =1;
          PWM_Duty =30;
          uwStep = HallSensor_GetPinState();
          HALLSensor_Detected_BLDC(PWM_Duty);
 		  PRINTF("flag=2 CurrPos : %d\r\n", mCurPosValue);
          Dir =0;
		 
     
     }
    else { 
            
			    en_t.HorizonStop_flag=0;
				PMW_AllClose_ABC_Channel();
				HALL_Pulse =0;
				iError =0;
				#ifdef DRV8302
				GPIO_PinWrite(DRV8302_EN_GATE_GPIO,DRV8302_EN_GATE_GPIO_PIN,0);
				#endif 
		
	      DelayMs(50);
	      GPIO_PortToggle(GPIOD,1<<BOARD_LED1_GPIO_PIN);
	      DelayMs(50);
	     HALL_Pulse =0;
         if(motor_ref.motor_run == 3){
				  UART_ReadBlocking(DEMO_UART, RxBuffer, 8);
				  PRINTF("PID input referece \n");
                  for(i=0;i<8;i++){
				  	    
						  if(RxBuffer[0]==0xff){
							  pid_r.KD_H=RxBuffer[1];
		                      pid_r.KI_H=RxBuffer[2];
							  pid_r.KD_H=RxBuffer[3];
							  pid_r.KP_V = RxBuffer[4];
							  pid_r.KI_V = RxBuffer[5];
							  pid_r.KD_V = RxBuffer[6];
		                         motor_ref.motor_run =RxBuffer[7];
		                     PRINTF("KP KI KD = %d %d %d \n\r",pid_r.KP_H,pid_r.KI_H,pid_r.KD_H);
							 PRINTF("vkp vki vkd = %d %d %d \n\r",pid_r.KP_V,pid_r.KI_V,pid_r.KD_V);
		                    
							 
					  	  }
						  else{
								 k0=RxBuffer[0];
								 PRINTF("USART Error !!!!!!!!\n\r");
								 PRINTF("k0 = %d \n\r",k0);
								 PRINTF("KP KI KD = %d %d %d \n\r",pid_r.KP_H,pid_r.KI_H,pid_r.KD_H);
								 PRINTF("VKP VKI VKD = %d %d %d \n\r",pid_r.KP_V,pid_r.KI_V,pid_r.KD_V);
								  motor_ref.motor_run =RxBuffer[7];
						      }
                        }
		          
					}
        // PRINTF("Motor Stop !!!!!!!!!!! \r\n");
        // PRINTF("motor_run = %d \r\n",motor_ref.motor_run );
                
    }
            
    
   
   /**********8Key process********************/  
   if(ucKeyCode !=KEY_UP){

	  switch(ucKeyCode){ 
                 
                  case DIR_CW_PRES ://Dir =1 ,PTE29-CW,KEY1
				  	// Dir = 1;
                    en_t.HorizonStop_flag=0;
			          BLDCMotor.Lock_Time=0;
                    if(Dir == 0) //
	   			    {

                        if((motor_ref.motor_run == 1)&&(motor_ref.Dir_flag == 0))//motor is runing
                        {
                            
                              PWM_Duty= 10;
							  uwStep = HallSensor_GetPinState();
				              HALLSensor_Detected_BLDC( PWM_Duty);
							   PWM_Duty = 5;
							  uwStep = HallSensor_GetPinState();
							  HALLSensor_Detected_BLDC( PWM_Duty);
                              DelayMs(30);
                         
                              motor_ref.Dir_flag =1;
                              Dir =1;
                            
                          
                        }
						else 
						{
							Dir=1;
                       		 motor_ref.Dir_flag =1;
						}
						
                     }
                      else
                      {
                        Dir=1;
                        motor_ref.Dir_flag =1;
                       
                       
                      
                      }
                         UART_WriteBlocking(DEMO_UART, printx1, sizeof(printx1) - 1);
				    HALL_Pulse =0;
                      
				  	break;
        		
                 case START_PRES:
				   motor_ref.motor_run =1;
				   HALL_Pulse =0;
                
				   en_t.HorizonStop_flag=0;
                   PRINTF("START_KEY @@@@@@@@@@@@@@@@@@@@@@@@@\n\r");
              
				  break;
		
				 case DIR_CCW_PRES: //Dir = 0;PTE24 = CCW,KEY3
				   // Dir = 0 ; //
				 
				   PRINTF("DIR =0\r\n");
	  			   
			     if(Dir==1) // Dir = 0; //ï¿½ï¿½Ê±ï¿½ï¿½ï¿½ï¿½×ª
				   {
	               
	                 if((motor_ref.motor_run == 1) &&(motor_ref.Dir_flag ==1))//motor is runing
	                 {
	                   
	                      PWM_Duty = 10;
						  uwStep = HallSensor_GetPinState();
			              HALLSensor_Detected_BLDC( PWM_Duty);
						   PWM_Duty = 5;
						  uwStep = HallSensor_GetPinState();
						  HALLSensor_Detected_BLDC( PWM_Duty);
	                      DelayMs(30);
	                   
	             
	                      Dir =0;
	                    
	                 }
						else
						{
						  Dir =0;
						 
	                     
						}
						
	               }
	               else
	                  {
	                     
	                     Dir = 0;
	                    
	                  
	                  }
				   HALL_Pulse =0;
				   BLDCMotor.Lock_Time=0;
                 UART_WriteBlocking(DEMO_UART, printx2, sizeof(printx2) - 1);
			   
			
           		break;
				case MOTOR_STOP_PRES:
                    en_t.HorizonStop_flag=0;
					 motor_ref.motor_run = 3;
					 HALL_Pulse =0;
				     BLDCMotor.Lock_Time=0;
                     PRINTF("motor stop = %d \n\r",motor_ref.motor_run);
                 break;
				case USART_RT_PRES:
					 
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

