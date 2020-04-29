#include <stdio.h>
#include "bsp_algorithm.h"

static void CoilSpring_Run_VerticalLockHall(void);

reference_t refer_t;

tpid_refer pid_r={0.1f,0.01f,0.1f,0.5f,0.01f,0.5f};

/****************************************************
	*
	*Function Name:void Spring_Itself_Check(void)
	*Function : itself spring pole arrive aim
	*
	*
*****************************************************/
void Spring_Itself_Check(void)
{
	
     static uint32_t cnt=0;
     uint32_t loop       = 1U;
     uint32_t secondLoop = 25U;
 
	static uint8_t mn=0;
	en_t.Start_n++;

	if (en_t.Start_n >= secondLoop)
	{
		//printf("%c\r\n", signals[cnt & 1]);
		cnt++;
		if (cnt >= loop)
		{
		    cnt = 0;
		    mn ++ ;
		    if(mn==1) refer_t.stdBuf[0]= HALL_Pulse;
		    if(mn==2)refer_t.stdBuf[1]= HALL_Pulse;
		    
		     printf("stdBuf[0] : %d\r\n", refer_t.stdBuf[0]);
		     printf("stdBuf[1] : %d\r\n", refer_t.stdBuf[1]);
		     en_t.Pos_diff = abs(abs(refer_t.stdBuf[1])-abs(refer_t.stdBuf[0]));

			printf("ftmHall : %d\r\n", HALL_Pulse);
		    if(mn >=2)mn =0;
		}
		
         en_t.Start_n = 0U;
	  if((refer_t.stdBuf[0] >0 && refer_t.stdBuf[1]<0)||(refer_t.stdBuf[0] <0 && refer_t.stdBuf[1]>0)){
                  
                  
                      en_t.HorizonStop_flag=0;
                      PMW_AllClose_ABC_Channel();
                      DelayMs(50);
                      GPIO_PortToggle(GPIOD,2<<BOARD_LED1_GPIO_PIN);
                      DelayMs(50);
                      HALL_Pulse =0;
                      motor_ref.motor_run=0;
                      en_t.eInit_n=1;
                      if(en_t.CCW_flag == 1){ /* run to horizon direction*/
                         if(refer_t.stdBuf[1] >0) en_t.X_axis = refer_t.stdBuf[1];
                         else if(refer_t.stdBuf[0]>0)en_t.X_axis = refer_t.stdBuf[0];
                         en_t.Y_axis = 0;
                         printf("ver_Y00 : %d\r\n", en_t.Y_axis);
                         printf(" Hor_X 00: %d\r\n", en_t.X_axis);
                        
                      }
                      else { /*run to vertical direction*/
                          if(refer_t.stdBuf[1] < 0) en_t.Y_axis = refer_t.stdBuf[1];
                          else if(refer_t.stdBuf[0]< 0)en_t.Y_axis = refer_t.stdBuf[0];
						  en_t.X_axis=0;
                          printf("ver_Y 11: %d\r\n", en_t.Y_axis);
                          printf(" hor_X 11: %d\r\n", en_t.X_axis);
                          
                      }
                
                      printf("stdBuf[0]000000 : %d\r\n", refer_t.stdBuf[0]);
                      printf("stdBuf[1]000000 : %d\r\n", refer_t.stdBuf[1]);
                   
                  }
	}


}

/****************************************************
	*
	*Function Name:void Search_Start_HorizonPos(void)
	*Function: start power automatic check horizon 
	*          position
	*
	*
*****************************************************/
void Search_Start_HorizonPos(void)
{
	PWM_Duty =70;
	uwStep = HallSensor_GetPinState();
	HALLSensor_Detected_BLDC(PWM_Duty);
				
}
/*************************************************************
	*
	*Function Name:void Search_Start_VerticalPos(void)
	*Function: auto detected start power horizon position
	*Input Reference: No
	*Return Reference :No
	*
*************************************************************/
void Search_Start_VerticalPos(void)
{
	
	PWM_Duty =70;
	uwStep = HallSensor_GetPinState();
	HALLSensor_Detected_BLDC(PWM_Duty);
}        
            
/************************************************************
	*
	*Function Name:void Srping_Horizon_Decelerate(void)
	*Function :has to spring pole horizon decelerate
	*
	*
*************************************************************/
void Spring_Horizon_Decelerate(void)
{
    volatile int32_t iError,last_iError,dError_sum;
	
	
	iError = HALL_Pulse - en_t.X_axis; /*  pid error  */
	printf("mHor X axis= %d \n\r",en_t.X_axis);
	printf("iError = %ld \r\n",iError);
	
	
	if(en_t.X_axis < 200){/*refer vertical*/
      if(abs(HALL_Pulse) <50 ){
		PWM_Duty =30;
		uwStep = HallSensor_GetPinState();
		HALLSensor_Detected_BLDC(PWM_Duty);
		en_t.HorizonStop_flag =1; /*input decelerate region flag*/
		 dError_sum = 0;
		 iError=0;
		 last_iError =0;
         printf("Stop30 CurrPos ##############################: %ld\r\n",HALL_Pulse);
		}
     }
	else if(abs(en_t.X_axis) >800){

		if((en_t.X_axis - HALL_Pulse) < 50 && (en_t.X_axis - HALL_Pulse)> 0 ){
			
			PWM_Duty =30;
			uwStep = HallSensor_GetPinState();
			HALLSensor_Detected_BLDC(PWM_Duty);
			en_t.HorizonStop_flag =1; /*input decelerate region flag*/
		
			dError_sum = 0;
			iError=0;
			last_iError =0;
			if(en_t.Home_flag ==1)en_t.End_flag =1;
			printf("Stop60 CurrPos : %ld\r\n", HALL_Pulse);
			}
							

	}
    if(en_t.HorizonStop_flag ==1){/*input horizon banance region*/
		Dir =1;
		PWM_Duty =30;
		uwStep = HallSensor_GetPinState();
		HALLSensor_Detected_BLDC(PWM_Duty);
	}
	else{
        dError_sum += iError; 

		if(dError_sum > 1000)dError_sum = 1000; /*error accumulate */
		if(dError_sum < -1000)dError_sum = -1000; 
		PID_PWM_Duty = (int32_t)((float)(iError /1000) + (float)(dError_sum /10000) + (float)((iError - last_iError)/1000));//proportion + itegral + differential

		printf("hor_pwm= %d \r\n",PID_PWM_Duty);
        
        if(abs(PID_PWM_Duty) >=70) PID_PWM_Duty =70;
        
		PID_PWM_Duty = abs(PID_PWM_Duty);
	

		last_iError = iError;
		PWM_Duty = PID_PWM_Duty;
		
	}

}

/************************************************
	*
	*Function Name: Vertical_Decelerate(void)
	*Function: vertical slowdown region 
    *          from horizon to vertical
    *
	*
************************************************/
void Spring_Vertical_Decelerate(void)
{
	uint16_t ldectnum;

	   if(abs(en_t.X_axis) > 100){
			if((abs(en_t.X_axis) + 4500) <= abs(HALL_Pulse)){ //|| (2 * en_t.X_axis -8000) < abs(HALL_Pulse)){
			  
				for(ldectnum =0;ldectnum<70;ldectnum++){
				 ldectnum++;
				 if(ldectnum <=70){
					 PWM_Duty = 70 - ldectnum;
				 }
				 uwStep = HallSensor_GetPinState();
				 HALLSensor_Detected_BLDC(PWM_Duty);
				 printf("V>800 break !!!!!!!\r\n");
			  }
			 PMW_AllClose_ABC_Channel();
			 motor_ref.motor_run =0;
			 printf("V>80\r\n");
			 printf("V~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\r\n");
			 
			 }
			 
		}
		else if(abs(en_t.X_axis) < 30 ){
			
			 if((abs(en_t.Y_axis)-en_t.Vertical_HALL_Pulse)< 50){

		       for(ldectnum =0;ldectnum<30;ldectnum++){
				 ldectnum++;
				if(ldectnum <=30){
					 PWM_Duty = 30 - ldectnum;
				 }
				 else  PWM_Duty =0;
				 uwStep = HallSensor_GetPinState();
				 HALLSensor_Detected_BLDC(PWM_Duty);
				 printf("V < 20 break ##########################################\r\n");
				
			    }
			 
				 PMW_AllClose_ABC_Channel();
				 motor_ref.motor_run =0;
				 en_t.oneKey_H_flag = 0;
			     en_t.oneKey_V_flag = 1;
				 en_t.Home_flag = 1;
				 printf("v100Pos= %d \n\r",en_t.Vertical_HALL_Pulse);
				 printf("V < 100 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
			 
			}
		}
   //}
	
}
/*****************************************
	*
	*
	*
	*
****************************************/
#if 0
uint8_t SWAP(uint8_t *p,uint8_t *q)
{
     uint8_t temp = *p;
	 if(*p < *q){
		  *p = *q;
		  *q = temp;
	      return *q; 
	 
	 }
	 else return *q;

}
#endif 
/***********************************************
	*
	*Function Name:void Self_Locking(void)
	*Function: itself locking somewhere position 
	*
	*
	*
	*
************************************************/
void Self_Locking(int16_t svalue)
{
   int16_t self_temp,lvalue,ldiff_value;
   int16_t error,pid_pwm,derror_sum ,last_error;
   float kp = 0.01,ki = 0.01,kd = 0.01;

   
   self_temp = ENC_GetPositionValue(DEMO_ENC_BASEADDR);
   
   
   ldiff_value = self_temp - svalue ;
   if(ldiff_value > 0){ /* to motor run horizon */

         if(Dir ==0){

	          Dir =1;
	          PWM_Duty =30; /*real be test*/
	          uwStep = HallSensor_GetPinState();
	          HALLSensor_Detected_BLDC(PWM_Duty);
	 		  
	          Dir =0;
         }
		 else{
              PWM_Duty =0; /*real be test*/
	          uwStep = HallSensor_GetPinState();
	          HALLSensor_Detected_BLDC(PWM_Duty);
		     
		 	}
   
   	}
    if(ldiff_value < 0){ /* motor to move vertical   */
   	while(ldiff_value > 2||ldiff_value < -2){
   	
			error =lvalue - en_t.X_axis ; /*  pid error  */
		    derror_sum += error; 
							
		    if(derror_sum > 1000)derror_sum =1000; /*error accumulate */
			if(derror_sum < -1000)derror_sum = -1000; 
            pid_pwm = (int32_t)(error *kp + derror_sum * ki + (error - last_error)*kd);//proportion + itegral + differential

			if(pid_pwm > 0 )/* motor to horizon  */
			{
               Dir = 1 ; /* reverser to direction vertical */
			   PWM_Duty = abs(pid_pwm);
	           uwStep = HallSensor_GetPinState();
	           HALLSensor_Detected_BLDC(PWM_Duty);
			
			}
			else{ /* motor to vertical*/
			    Dir = 0 ; /* reverser to direction horizon */
			   PWM_Duty = abs(pid_pwm);
	           uwStep = HallSensor_GetPinState();
	           HALLSensor_Detected_BLDC(PWM_Duty);
			
			}
		   lvalue = ENC_GetPositionValue(DEMO_ENC_BASEADDR);
			
   		}
   
    	}

}
/************************************************
	*
	*Function Name:void Balance_HorizonRegion(void)
	*
	*
	*
*************************************************/
void Balance_HorizonRegion(void)
{
	int32_t mCurPosValue,lhorizonpos;
	uint16_t z=0;
	
	mCurPosValue = HALL_Pulse; /*read current position of value*/
	lhorizonpos =en_t.X_axis - HALL_Pulse ;

	printf("stop lhorizonpos = %d \n",lhorizonpos);

	if(en_t.X_axis < 100){
			if(lhorizonpos <=100 && lhorizonpos < 0 ){

				for(z=0;z<70;z++){
					
				Dir =1;
				PID_PWM_Duty =PID_PWM_Duty - z;
				if(PID_PWM_Duty <= 0)PID_PWM_Duty =0;
				PWM_Duty =PID_PWM_Duty ;
				uwStep = HallSensor_GetPinState();
				HALLSensor_Detected_BLDC(PWM_Duty);
				Dir =0 ;
				DelayMs(3);
				Stop_Fun();
				printf("X_axis <100 ~~~~~~~~~~~~~~~~~~~~~~\n");
				
			    }
				motor_ref.motor_run =0;
				//en_t.HorizonStop_flag =2;
	   }
	}
	else if(en_t.X_axis > 800){
				if(lhorizonpos > 50 && lhorizonpos > 0){
					for(z=0;z<30;z++){
			           
						Dir = 1;
						PWM_Duty =30 - z;
						uwStep = HallSensor_GetPinState();
						HALLSensor_Detected_BLDC(PWM_Duty);
						Dir =0;
						DelayMs(3);
						printf("X_axis>800 Stop&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&\n");
						Stop_Fun();
					}
					motor_ref.motor_run =0;
				  //  en_t.HorizonStop_flag =2;
				}
	 }
	
}

/***********************************************
	*
	*Function Name: iPrintf(void)
	*
	*
	*
************************************************/
void iPrintf(void)
{
	printf("eIn_n = %d \r\n",en_t.eInit_n);
	printf("PWM_Duty = %d \r\n",PWM_Duty);
	printf("X_axis_hor = %d\r\n",en_t.X_axis);
	printf("Y_axis_ver = %d\r\n",en_t.Y_axis);
	printf("Hall number = %d \r\n",HALL_Pulse);

}
/*********************************************************
	*
	*Function Name:void Stop_Fun(void)
	*
	*
	*
	*
**********************************************************/
void Stop_Fun(void)
{
	PWM_Duty =0 ;
	PMW_AllClose_ABC_Channel();
	DelayMs(50);
	GPIO_PortToggle(GPIOD,1<<BOARD_LED1_GPIO_PIN);
	DelayMs(50);
}
/************************************************
	*
	*Function Name:void CoilSpring_Run_VerticalLockHall(void)
	*Function : motor normal check vertical stop position.
	*
	*
	*
	*
*************************************************/
static void CoilSpring_Run_VerticalLockHall(void)
{
     static uint32_t cnt=0;
     uint32_t loop       = 1U;
     uint32_t secondLoop = 25U;
     static int16_t verBuf[2];
	 static uint8_t mn=0;
	
	 en_t.m_n++;

	if (en_t.m_n >= secondLoop)
	{
		cnt++;
		if (cnt >= loop)
		{
		    cnt = 0;
		    mn ++ ;
		    if(mn==1) verBuf[0]= HALL_Pulse;
		    if(mn==2) verBuf[1]= HALL_Pulse;
		    
		     printf("ver_stdBuf[0] : %d\r\n", verBuf[0]);
		     printf("ver_stdBuf[1] : %d\r\n", verBuf[1]);
		     en_t.Pos_diff = abs(abs(verBuf[1])-abs(verBuf[0]));

			printf("coilhall : %d\r\n", HALL_Pulse);
		    if(mn >=2)mn =0;
		}
		
         en_t.m_n= 0U;
	  if((verBuf[0] >0 && verBuf[1]<0)||(verBuf[0] <0 && verBuf[1]>0)||en_t.Pos_diff < 25){
                  

			PWM_Duty = 0;
			PMW_AllClose_ABC_Channel();
			HALL_Pulse =0;
			motor_ref.motor_run=0;

			en_t.X_axis = 0;
			if(refer_t.stdBuf[1] < 0) en_t.Y_axis = verBuf[1];
			else if(refer_t.stdBuf[0]< 0)en_t.Y_axis = verBuf[0];
			en_t.Vertical_check_n ++ ;
			printf("ver_Y coil: %d\r\n", en_t.Y_axis);
			printf(" hor_X coil: %d\r\n", en_t.X_axis);
            printf("coil verBuf[0] : %d\r\n", verBuf[0]);
			printf("coil verBuf[1] : %d\r\n", verBuf[1]);
					 
                   
      }
	 
	}	
}
/************************************************************
	*
	*Function Name:void Horizon_StopRegion(void)
	*Function :Horizon pole stop region different
	*		   stop ,this is balance horizon
	*
	*
*************************************************************/
void Horizon_StopRegion(void)
{
      uint32_t lstn=0;
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
		   if(refer_t.key_automatic_flag==1){
	
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
			 
		   }


}

