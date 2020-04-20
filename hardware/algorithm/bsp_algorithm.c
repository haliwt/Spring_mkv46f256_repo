#include <stdio.h>
#include "bsp_algorithm.h"

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

#if 0

pid_t algpid_t ;

static void PID_HorizonSTOP_Region(int32_t mStopHoldP,int32_t mCurPos);
static void PID_VerticalStop_Region(int32_t mStopHoldP,int32_t mCurPos);
static uint8_t Greedy_Algorithms(int32_t mstandardvalue, uint8_t merrorvalue,uint8_t msl);

/*************************************************
	*
	*funtion Name:void Detect_HorVer_Positon(void)
	*Function :detect horizon home and vertical end position
	*
	*
**************************************************/
void Detect_HorVer_Position(void)
{
	algpid_t.hv_n++; //recoder input detect horizon or vertical times.
	PRINTF("Detect_HorVer_Postioo\n");
	if(algpid_t.hv_n==1){
		algpid_t.Buff[0]=algpid_t.mHoldPos;

	}
	else if(algpid_t.hv_n==3){
		algpid_t.Buff[1]= algpid_t.mCurPosValue;
	}

	/*judge and setup this Home and End Position */
	if(algpid_t.hv_n == 3){
		
		if(algpid_t.Buff[0]==algpid_t.Buff[1]){

			/*judge home and end position twice*/
		   
			en_t.HorVer_R_times++;
			PRINTF("HorVer_times=%d \n\r",en_t.HorVer_R_times);
			PMW_AllClose_ABC_Channel();
			motor_ref.motor_run = 0;
			algpid_t.oneKeyDetector_flag ++; /*记录自动检测垂直和水平位置次数*/
		
			
			PRINTF("en_t.HorVer_R_times = %d\n\r",en_t.HorVer_R_times);
			/*To judge  Home position and End position*/
			 if(HALL_Pulse >=0){ /*Dir = 0*/
			 	    /*Direction to motor run  horizon position (home)*/
					PRINTF("HALL > 0\n\r");
				    PRINTF("Position revolution value: %d\r\n", ENC_GetHoldRevolutionValue(DEMO_ENC_BASEADDR));
			 		if(en_t.HorVer_R_times==1 || en_t.HorVer_R_times ==3){
						    /*The first judge horizon position (home)*/
							en_t.Horizon_HALL_Pulse =HALL_Pulse;
					      
							en_t.Horizon_Position = ENC_GetHoldPositionValue(DEMO_ENC_BASEADDR);
							en_t.First_H_dec = 1;
							PRINTF("HorizP_1 = %d\r\n",en_t.Horizon_Position);
							PRINTF("Hor_HALL_1 = %d\r\n",en_t.Horizon_HALL_Pulse);
							HALL_Pulse =0;
							if(algpid_t.oneKeyDetector_flag==1){/*一键学习式*/
								
								Dir =1; //motor run to Vertical direction
								motor_ref.motor_run = 1;
							}
							if(algpid_t.oneKeyDetector_flag == 3)algpid_t.total_n=2;
			 		}
					else{

						if((en_t.HorVer_R_times==2)&&(en_t.First_H_dec == 1)){

						        /* */
							    
								if(en_t.First_H_dec==1){

								    en_t.Vertical_HALL_Pulse =HALL_Pulse;
							        en_t.End_H_flag = 1;
									en_t.Vertical_Position = ENC_GetHoldPositionValue(DEMO_ENC_BASEADDR);
								
									PRINTF("VerPos_2 = %d\r\n",en_t.Vertical_Position);
									PRINTF("VerHal_2 = %d\r\n",en_t.Vertical_HALL_Pulse);
									HALL_Pulse =0;
									/*一键学习方式*/
									Dir =0; //motor run to horizon direction
								    motor_ref.motor_run = 1;
									
								}
								else if((en_t.HorVer_R_times==2)&&(en_t.End_V_flag !=1)){
									/**/
									en_t.Horizon_HALL_Pulse =HALL_Pulse;
							        en_t.End_H_flag = 1;
									en_t.Horizon_Position = ENC_GetHoldPositionValue(DEMO_ENC_BASEADDR);
								
									PRINTF("HoPos_2 = %d\r\n",en_t.Horizon_Position);
									PRINTF("HoHal_2 = %d\r\n",en_t.Horizon_HALL_Pulse);
									HALL_Pulse =0;
									Dir =1; //motor run to Vertical direction
								    motor_ref.motor_run = 1;
								}
					 
				        }
					 
			        }
		 }
		else {  /* Dir = 1;HALL_Pulse == 0 motor stop  HALL < 0  */
				/*Direction vertical motor run to Position */
				PRINTF("HALL < 0 \r\n");
				
				PRINTF("End_H_flag = %d \r\n",en_t.End_H_flag);
				PRINTF("First_H_dec = %d \r\n",en_t.First_H_dec);
				
				//PRINTF("Position revolution value: %d\r\n", ENC_GetHoldRevolutionValue(DEMO_ENC_BASEADDR));
			
				if(en_t.HorVer_R_times == 1 || en_t.HorVer_R_times == 3 ){
					 PRINTF("-HallN 1=- %d\r\n", HALL_Pulse );
					 en_t.Vertical_HALL_Pulse = HALL_Pulse;
			    
					 en_t.First_V_dec =1;
					 en_t.Vertical_Position = ENC_GetHoldPositionValue(DEMO_ENC_BASEADDR);
					
					 PRINTF("--Ver_1 = %d\r\n",en_t.Vertical_Position);
					 PRINTF("--Ver_HALL_1 =- %d\r\n",en_t.Vertical_HALL_Pulse);
					 HALL_Pulse =0;
					 if(algpid_t.oneKeyDetector_flag==1){
								
								Dir =0; //motor run to Horizon direction
								motor_ref.motor_run = 1;
					 }
					 if(algpid_t.total_n==3)algpid_t.total_n=2;
				}
				else if(en_t.HorVer_R_times==2){ 

					
					/*the second detector horizon Position*/
					if(en_t.First_V_dec == 1){
						
						 en_t.Horizon_HALL_Pulse = HALL_Pulse;
						 en_t.End_V_flag = 1;
						 en_t.Horizon_Position = ENC_GetHoldPositionValue(DEMO_ENC_BASEADDR);
						 
						 PRINTF("--Hor_2 = %d\r\n",en_t.Horizon_Position);
						 PRINTF("--Hor_HALL_2 =- %d\r\n",en_t.Horizon_HALL_Pulse);
						 HALL_Pulse =0;
						 /*one key study machine*/
						 Dir =1; //motor run to vertical direction
						 motor_ref.motor_run = 1;
					}
					else if(en_t.End_H_flag !=1){
						
						/*the second detector Horizon Position*/
						en_t.Vertical_HALL_Pulse = HALL_Pulse;
				        en_t.End_V_flag = 1;

						en_t.Vertical_Position = ENC_GetHoldPositionValue(DEMO_ENC_BASEADDR);
						
						 PRINTF("--VerPos_2= %d\r\n",en_t.Vertical_Position);
						 PRINTF("--Ver_HALL_2 =- %d\r\n",en_t.Vertical_HALL_Pulse);
						 HALL_Pulse =0;
						 algpid_t.total_n=2;
						  /*one key study machine*/
						Dir =0; //motor run to Horizon direction
						motor_ref.motor_run = 1;
					}
						
				}
		 	}
		}
    }
    if(algpid_t.hv_n>=3)algpid_t.hv_n=0;
}
/*****************************************************************************************
	*
	*Function Name: Horizontal_Decelerate_Function(void)
	*Function: horizon decelerate speed algroithm.
	*
	*
*****************************************************************************************/
void Horizontal_Decelerate_Function(void)
{
		int32_t HDff;
		uint8_t n;
		float tempv;
		algpid_t.iError =(float)((en_t.Horizon_Position-algpid_t.mCurPosValue)/en_t.VH_Total_Dis) ;//iError =[0 1]
	//	if(algpid_t.iError <= 20 && algpid_t.iError >= -20)algpid_t.iError =0; //error range 
		//#ifdef DEBUG_PRINT
	   		printf("algpid_t.iError= %2.3f \n\r",algpid_t.iError);
	      PRINTF("mCurPosValue= %d \n\r",algpid_t.mCurPosValue);
		    PRINTF("currHALL= %d \n\r",HALL_Pulse);
	//	#endif
	   	HDff =(int32_t) algpid_t.iError; 
	   	if(HDff>=0)
	  		PRINTF("HDff = %d \n\r",HDff);
	   	else
	   		PRINTF("-HDff = - %d \n\r",HDff);
	     //pid judge arrive to difference horizon position 
	   	if(((HDff <= 400 && HDff >= -400)&&(HALL_Pulse>10))&&(en_t.eInit_n==1)){

			  algpid_t.hor_n++;
		    BLDCMotor.Lock_Time++; /*Motor block up times,has error judgement */
		    if(algpid_t.hor_n>=2){
	          if( algpid_t.total_n == 1)//WT.EDIT 2020.04.08     
				       en_t.HorizonStop_flag =2;
					HDff = abs(HDff);
					PRINTF("HDff= %d\r\n",HDff);
					PRINTF("PID_PWM_Duty = %d\r\n",PID_PWM_Duty);

					PWM_Duty=0;
					PID_PWM_Duty=0;
					/*transient stop hover */
					for(n =0;n<250;n++)//300 times motor run to Vertical is error
					{
	                    Dir =1; /*motor run to vertical reverser direction*/
					    PWM_Duty=50;
						uwStep = HallSensor_GetPinState();
						HALLSensor_Detected_BLDC(PWM_Duty);
					
				    	Dir =0; /*motor run to horizon */
		        	}
						PRINTF("nnnnnnnn---- \r\n");
				    if(algpid_t.hor_n>=2)algpid_t.hor_n=0;
								   
			}
				
		}

	if(en_t.HorizonStop_flag ==2){
		
		    PWM_Duty = PID_PWM_Duty;
	}
	else {//pid arithmetic
			algpid_t.dError_sum += algpid_t.iError; 
			
		  if(algpid_t.dError_sum > 50)algpid_t.dError_sum =50; /*error accumulate */
			if(algpid_t.dError_sum < -50)algpid_t.dError_sum = -50; 
	        tempv= (algpid_t.iError *P_DATA + algpid_t.dError_sum * I_DATA + (algpid_t.iError - algpid_t.last_iError)*D_DATA);//proportion + itegral + differential
			PID_PWM_Duty = (int32_t)tempv;
			#ifdef DEBUG_PRINT
				if(tempv> 0)
					PRINTF("PID pwm= %2.3f\r \n",tempv);
				else if(PID_PWM_Duty == 0)PRINTF("PID pwm= %2.3f\r \n",tempv);
				else PRINTF("-PID pwm= -%2.3f\r \n",tempv);
			#endif
			PID_PWM_Duty = abs(PID_PWM_Duty);
			if(PID_PWM_Duty >=50)PID_PWM_Duty=50;
	      
	     	algpid_t.last_iError = algpid_t.iError;
			PWM_Duty = PID_PWM_Duty;
		
	}
	HALL_Pulse =0;	

}
/**********************************************************************
	*
	*Function Name:Vertical_Decelerate_Function()
	*Function: pole to run Vertical decelerate
	*
	*
***********************************************************************/
void Vertical_Decelerate_Function(void)
{
  		float temp;
		algpid_t.iVError =(float) ((algpid_t.mCurPosValue -en_t.Vertical_Position) /en_t.VH_Total_Dis);
		//	if(algpid_t.iVError <= 20 && algpid_t.iVError >= -20)algpid_t.iVError =0;
		//#ifdef DEBUG_PRINT
			printf("V-iError= %2.2f \n\r",algpid_t.iVError);
	      //	PRINTF("mCurPosValue= %d \n\r",algpid_t.mCurPosValue);
		 //   PRINTF("currHALL= %d \n\r",HALL_Pulse);
	//	#endif
	   
		#ifdef DEBUG_PRINT
		   	if( algpid_t.iVError>=0)
		      	PRINTF("VDff = %d \n\r",VDff);
			 else
			   	PRINTF("-VDff = - %d \n\r",VDff);
	    #endif 
		
			
	   HALL_Pulse = abs(HALL_Pulse);
	   PRINTF("VDff= %d  \r\n", algpid_t.iVError);
	   PRINTF("VcurrHALL= %d \n\r",HALL_Pulse);
	  if(((  algpid_t.iVError<=50 )&&(HALL_Pulse>2))&&en_t.eInit_n==1){
	  	      BLDCMotor.Lock_Time ++;
	          HALL_Pulse = 0;
			 if(BLDCMotor.Lock_Time >=2){//judge the pole if arrived the vertical position

					en_t.VerticalStop_flag =1 ;
					Dir =0;
				    PWM_Duty=30;
					uwStep = HallSensor_GetPinState();
					HALLSensor_Detected_BLDC(PWM_Duty);
					Dir =1; 
					algpid_t.mStopVerPos = ENC_GetPositionValue(DEMO_ENC_BASEADDR);

					 PRINTF("VDff= %d \r\n", algpid_t.iVError);
	                 PMW_AllClose_ABC_Channel();
	                 motor_ref.motor_run =0;
					 DelayMs(1000);
	                 PRINTF("locktime = %d\r\n",BLDCMotor.Lock_Time);
	                 BLDCMotor.Lock_Time=0;
	                 PRINTF("VVVVVVVV\r\n");
					 HALL_Pulse =0;
					 algpid_t.iVError =0;
					 algpid_t.dvError_sum=0;
					 algpid_t.last_ivError=0;
					 PRINTF("PID_PWM= %d \r\n",PID_PWM_Duty);
	            }
	}
				
	algpid_t.dvError_sum += algpid_t.iVError; 

	if(algpid_t.dvError_sum > 50)algpid_t.dvError_sum =50; 
	if(algpid_t.dvError_sum < -50)algpid_t.dvError_sum = -50; 
	temp =(float)(algpid_t.iVError *VP_DATA + algpid_t.dvError_sum * VI_DATA + (algpid_t.iVError - algpid_t.last_ivError)*VD_DATA);//proportion + itegral + differential
	PID_PWM_Duty = (int32_t)temp;
    #ifdef DEBUG_PRINT
		if(temp > 0)
			printf("PID pwm= %2.3f\r \n",temp);
		else if(PID_PWM_Duty == 0)printf("PID pwm= %2.3f\r\n",temp);
		else printf("-PID pwm= -%2.3f\r \n",temp);
	#endif
	
	PID_PWM_Duty = abs(PID_PWM_Duty);
	algpid_t.last_ivError = algpid_t.iVError;
	PWM_Duty = PID_PWM_Duty;
	HALL_Pulse =0;
}
/*************************************************
	*
	*Function Name:
	*Function:horizon stop region when the motor run limit to stop
	*
	*
**************************************************/
void HorizonStop_Region(void)
{
	uint8_t temp;
	 algpid_t.mCurPosValue = ENC_GetPositionValue(DEMO_ENC_BASEADDR); /*read current position of value*/
	//PRINTF("Position differential value: %d\r\n", (int16_t)ENC_GetHoldPositionDifferenceValue(DEMO_ENC_BASEADDR));
	//PRINTF("Position revolution value: %d\r\n", ENC_GetHoldRevolutionValue(DEMO_ENC_BASEADDR));
	/*贪心算法*/
	temp = Greedy_Algorithms(80,20,1);
	Dir =1;
	if(algpid_t.horizon_balance_stop_flag ==1)
		PWM_Duty =temp;
	else 
		PWM_Duty = 30;
	uwStep = HallSensor_GetPinState();
	HALLSensor_Detected_BLDC(PWM_Duty);
	HALL_Pulse =0;
	Dir =0;
	PRINTF("STOP HOR motor Run\r\n");
	HALL_Pulse =0;
	Dir =0;//horizon direction
	algpid_t.iError=0;  /*pid error reference clear to zero*/
	algpid_t.dError_sum=0;
	algpid_t.last_iError=0;
	algpid_t.iVError =0;
	algpid_t.last_ivError =0;
	algpid_t.dvError_sum =0;

}
/************************************************************
	*
	*Function Name:Balance_Stop_Function()
	*Function:Balance stop function.machine learning
	*
	*
************************************************************/
void Balance_Stop_Function(void)
{
	/*keep balance pole position*/
	int8_t lpwm_value  ;//l -local 
	en_t.Idrun_times =0;  
  	en_t.HorizonStop_flag=0;
  	algpid_t.iVError=0;
	algpid_t.iError=0;
	algpid_t.dError_sum=0;
	algpid_t.last_iError=0;
	algpid_t.iVError =0;
	algpid_t.last_ivError =0;
	algpid_t.dvError_sum =0;
	HALL_Pulse =0;

	/*贪心算法*/
	lpwm_value=Greedy_Algorithms(0,20,0); /**/
	
	Dir =1;
	if(algpid_t.balance_stop_flag ==1)
		PWM_Duty =lpwm_value;
	else 
		PWM_Duty = 30;
	uwStep = HallSensor_GetPinState();
	HALLSensor_Detected_BLDC(PWM_Duty);
//	PRINTF("STOP HOR ^^^^^^^^^^\r\n");
	HALL_Pulse =0;
	Dir =0;
}
/*************************************************
	*
	*Function Name:static void PID_STOP_Region()
	*Function:Balance stop function.horizon position balance
	*Input Regerence:pid standard value criterion ,current read position cvalue
	*
	*
**************************************************/
static void PID_HorizonSTOP_Region(int32_t mStopHoldP,int32_t mCurPos)
{
	volatile int32_t tempvalue,tempierror,ierror_sum,ilast_error;
	float pDATA = 1.0f,iDATA = 0.1f,dDATA =0.1f,iresult;
	tempvalue = mCurPos - en_t.Horizon_Position ;
	tempierror = mStopHoldP - tempvalue; //error of value 

	ierror_sum += tempierror; //accumulate error of value
	if(ierror_sum  > 100)ierror_sum  =50; 
	if(ierror_sum  < -100)ierror_sum  = -50; 

	iresult = (float)(tempierror *pDATA + ierror_sum * iDATA + ( tempierror- ilast_error)*dDATA);//proportion + itegral + differential   
	ilast_error = ierror_sum;
	/*balance position wave value pid algorithem*/
	if((HALL_Pulse >=0)&&( mStopHoldP <= 20 &&  mStopHoldP >= -20 )){  
		/*motor run to horizon position Dir =0 */
		Dir = 1; /* negative direction run to vertical reverser*/
		iresult =abs(iresult);
		PWM_Duty =(uint32_t) iresult ;
		uwStep = HallSensor_GetPinState();
		HALLSensor_Detected_BLDC(PWM_Duty);
	    
	}
   if((HALL_Pulse < 0)&&( mStopHoldP <= 20 &&  mStopHoldP >= -20 )){/* Dir = 1; Run to CW direction to verticonal position*/

		Dir = 0; /* positive direction run to horizon reverser */
		iresult =abs(iresult);
		PWM_Duty = (uint32_t)iresult ;
		uwStep = HallSensor_GetPinState();
		HALLSensor_Detected_BLDC(PWM_Duty);
		
	}
}
/**********************************************************************
	*
	*Function :VerticalStop_Regin(void)
	*
	*
	*
**********************************************************************/
void VerticalStop_Regin(void)
{
    /*keep balance pole position*/
	en_t.Idrun_times =0;  
    en_t.HorizonStop_flag=0;
    algpid_t.iVError=0;
	algpid_t.iError=0;
	algpid_t.dError_sum=0;
	algpid_t.last_iError=0;
	algpid_t.iVError =0;
	algpid_t.last_ivError =0;
	algpid_t.dvError_sum =0;
	algpid_t.mCurPosValue = ENC_GetPositionValue(DEMO_ENC_BASEADDR); /*read current position of value*/
	HALL_Pulse =0;
	
	PID_VerticalStop_Region(algpid_t.mStopVerPos,algpid_t.mCurPosValue);

}

/**********************************************************************
	*
	*Function :PID_VerticalStop
	*
	*
	*
**********************************************************************/
static void PID_VerticalStop_Region(int32_t mStopHoldP,int32_t mCurPos)
{
	volatile int32_t tempvalue,tempierror,ierror_sum,ilast_error;
	float pDATA = 1.0f,iDATA = 0.1f,dDATA =0.1f,iresult;
	tempvalue = mCurPos - en_t.Horizon_Position ;
	tempierror = mStopHoldP - tempvalue; //error of value 

	ierror_sum += tempierror; //accumulate error of value
	if(ierror_sum  > 100)ierror_sum  =50; 
	if(ierror_sum  < -100)ierror_sum  = -50; 

	iresult = (float)(tempierror *pDATA + ierror_sum * iDATA + ( tempierror- ilast_error)*dDATA);//proportion + itegral + differential   
	ilast_error = ierror_sum;
	/*balance position wave value pid algorithem*/
	if((HALL_Pulse >=0)&&( mStopHoldP <= 20 &&  mStopHoldP >= -20 )){  
		/*motor run to horizon position Dir =0 */
		Dir = 1; /* negative direction run to vertical reverser*/
		iresult =abs(iresult);
		PWM_Duty =(uint32_t) iresult ;
		uwStep = HallSensor_GetPinState();
		HALLSensor_Detected_BLDC(PWM_Duty);
	    
	}
   if((HALL_Pulse < 0)&&( mStopHoldP <= 20 &&  mStopHoldP >= -20 )){/* Dir = 1; Run to CW direction to verticonal position*/

		Dir = 0; /* positive direction run to horizon reverser */
		iresult =abs(iresult);
		PWM_Duty = (uint32_t)iresult ;
		uwStep = HallSensor_GetPinState();
		HALLSensor_Detected_BLDC(PWM_Duty);


  }

}
/***************************************************************************************
    *
	* Function Name: Greedy_Algorithms()
	* Input Regerence: standard value , error value,msl -
	*         0:-balance stop  1:-horizon motor run stop
	*
	*
*****************************************************************************************/
static uint8_t Greedy_Algorithms(int32_t mstandardvalue, uint8_t merrorvalue,uint8_t msl)
{
	int32_t lvalue,lsvalue,i;   //l -local 
	HALL_Pulse =0;
	if(msl == 0)
		algpid_t.mCurPosValue = ENC_GetPositionValue(DEMO_ENC_BASEADDR) - mstandardvalue;
	else 
		algpid_t.mCurPosValue = en_t.Horizon_Position -ENC_GetPositionValue(DEMO_ENC_BASEADDR)-mstandardvalue ;

	if(algpid_t.g_memory_n ==0){
		algpid_t.g_memory_n++;
	    algpid_t.Buff[0]= algpid_t.mCurPosValue; /*save standard value in algpid_t.Buff[0]*/
	}
	lvalue = algpid_t.Buff[0];
	if(algpid_t.horizon_balance_stop_flag==1){
	}
	else if((algpid_t.Buff[0] != algpid_t.mCurPosValue) && (algpid_t.balance_stop_flag != 1)){
		if(algpid_t.horizon_balance_stop_flag==1)
	
		/*贪心算法*/
		while(HALL_Pulse >0 ){
			
				for(i=0;i<21;i++){
					/*Dir =0 run to horizon position*/
					Dir =1; //reverser to direction vertical 
					algpid_t.times_vertical_stop = i;
					PWM_Duty =30 + algpid_t.times_vertical_stop * ALPHA; /*study coefficient */
					uwStep = HallSensor_GetPinState();
					HALLSensor_Detected_BLDC(PWM_Duty);
					if(HALL_Pulse ==0 || HALL_Pulse < 0 ) break;
				}
		}
		
		while(HALL_Pulse < 0){
		   /*Dir =1 run to Vertical position*/
		   for(i=0;i<20;i++){
			Dir =0; //reverser to direction horizon
			algpid_t.times_horizon_stop = i;
			PWM_Duty =30 - algpid_t.times_horizon_stop * EPSILON;
			uwStep = HallSensor_GetPinState();
			HALLSensor_Detected_BLDC(PWM_Duty);
			if(HALL_Pulse ==0 || HALL_Pulse > 0 ) break;
		   }
			
		}
		lsvalue = ENC_GetPositionValue(DEMO_ENC_BASEADDR);/*the second read current postion value*/
		if(HALL_Pulse == 0 || (lsvalue <= algpid_t.Buff[0]+merrorvalue && lsvalue >=algpid_t.Buff[0]-merrorvalue)){

		    algpid_t.pwm_api = PWM_Duty;
			  DelayMs(1000);
		    if(algpid_t.pwm_api == PWM_Duty){
				if(msl == 0)
					algpid_t.balance_stop_flag = 1;
				else 
					algpid_t.horizon_balance_stop_flag=1;
		
		    }
			
		}
		
		return algpid_t.pwm_api;
	}
	return algpid_t.pwm_api;
	
	
}
#endif 