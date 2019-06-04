// Header:    
// File Name: CloseLoopControl.c
// Author:		LinhTran
// Date:      3/12/2014

#include "data_struct.h"
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "CloseLoopControl.h"
#include "board.h"
#include "EncoderIncremental.h"
#include "ExternalCounter.h" 	

#define DT 0.001	//1 Khz
#define SENSOR_RIGHT  0
#define SENSOR_LEFT   0
#define UI_MAX        999
#define UP_MAX        999
#define UD_MAX        999
#define PID_MAX       999
#define PID_MIN				30
#define VELOCITY_MAX  1000.0
#define VELOCITY_MIN	1.0
#define CURRENT_DOW		4		//mA

// Phien
#define CUR_MAX				7000
#define VEL_MAX				10000
float cur_ref, vel_ref;

#define MIN_VEL_EPSILON 	1.7
#define MAX_VEL_EPSILON		50

extern STATE_MECHINE asStateMechine;
extern DATA_HW_INFOR asDataHwInfor;
extern DATA_RUNING_INFOR asRuningInforCurrent,asRuningInforSet,asRuningInforOlder;
extern DATA_MOTOR_INFOR asMotorInfor;
extern volatile int64_t aExternCounterValue;
extern volatile uint8_t aHaveNewCommand;

// Phien
extern DATA_MOTOR_CURR_INFOR asMotorCurrInfor;

// Phien
float aCurrentErr;

double denta;
float aPositionErr,aPositionOldErr,aPositionOlderErr;
float aVelocityErr, aVelocityOldErr;
float aVelocityErr;
uint8_t aMotorDir;
float uP,uI,uD;
int16_t PID;
int8_t aLoopEnable=0;

extern  int8_t MOTOR_ControlDir(void);
extern void ReadExternalCounter(void);
extern float READ_VelocitySetByADC(void);


////////term:
float currentErr, currentSet;

float absF(float aArg){
  if(aArg<0)  return -1*aArg;
  else return aArg;
}
double absD(double aArg){
  if(aArg<0) return -1.0*aArg;
  else return aArg;
}
int8_t ERROR_MotorCheck(void){
	int8_t ret =0;
	static int8_t aFlagSuccess=0;
	float  aFollowError;
	
	switch( asStateMechine.asRunningMode){
			case MODE_TURNING:
				if(asRuningInforCurrent.aListError ==0){
					static uint16_t countTuning =0;
					if(asStateMechine.asLoopControlStatus == CLOOP_PROCESS){
							countTuning++;
					}
					else{
						countTuning=0;
					}
					//check error:
					if(countTuning==100){
							if(asRuningInforCurrent.aCurrent>CURRENT_DOW){	
										if(asRuningInforCurrent.aVelocity<0){
											asRuningInforCurrent.aListError |= PHASE_ENCODER_REVERSE;
											asStateMechine.asLoopControlStatus = CLOOP_STOOP;
										}
										else if(asRuningInforCurrent.aVelocity<VELOCITY_MIN){
											asRuningInforCurrent.aListError |= ENCODER_ERR;
											asStateMechine.asLoopControlStatus = CLOOP_STOOP;
										}
							}
							else{
								asRuningInforCurrent.aListError |= PHASE_ERR;
								asStateMechine.asLoopControlStatus = CLOOP_STOOP;
							}
					}
				}
				break;
			case MODE_PI_VELOCITY:
			case MODE_PI_VELOCITY_BY_ADC:
				if(asRuningInforCurrent.aListError ==0){
					if(asRuningInforCurrent.aListError ==0){
							aFollowError = asRuningInforCurrent.aEstimatesValue - asRuningInforCurrent.aVelocity;
							if(absF(aFollowError)>asDataHwInfor.aFollowError){
								if(asRuningInforCurrent.aCurrent>CURRENT_DOW){
										if(absD(asRuningInforCurrent.aVelocity)>VELOCITY_MIN){
												if(asRuningInforCurrent.aEstimatesValue /asRuningInforCurrent.aVelocity<0){
														asRuningInforCurrent.aListError |= PHASE_ENCODER_REVERSE;
												}
												else{
														asRuningInforCurrent.aListError |= FOLLOW_ERR; 
												}
										}
										else{
												asRuningInforCurrent.aListError |= ENCODER_ERR;
										}	
								}
								else{
										asRuningInforCurrent.aListError |= PHASE_ERR;
								}
								asStateMechine.asLoopControlStatus = CLOOP_STOOP;
							}
					}
						
				}
				break;
			 default :
				 if(asRuningInforCurrent.aListError ==0){
						aFollowError = asRuningInforCurrent.aEstimatesValue - asRuningInforCurrent.aPosition;
						if(absF(aFollowError)>asDataHwInfor.aFollowError){
									if(absD(asRuningInforCurrent.aVelocity)>VELOCITY_MIN){		// asRuningInforCurrent.aVelocity
											 if((asRuningInforCurrent.aPosition-asRuningInforOlder.aPosition)/(asRuningInforCurrent.aEstimatesValue-asRuningInforOlder.aPosition)>0){	//Following Error:
													 if(asRuningInforCurrent.aCurrent>CURRENT_DOW){
														 asRuningInforCurrent.aListError |= FOLLOW_ERR; 
													 }
													 else{																															//MOTOR Phase have no conect
															asRuningInforCurrent.aListError |= PHASE_ERR;
														}
											 }
											 else{																														
													 if(aFlagSuccess!=1){//Encoder or Phase of motor have to reverse 
															 asRuningInforCurrent.aListError |= PHASE_ENCODER_REVERSE;
													 }
													 else{
														   asRuningInforCurrent.aListError |= ENCODER_ERR;
													 }
											 }
									}
									else{
											if(asRuningInforCurrent.aCurrent>CURRENT_DOW){										//Encoder have no conect
												 asRuningInforCurrent.aListError |= ENCODER_ERR;
											}
											else{																															//MOTOR Phase have no conect
												 asRuningInforCurrent.aListError |= PHASE_ERR;
											}
									}
									asStateMechine.asLoopControlStatus = CLOOP_STOOP;
									aFlagSuccess=0;
						}
						else if(absD(asRuningInforCurrent.aVelocity)>VELOCITY_MIN && aFlagSuccess!=1){
								if((asRuningInforCurrent.aPosition-asRuningInforOlder.aPosition)/(asRuningInforCurrent.aEstimatesValue-asRuningInforOlder.aPosition)>0){	
									aFlagSuccess =1;
								}
							
						}
				 }
				break;
	}	
	return asRuningInforCurrent.aListError;
}

void StateFeedback_PositionControl(void){ //ok                               //state Feedback : Dieu khien vi tri
	static char stopped =0;
	static double interW;	  //su dung de noi suy gia toc:
	static uint32_t index;
	
     if(asStateMechine.asLoopControlStatus == CLOOP_STARTING){
				double s12;
	      asStateMechine.asLoopControlStatus=CLOOP_PROCESS;   
    
				interW =absD(asRuningInforCurrent.aVelocity );
				stopped=0;
				asRuningInforCurrent.aEstimatesValue = asRuningInforCurrent.aPosition ;
				s12 = absD(asRuningInforSet.aPosition -asRuningInforCurrent.aPosition ) - asRuningInforSet.aVelocity *asRuningInforSet.aVelocity /asRuningInforSet.aAccel ;
			  if(s12>0){
					index = (uint32_t)((asRuningInforSet.aVelocity /asRuningInforSet.aAccel  + s12/asRuningInforSet.aVelocity )/DT);
				}
				else{
					index = (uint32_t)((sqrt(absD(asRuningInforSet.aPosition -asRuningInforCurrent.aPosition )/asRuningInforSet.aAccel ))/DT);
					
				}
     }
//Noi suy gia toc và van toc:
     if((asRuningInforSet.aPosition !=asRuningInforCurrent.aEstimatesValue) &&asStateMechine.asCmType != CM_PULSE){
///////////// /////////////////////     	  //noi suy gia toc:	
           if(index){//tang toc:
               interW+= asRuningInforSet.aAccel *DT;
               if(interW> asRuningInforSet.aVelocity ) interW = asRuningInforSet.aVelocity ;
               index--;
             }
           else{  //giam toc:
               interW-= asRuningInforSet.aAccel *DT;
               if(interW< VELOCITY_MIN) interW = VELOCITY_MIN;
           }
     ////////////////////////////////////		  //Noi suy van toc:
           denta = interW*DT   ;
           if((asRuningInforSet.aPosition  > asRuningInforCurrent.aPosition )){
                asRuningInforCurrent.aEstimatesValue+=denta;
                if(asRuningInforCurrent.aEstimatesValue>asRuningInforSet.aPosition ){asRuningInforCurrent.aEstimatesValue=asRuningInforSet.aPosition ;}
           }
           else if((asRuningInforSet.aPosition <asRuningInforCurrent.aPosition ))
           {    asRuningInforCurrent.aEstimatesValue-= denta;
                if(asRuningInforCurrent.aEstimatesValue<asRuningInforSet.aPosition ){asRuningInforCurrent.aEstimatesValue=asRuningInforSet.aPosition ;}
           }
      }
      else if((asRuningInforSet.aPosition ==asRuningInforCurrent.aEstimatesValue) && asStateMechine.asCmType != CM_PULSE){
				 //Tien trinh dieu khien theo uoc luoc da hoan tat:
				 asStateMechine.asLoopControlStatus = CLOOP_FINISH;
      }
      else{    //Control by Pulse:
				 asRuningInforCurrent.aEstimatesValue = (aExternCounterValue*6.283185307)/asDataHwInfor.aBaudrate;
      }

      aPositionErr = asRuningInforCurrent.aEstimatesValue-asRuningInforCurrent.aPosition ;
//xac dinh chieu va kiem tra sensor:
      if(aPositionErr>0){
	    	aMotorDir =1;
			if(SENSOR_RIGHT && stopped==0){
				 asRuningInforSet.aPosition   = asRuningInforCurrent.aPosition ;	//dung tai cho khong cho tiep tuc quay nua.
				 stopped=1;
				 return;
			}
	  }
      else if(aPositionErr<0){
	   		aMotorDir =0;
			if(SENSOR_LEFT && stopped==0){
				 asRuningInforSet.aPosition = asRuningInforCurrent.aPosition ;	//dung tai cho khong cho tiep tuc quay nua.	
				 stopped=1;
				 return;
			}
		
	  }
//Ham stateFeeback: u= asMotorInfor.aK1(asRuningInforSet.aPosition -asRuningInforCurrent.aPosition )-asMotorInfor.aK2*asRuningInforCurrent.aVelocity ;
//Khau P:
	   uP =asMotorInfor.aK1*(aPositionErr);
//Khau I:
	   uI= asMotorInfor.aK2*(asRuningInforCurrent.aVelocity );
//Khau gioi han bo I:
	  if(uI<-UI_MAX){
          uI= -UI_MAX;
      }
	  else if(uI>UI_MAX){
	   	  uI = UI_MAX;
	  }

    PID =  (int16_t)absF( uP-uI );

//	  if(PID<pBegin){
	  // 	PID+= pBegin;	 //cat bo khoang chet Pwm duoi
//	  }
	  //Khau gioi han Ham truyen:
    if(PID>1000)    PID=1000;
//Nap gia tri dieu khien vao bo dieu khien.
   DCM_UpdateDuty( PID, aMotorDir );
}

void PI_CurrentControl(void){
	float adcValue;
	  static float upV,uiV,upI,uiI, pidV_Out, pidI_Out;
	
    if(asStateMechine.asLoopControlStatus == CLOOP_STARTING){
					asStateMechine.asLoopControlStatus=CLOOP_PROCESS;  
          denta = absD(asRuningInforSet.aAccel )*DT;    
          asRuningInforCurrent.aEstimatesValue =  asRuningInforCurrent.aVelocity ;
    }
		//Read command from other periperal:
		if(asStateMechine.asCmType == CM_ANALOG){
				//Chieu DK phu thuoc vao chan aMotorDir:
				adcValue = READ_VelocitySetByADC();
				if(adcValue<10){
					adcValue =0;
				}
				
				if(EXT_DIR_STATE()){
						asRuningInforSet.aVelocity 	= asMotorInfor.aK3*(adcValue/4096.0);
				}
				else{
						asRuningInforSet.aVelocity 	= -1*asMotorInfor.aK3*(adcValue/4096.0);
				}
			//Cho van toc set bang khong neu tin hieu set qua nho:
				if(absD(asRuningInforSet.aVelocity)<1){
						asRuningInforSet.aVelocity =0;
				}										   
		}
	  else if(asStateMechine.asCmType == CM_PULSE){
				
		}
		
		//Ram velocity:
   if( asRuningInforCurrent.aEstimatesValue<asRuningInforSet.aVelocity ){
          asRuningInforCurrent.aEstimatesValue += denta;
          if(asRuningInforCurrent.aEstimatesValue>asRuningInforSet.aVelocity ){
             asRuningInforCurrent.aEstimatesValue=asRuningInforSet.aVelocity ;
						 asStateMechine.asLoopControlStatus=CLOOP_FINISH; 
          }
	 }
	 else if(asRuningInforCurrent.aEstimatesValue>asRuningInforSet.aVelocity ){
	  	asRuningInforCurrent.aEstimatesValue-= denta;
			if(asRuningInforCurrent.aEstimatesValue<asRuningInforSet.aVelocity ){
             asRuningInforCurrent.aEstimatesValue=asRuningInforSet.aVelocity ;
						 asStateMechine.asLoopControlStatus=CLOOP_FINISH; 
       }
	 }
	 //Calculate PI:
	aVelocityErr =  asRuningInforCurrent.aEstimatesValue -  asRuningInforCurrent.aCurrent;
		      		  	
 
	upI = asMotorCurrInfor.aKp_I*(aVelocityErr);
	uiI += asMotorCurrInfor.aKi_I*(aVelocityErr)*DT;
	  //gioi han uiV;
			if(uiI>UI_MAX){
				uiI = UI_MAX;
			}
			else if(uiI<-UI_MAX){
				uiI = -UI_MAX;
			}
		pidI_Out = upI + uiI;

		PID = absF(pidI_Out);	
		if(PID>999) PID =999;
			
		if( pidV_Out>0){

				 DCM_UpdateDuty( PID, 1 );
		}
		else if(pidV_Out<=0) {

				DCM_UpdateDuty( PID, 0 );
		}	        
	
}

void PI_ControlVelocity(void){	   //ok
float adcValue;
	  static float upV,uiV,upI,uiI, pidV_Out, pidI_Out;
	
    if(asStateMechine.asLoopControlStatus == CLOOP_STARTING){
					asStateMechine.asLoopControlStatus=CLOOP_PROCESS;  
          denta = absD(asRuningInforSet.aAccel )*DT;    
          asRuningInforCurrent.aEstimatesValue =  asRuningInforCurrent.aVelocity ;
    }
		//Read command from other periperal:
		if(asStateMechine.asCmType == CM_ANALOG){
				//Chieu DK phu thuoc vao chan aMotorDir:
				adcValue = READ_VelocitySetByADC();
				if(adcValue<10){
					adcValue =0;
				}
				
				if(EXT_DIR_STATE()){
						asRuningInforSet.aVelocity 	= asMotorInfor.aK3*(adcValue/4096.0);
				}
				else{
						asRuningInforSet.aVelocity 	= -1*asMotorInfor.aK3*(adcValue/4096.0);
				}
			//Cho van toc set bang khong neu tin hieu set qua nho:
				if(absD(asRuningInforSet.aVelocity)<2){
						asRuningInforSet.aVelocity =0;
				}										   
		}
	  else if(asStateMechine.asCmType == CM_PULSE){
				
		}
		
		//Ram velocity:
   if( asRuningInforCurrent.aEstimatesValue<asRuningInforSet.aVelocity ){
          asRuningInforCurrent.aEstimatesValue += denta;
          if(asRuningInforCurrent.aEstimatesValue>asRuningInforSet.aVelocity ){
             asRuningInforCurrent.aEstimatesValue=asRuningInforSet.aVelocity ;
						 asStateMechine.asLoopControlStatus=CLOOP_FINISH; 
          }
	 }
	 else if(asRuningInforCurrent.aEstimatesValue>asRuningInforSet.aVelocity ){
	  	asRuningInforCurrent.aEstimatesValue-= denta;
			if(asRuningInforCurrent.aEstimatesValue<asRuningInforSet.aVelocity ){
             asRuningInforCurrent.aEstimatesValue=asRuningInforSet.aVelocity ;
						 asStateMechine.asLoopControlStatus=CLOOP_FINISH; 
       }
	 }
	 else{
		 asStateMechine.asLoopControlStatus=CLOOP_FINISH; 
	 }
	 //Calculate PI:
	aVelocityErr =  asRuningInforCurrent.aEstimatesValue -  asRuningInforCurrent.aVelocity;
		      		  	
  upV = asMotorInfor.aK1*(aVelocityErr);
  uiV += asMotorInfor.aK2*(aVelocityErr)*DT;
	 
	  //gioi han uiV;
			if(uiV>UI_MAX){
				uiV = UI_MAX;
			}
			else if(uiV<-UI_MAX){
				uiV = -UI_MAX;
			}
		  pidV_Out = upV + uiV;
			
			
		//	PID 	 = pidV_Out;
			if(absF(pidV_Out)>PID_MAX){
				if(pidV_Out>0){
					pidV_Out = PID_MAX;
				}
				else pidV_Out = -PID_MAX;
			}
		
			
			//////////////////////////////////////////////////////////////////Current Control:
//			//gioi han dong dien vao:
//			if(pidV_Out>10000.0) pidV_Out =10000.0;
//			else if(pidV_Out<-10000)pidV_Out =-10000.0;	
//				
//		upI = asMotorCurrInfor.aKp_I*(absF(pidV_Out) - asRuningInforCurrent.aCurrent);
//		uiI += asMotorCurrInfor.aKi_I*(absF(pidV_Out) - asRuningInforCurrent.aCurrent)*DT;
//	  //gioi han uiV;
//			if(uiI>UI_MAX){
//				uiI = UI_MAX;
//			}
//			else if(uiI<-UI_MAX){
//				uiI = -UI_MAX;
//			}
//			
//		pidI_Out = upI + uiI;

//		PID = absF(pidI_Out);	
//		if(PID>999) PID =999;
			
		if( pidV_Out>0){

				 DCM_UpdateDuty( pidV_Out, 1 );
		}
		else if(pidV_Out<=0) {

				DCM_UpdateDuty( pidV_Out, 0 );
		}	        
}


void PI_ControlVelocityByADC(void){	   //ok
   double wSetTerm;
   	
//Chieu DK phu thuoc vao chan aMotorDir:
	if( MOTOR_ControlDir()){
			asRuningInforCurrent.aEstimatesValue 	= asMotorInfor.aK3*READ_VelocitySetByADC();
	}
	else{
			asRuningInforCurrent.aEstimatesValue 	= -1*asMotorInfor.aK3*READ_VelocitySetByADC();
	}
//Cho van to set bang khong neu tin hieu set qua nho:
	if(absD(asRuningInforSet.aEstimatesValue)<1){
			asRuningInforCurrent.aEstimatesValue =0;
	}

//////////////
	aVelocityErr =  asRuningInforCurrent.aEstimatesValue - asRuningInforCurrent.aVelocity;
		      		  
	uP= asMotorInfor.aK1*aVelocityErr;
	uI+= asMotorInfor.aK2*(aVelocityErr)*DT;

//Khau gioi han bo I:
	  if(uI<-UI_MAX){
          uI= -UI_MAX;
      }
	  else if(uI>UI_MAX){
	   	  uI = UI_MAX;
	  }	
///////////////////////
	PID = uP+uI;
    if(PID>1000){
	   	PID =1000; //gioi han
	}
	if(PID<-1000)	PID=-1000;						//khau tich phan nen phai nhan them DT =0.001s;
//kiem tra sensor:

	 if( PID>0){
		 if(SENSOR_RIGHT){
				 DCM_UpdateDuty( 0, 1 );
				 return;
		 }
		 DCM_UpdateDuty( PID, 1 );
   }
   else if(PID<=0) {
		  if(SENSOR_LEFT){	
		  		DCM_UpdateDuty( 0, 0 );
				return;
		  }
      DCM_UpdateDuty( -1*PID, 0 );
   }	    
}
void PID_PositionControl(void){			  //ok
	float kTerm;
	static char stopped =0;
	static double interW;	  //su dung de noi suy gia toc:
	static uint32_t index;
	static float pidFirts, overPID;
	
     if(asStateMechine.asLoopControlStatus ==CLOOP_STARTING){
				double s12;
	      asStateMechine.asLoopControlStatus = CLOOP_PROCESS;
				interW =absD(asRuningInforCurrent.aVelocity );
				stopped=0;
				asRuningInforCurrent.aEstimatesValue = asRuningInforCurrent.aPosition ;
				s12 = absD(asRuningInforSet.aPosition -asRuningInforCurrent.aPosition ) - asRuningInforSet.aVelocity *asRuningInforSet.aVelocity /asRuningInforSet.aAccel ;
        if(s12>0){
					index = (uint32_t)((asRuningInforSet.aVelocity /asRuningInforSet.aAccel  + s12/asRuningInforSet.aVelocity )/DT);
				}
				else{
					index = (uint32_t)((sqrt(absD(asRuningInforSet.aPosition -asRuningInforCurrent.aPosition )/asRuningInforSet.aAccel ))/DT);
				}
     }
//Noi suy gia toc và van toc khi dung mang hoac USB dieu khien:
     if((asRuningInforSet.aPosition !=asRuningInforCurrent.aEstimatesValue && asStateMechine.asCmType != CM_PULSE)){
///////////// /////////////////////     	  //noi suy gia toc:	
					if(index){//tang toc:
							interW+= asRuningInforSet.aAccel *DT;
							if(interW> asRuningInforSet.aVelocity ) interW = asRuningInforSet.aVelocity ;
							index--;
						}
					else{  //giam toc:
							interW-= asRuningInforSet.aAccel *DT;
							if(interW< VELOCITY_MIN) interW = VELOCITY_MIN;
					}
	////////////////////////////////////		  //Noi suy van toc:
					denta = interW*DT   ;
					if((asRuningInforSet.aPosition  >= asRuningInforCurrent.aPosition )){
							asRuningInforCurrent.aEstimatesValue+=denta;
							if(asRuningInforCurrent.aEstimatesValue>asRuningInforSet.aPosition )
							{
								asRuningInforCurrent.aEstimatesValue=asRuningInforSet.aPosition ;				
							}
					 }
					 else if((asRuningInforSet.aPosition <= asRuningInforCurrent.aPosition ))
					 {  
						  asRuningInforCurrent.aEstimatesValue -= denta;
							if(asRuningInforCurrent.aEstimatesValue < asRuningInforSet.aPosition )
							{
								asRuningInforCurrent.aEstimatesValue=asRuningInforSet.aPosition ;
							}
					 }
      }
      else if(asRuningInforSet.aPosition ==asRuningInforCurrent.aEstimatesValue && asStateMechine.asCmType != CM_PULSE){
         //Tien trinh dieu khien theo uoc luoc da hoan tat:
					asStateMechine.asLoopControlStatus = CLOOP_FINISH;
      }
      else{    //Control by Pulse:
				 asRuningInforCurrent.aEstimatesValue = (aExternCounterValue*6.283185307)/asDataHwInfor.aBaudrate;
      }
	    aPositionOldErr = aPositionErr;
      aPositionErr = asRuningInforCurrent.aEstimatesValue-asRuningInforCurrent.aPosition ;
//xac dinh chieu va kiem tra sensor:
      if((aPositionErr)>0){
	    	aMotorDir =1;
			if(SENSOR_RIGHT && stopped==0){
				 asRuningInforSet.aPosition   = asRuningInforCurrent.aPosition ;	//dung tai cho khong cho tiep tuc quay nua.
				 stopped=1;
				 return;
			}
	  }
      else if(aPositionErr<0){
	   		aMotorDir =0;
			if(SENSOR_LEFT && stopped==0){
				 asRuningInforSet.aPosition = asRuningInforCurrent.aPosition ;	//dung tai cho khong cho tiep tuc quay nua.	
				 stopped=1;
				 return;
			}
	  }

//Khau ti le:
     uP =asMotorInfor.aK1*(aPositionErr);				//asMotorInfor.aK1*e(t);			 e(t): la ham sai soi
//Khau tich phan:
	  uI+= (asMotorInfor.aK2 )*(aPositionErr)*DT;//(asMotorInfor.aK2+ overPID*asMotorCurrInfor.aKi_P  )*(aPositionErr)*DT;				//asMotorInfor.aK2*e(t)*DT;	thoi gian lay mau la DT
//Khau vi phan:
	  uD = asMotorInfor.aK3*(aPositionErr-aPositionOldErr)/DT;  //k3*de(t)/DT;  vi phan cua ham e(t) chia cho denta t
//Gioi han bo tich phan lai: 	     
      if(absF(uI)>UI_MAX){
           if(uI<0) uI =-UI_MAX;
					 else uI = UI_MAX;
      }
			///////////////////////////////////////////////////////////////modifile:
			
//			// su dung antiwinup:
//			pidFirts = uP+uI+uD;
//		  
//			if( pidFirts>0){
//				if(pidFirts>PID_MAX) PID = PID_MAX;
//				if(pidFirts<PID_MIN) PID = PID_MIN;
//				else PID = pidFirts;
//			}
//			else if(pidFirts<0){
//				if(pidFirts<-PID_MAX) PID = -PID_MAX;
//				if(pidFirts>-PID_MIN) PID = -PID_MIN;
//				else PID = pidFirts;
//			}
//			else{
//				PID = (int16_t)pidFirts;
//			}	
//			overPID = PID-pidFirts;
			
//    PID =(int16_t)absF( uP+uI+uD );
//    if(PID>PID_MAX)    PID=PID_MAX;
//     DCM_UpdateDuty( PID, aMotorDir );
			
		PID =(int16_t)(uP+uI+uD );
    if(absF(PID)>PID_MAX){
			if(PID>PID_MAX) PID= PID_MAX;
			else PID =-PID_MAX;
		}

		
		if(PID>0){
			 DCM_UpdateDuty( absF(PID), 1 );
		}
		else{
			 DCM_UpdateDuty( absF(PID), 0 );
		}
   
}

void PidPiPi_PositionControl(void){			  //ok
	float kTerm;
	static char stopped =0;
	static double interW;	  //su dung de noi suy gia toc:
	static uint32_t index;
	static float upP,uiP,udP,upV,uiV,udV,upI,uiI, pidP_Out,pidV_Out,pidI_Out;
	static float velocityError,aVelocityOldErr;
	static float kAnti;
	
     if(asStateMechine.asLoopControlStatus ==CLOOP_STARTING){
				double s12;
	      asStateMechine.asLoopControlStatus = CLOOP_PROCESS;
				interW =absD(asRuningInforCurrent.aVelocity );
				stopped=0;
				asRuningInforCurrent.aEstimatesValue = asRuningInforCurrent.aPosition ;
				s12 = absD(asRuningInforSet.aPosition -asRuningInforCurrent.aPosition ) - asRuningInforSet.aVelocity *asRuningInforSet.aVelocity /asRuningInforSet.aAccel ;
        if(s12>0){
					index = (uint32_t)((asRuningInforSet.aVelocity /asRuningInforSet.aAccel  + s12/asRuningInforSet.aVelocity )/DT);
				}
				else{
					index = (uint32_t)((sqrt(absD(asRuningInforSet.aPosition -asRuningInforCurrent.aPosition )/asRuningInforSet.aAccel ))/DT);
				}
     }
//Noi suy gia toc và van toc khi dung mang hoac USB dieu khien:
     if((asRuningInforSet.aPosition !=asRuningInforCurrent.aEstimatesValue && asStateMechine.asCmType != CM_PULSE)){
///////////// /////////////////////     	  //noi suy gia toc:	
					if(index){//tang toc:
							interW+= asRuningInforSet.aAccel *DT;
							if(interW> asRuningInforSet.aVelocity ) interW = asRuningInforSet.aVelocity ;
							index--;
						}
					else{  //giam toc:
							interW-= asRuningInforSet.aAccel *DT;
							if(interW< VELOCITY_MIN) interW = VELOCITY_MIN;
					}
	////////////////////////////////////		  //Noi suy van toc:
					denta = interW*DT   ;
					if((asRuningInforSet.aPosition  >= asRuningInforCurrent.aPosition )){
							asRuningInforCurrent.aEstimatesValue+=denta;
							if(asRuningInforCurrent.aEstimatesValue>asRuningInforSet.aPosition )
							{
								asRuningInforCurrent.aEstimatesValue=asRuningInforSet.aPosition ;				
							}
					 }
					 else if((asRuningInforSet.aPosition <= asRuningInforCurrent.aPosition ))
					 {  
						  asRuningInforCurrent.aEstimatesValue -= denta;
							if(asRuningInforCurrent.aEstimatesValue < asRuningInforSet.aPosition )
							{
								asRuningInforCurrent.aEstimatesValue=asRuningInforSet.aPosition ;
							}
					 }
      }
      else if(asRuningInforSet.aPosition ==asRuningInforCurrent.aEstimatesValue && asStateMechine.asCmType != CM_PULSE){
         //Tien trinh dieu khien theo uoc luoc da hoan tat:
					asStateMechine.asLoopControlStatus = CLOOP_FINISH;
      }
      else{    //Control by Pulse:
				 asRuningInforCurrent.aEstimatesValue = (aExternCounterValue*6.283185307)/asDataHwInfor.aBaudrate;
      }
	    aPositionOldErr = aPositionErr;
      aPositionErr = asRuningInforCurrent.aEstimatesValue-asRuningInforCurrent.aPosition ;

////xac dinh chieu va kiem tra sensor:
//      if((aPositionErr)>0){
//	    	aMotorDir =1;
//			if(SENSOR_RIGHT && stopped==0){
//				 asRuningInforSet.aPosition   = asRuningInforCurrent.aPosition ;	//dung tai cho khong cho tiep tuc quay nua.
//				 stopped=1;
//				 return;
//			}
//	  }
//      else if(aPositionErr<0){
//	   		aMotorDir =0;
//			if(SENSOR_LEFT && stopped==0){
//				 asRuningInforSet.aPosition = asRuningInforCurrent.aPosition ;	//dung tai cho khong cho tiep tuc quay nua.	
//				 stopped=1;
//				 return;
//			}
//	  }
/////////////////////////////////////////////////////////////////////////
//Khau ti le:
     upP = asMotorCurrInfor.aKp_P*(aPositionErr);				//asMotorInfor.aK1*e(t);			 e(t): la ham sai soi
//Khau tich phan:
	 // uiP+= asMotorCurrInfor.aKi_P*(aPositionErr)*DT;				//asMotorInfor.aK2*e(t)*DT;	thoi gian lay mau la DT
//Khau vi phan:
	  udP = asMotorCurrInfor.aKd_P*(aPositionErr-aPositionOldErr)/DT;  //k3*de(t)/DT;  vi phan cua ham e(t) chia cho denta t
//Gioi han bo tich phan lai: 	     
//      if(absF(uiP)>UI_MAX){
//           if(uiP<0) uiP =-UI_MAX;
//					 else uiP = UI_MAX;
//      }
			
			pidP_Out 		= upP+udP;
			//Gioi han van toc:
			if(pidP_Out>VELOCITY_MAX){
				 pidP_Out = VELOCITY_MAX;
			}
			else if(pidP_Out<-VELOCITY_MAX) pidP_Out =-VELOCITY_MAX;
///////////////////////////////////////////////////////////////Velocity control:
//			if(absF(pidP_Out)< 0.5){
//				pidP_Out =0;
//			}
		//aVelocityOldErr = velocityError;
		velocityError = pidP_Out - asRuningInforCurrent.aVelocity;
			
		upV = asMotorCurrInfor.aKp_V*(velocityError);
		uiV += asMotorCurrInfor.aKi_V*(velocityError)*DT;
		//udV = asMotorCurrInfor.aKd_V*aVelocityOldErr/DT;
			
//			//Antiwin-up:
//		upV = asMotorCurrInfor.aKp_V*(velocityError); 
//		uiV += (asMotorCurrInfor.aKi_V +kAnti)*(velocityError)*DT;
		
	  //gioi han uiV;
		if(uiV>UI_MAX){
				uiV = UI_MAX;
		}
		else if(uiV<-UI_MAX){
				uiV = -UI_MAX;
		}
		pidV_Out = upV + uiV ;//+udV;	
			
		if(absF(pidV_Out)>PID_MAX){
				if(pidV_Out>0){
					pidV_Out = PID_MAX;
				}
				else pidV_Out = -PID_MAX;
		}
		
		//kAnti = asMotorInfor.aK1*(pidV_Out - (upV+uiI));	
			
//			//////////////////////////////////////////////////////////////////Current Control:
//			//gioi han dong dien vao:
//			if(pidV_Out>10000.0) pidV_Out =10000.0;
//			else if(pidV_Out<-10000)pidV_Out =-10000.0;	
//				
//		upI = asMotorCurrInfor.aKp_I*(absF(pidV_Out) - asRuningInforCurrent.aCurrent);
//		uiI += asMotorCurrInfor.aKi_I*(absF(pidV_Out) - asRuningInforCurrent.aCurrent)*DT;
//	  //gioi han uiV;
//			if(uiI>UI_MAX){
//				uiI = UI_MAX;
//			}
//			else if(uiI<-UI_MAX){
//				uiI = -UI_MAX;
//			}
//			
//		pidI_Out = upI + uiI;

//		PID = absF(pidI_Out);	
//		if(PID>999) PID =999;
//			
//		if( pidV_Out>0){

//				 DCM_UpdateDuty( PID, 1 );
//		}
//		else if(pidV_Out<=0) {

//				DCM_UpdateDuty( PID, 0 );
//		 }	    

	if( pidV_Out>0){
			 if(SENSOR_RIGHT){
					 DCM_UpdateDuty( 0, 1 );
					 return;
			 }
			 DCM_UpdateDuty( pidV_Out, 1 );
	}
	else if(pidV_Out<=0) {
			if(SENSOR_LEFT){	
					DCM_UpdateDuty( 0, 0 );
					return;
			}
			DCM_UpdateDuty( -1*pidV_Out, 0 );
   }	    
}

void PID_PositionControlCase2(void){			  //ok
	static char stopped =0;
	static double interW;	  //su dung de noi suy gia toc:
	static float pidOld,pid,a0,a1,a2;

	
	static uint32_t index;
	
     if(asStateMechine.asLoopControlStatus ==CLOOP_STARTING){
				double s12;
	      asStateMechine.asLoopControlStatus = CLOOP_PROCESS;
				interW =absD(asRuningInforCurrent.aVelocity );
				stopped=0;
			 
			  a0 = asMotorInfor.aK1 + asMotorInfor.aK2*DT/2 +asMotorInfor.aK3/DT;
				a1 = -asMotorInfor.aK1 + asMotorInfor.aK2*DT/2 -2*asMotorInfor.aK3/DT;
				a2 = asMotorInfor.aK3/DT;
			 
				asRuningInforCurrent.aEstimatesValue = asRuningInforCurrent.aPosition ;
				s12 = absD(asRuningInforSet.aPosition -asRuningInforCurrent.aPosition ) - asRuningInforSet.aVelocity *asRuningInforSet.aVelocity /asRuningInforSet.aAccel ;
        if(s12>0){
					index = (uint32_t)((asRuningInforSet.aVelocity /asRuningInforSet.aAccel  + s12/asRuningInforSet.aVelocity )/DT);
				}
				else{
					index = (uint32_t)((sqrt(absD(asRuningInforSet.aPosition -asRuningInforCurrent.aPosition )/asRuningInforSet.aAccel ))/DT);
				}
     }
//Noi suy gia toc và van toc khi dung mang hoac USB dieu khien:
     if((asRuningInforSet.aPosition !=asRuningInforCurrent.aEstimatesValue && asStateMechine.asCmType != CM_PULSE)){
///////////// /////////////////////     	  //noi suy gia toc:	
					if(index){//tang toc:
							interW+= asRuningInforSet.aAccel *DT;
							if(interW> asRuningInforSet.aVelocity ) interW = asRuningInforSet.aVelocity ;
							index--;
						}
					else{  //giam toc:
							interW-= asRuningInforSet.aAccel *DT;
							if(interW< VELOCITY_MIN) interW = VELOCITY_MIN;
					}
	////////////////////////////////////		  //Noi suy van toc:
					denta = interW*DT   ;
					if((asRuningInforSet.aPosition  >= asRuningInforCurrent.aPosition )){
							asRuningInforCurrent.aEstimatesValue+=denta;
							if(asRuningInforCurrent.aEstimatesValue>asRuningInforSet.aPosition )
							{
								asRuningInforCurrent.aEstimatesValue=asRuningInforSet.aPosition ;				
							}
					 }
					 else if((asRuningInforSet.aPosition <= asRuningInforCurrent.aPosition ))
					 {  
						  asRuningInforCurrent.aEstimatesValue -= denta;
							if(asRuningInforCurrent.aEstimatesValue < asRuningInforSet.aPosition )
							{
								asRuningInforCurrent.aEstimatesValue=asRuningInforSet.aPosition ;
							}
					 }
      }
      else if(asRuningInforSet.aPosition ==asRuningInforCurrent.aEstimatesValue && asStateMechine.asCmType != CM_PULSE){
         //Tien trinh dieu khien theo uoc luoc da hoan tat:
					asStateMechine.asLoopControlStatus = CLOOP_FINISH;
      }
      else{    //Control by Pulse:
				 asRuningInforCurrent.aEstimatesValue = (aExternCounterValue*6.283185307)/asDataHwInfor.aBaudrate;
      }
			aPositionOlderErr = aPositionOldErr;
	    aPositionOldErr = aPositionErr;
      aPositionErr = (float)(asRuningInforCurrent.aEstimatesValue-asRuningInforCurrent.aPosition) ;
//xac dinh chieu va kiem tra sensor:
//      if((aPositionErr)>0){
//	    	aMotorDir =1;
//			if(SENSOR_RIGHT && stopped==0){
//				 asRuningInforSet.aPosition   = asRuningInforCurrent.aPosition ;	//dung tai cho khong cho tiep tuc quay nua.
//				 stopped=1;
//				 return;
//			}
//	  }
//      else if(aPositionErr<0){
//	   		aMotorDir =0;
//			if(SENSOR_LEFT && stopped==0){
//				 asRuningInforSet.aPosition = asRuningInforCurrent.aPosition ;	//dung tai cho khong cho tiep tuc quay nua.	
//				 stopped=1;
//				 return;
//			}
//	  }
			//
		pid += a0*aPositionErr + a1*aPositionOldErr + a2*aPositionOlderErr;
		if(absF(pid)>PID_MAX){
			if(pid>PID_MAX) pid = PID_MAX;
			else if(pid<PID_MAX) pid= -PID_MAX;
			
		}

//    PID =(uint16_t)absF( pid );
//    if(PID>PID_MAX)    PID= (uint16_t)PID_MAX;
//    DCM_UpdateDuty( PID, aMotorDir );

//	// su dung antiwinup:
//			pidFirts = uP+uI+uD;
//		  
//			if( pidFirts>0){
//				if(pidFirts>PID_MAX) PID = PID_MAX;
//				if(pidFirts<PID_MIN) PID = 0;
//				else PID = pidFirts;
//			}
//			else if(pidFirts<0){
//				if(pidFirts<-PID_MAX) PID = -PID_MAX;
//				if(pidFirts>-PID_MIN) PID = 0;
//				else PID = pidFirts;
//			}
//			else{
//				PID = (int16_t)pidFirts;
//			}
//			
//			overPID = PID-pidFirts;
////    PID =(int16_t)absF( uP+uI+uD );
////    if(PID>PID_MAX)    PID=PID_MAX;
////     DCM_UpdateDuty( PID, aMotorDir );
////			
////		PID =(int16_t)(uP+uI+uD );
////    if(absF(PID)>PID_MAX){
////			if(PID>PID_MAX) PID= PID_MAX;
////			else PID =-PID_MAX;
////		}

		PID = pid;
		if(PID>0){
			 DCM_UpdateDuty( absF(PID), 1 );
		}
		else{
			 DCM_UpdateDuty( absF(PID), 0 );
		}
}
////////////////////////////////////////////
unsigned int quick_sort(float *a, unsigned int size)
{
	unsigned int i,j;
	float temp;

	for (i = 0; i < size -1; i++)
	{
		for (j = i + 1; j < size; j++)
		{
			if (a[i] < a[j])
			{
				temp = a[i];
				a[i] = a[j];
				a[j] = temp;
			}
		}
	}

	return 0;
}
unsigned int quick_sortInt(unsigned int *a, unsigned int size)
{
	unsigned int i,j;
	unsigned temp;

	for (i = 0; i < size -1; i++)
	{
		for (j = i + 1; j < size; j++)
		{
			if (a[i] < a[j])
			{
				temp = a[i];
				a[i] = a[j];
				a[j] = temp;
			}
		}
	}

	return 0;
}

float abs_def(float a)
{
	if (a > 0)
		return a;
	else return -a;
}

unsigned int cubic_equation_solver(float a, float b, float c, float d, float *x)
{
	float disc, q, r, dum1, term1, r13;

    b /= a;
    c /= a;
    d /= a;

    q = (3*c - (b*b))/9;
    r = -(27*d) + b*(9*c - 2*(b*b));
    r /= 54;
    disc = q*q*q + r*r;
    term1 = (b/3);

    if (disc > 0)
    {
    	return 1;
    }

    else if (disc == 0)
    {
    	r13 = ((r < 0) ? -pow(-r,(1/3)) : pow(r,(1/3)));
		x[0] = -term1 + 2*r13;
		x[2] = x[1] = -(r13 + term1);
    }
    else
    {
        q = -q;
        dum1 = q*q*q;
        dum1 = acos(r/sqrt(dum1));
        r13 = 2*sqrt(q);
        x[0] = -term1 + r13*cos(dum1/3);
        x[1] = -term1 + r13*cos((dum1 + 2*3.14159)/3);
        x[2] = -term1 + r13*cos((dum1 + 4*3.14159)/3);
    }

    return 0;
}

unsigned int Tunning_Position_Veloctiy_Controller(float J, float B, float *vKp, float *vKi, float *pKp, float *pKd, float pole)
{
	unsigned int i, k, ret;

	float 	vel_z = 0,vel_p1 = 0,vel_p2 = 0;

	float 	pos_z[2] = {0}, pos_p[3] = {0};

	float vel_epsilon = MIN_VEL_EPSILON, pos_epsilon;
	unsigned int vel_omega = 0, max_vel_omega, pos_omega;

	float a, b, c, d;

	float vel_ep_store[200] =  {0};
	unsigned int vel_o_store[200] = {0}, pos_o_store[300] = {0};
	float delta_p_z_2[300] = {0}, delta_temp[300] = {0};

		// Find  Vel_Omega maximum
		while (vel_p2 <= pole)
		{
			vel_omega += 5;
			vel_p2 = vel_epsilon*vel_omega + vel_omega* sqrt(vel_epsilon*vel_epsilon-1);
		}

		max_vel_omega = vel_omega;

		i =  0;
		for (vel_omega = 5; vel_omega < max_vel_omega; vel_omega += 5)
		{
			while ((vel_p1 > vel_z + 0.5) || (vel_p1 <= vel_z) || vel_p2 < 0.9 * pole || vel_p2 > pole)
			{
				vel_epsilon  +=  0.1;
				vel_z = J * vel_omega*vel_omega/ (2*J*vel_epsilon*vel_omega-B);
				vel_p1 = vel_epsilon*vel_omega - vel_omega* sqrt(vel_epsilon*vel_epsilon-1);
				vel_p2 = vel_epsilon*vel_omega + vel_omega* sqrt(vel_epsilon*vel_epsilon-1);

				if (vel_epsilon >= MAX_VEL_EPSILON)
				{
					break;
				}
			}

			vel_p1 = 0;
			vel_z = 0;

			if  (vel_epsilon >= MAX_VEL_EPSILON)
			{
				vel_epsilon = MIN_VEL_EPSILON;
				continue;
			}

			vel_ep_store[i] = vel_epsilon;
			vel_o_store[i] = vel_omega;
			i++;

			vel_epsilon = MIN_VEL_EPSILON;

		}

		vel_epsilon = vel_ep_store[i/2];
		vel_omega = vel_o_store[i/2];

		*vKi = J * vel_omega * vel_omega;
		*vKp = 2 * J * vel_epsilon * vel_omega - B;
		////////////////////////////////////////////

		pos_epsilon = vel_epsilon;
		pos_omega = vel_omega;


		i = 0;
		while (pos_p[2] > -10 * pole && pos_omega<10000)
		{
				pos_omega += 10;

				pos_z[0] = - *vKi / *vKp;
				pos_z[1] = - (J * pos_omega * pos_omega) / (2* J * pos_epsilon * pos_omega - B);
				quick_sort(pos_z, 2);

				a = J;
				b = *vKp + B + *vKp * (2 * J * pos_epsilon * pos_omega - B);
				c = *vKi + *vKp * J * pos_omega * pos_omega + *vKi * (2 * J * pos_epsilon * pos_omega - B);
				d = *vKi * J * pos_omega * pos_omega;

				ret = cubic_equation_solver(a, b, c, d, pos_p);

				if (ret == 1)
				{
					continue;
				}
				quick_sort(pos_p, 3);

				if ((abs_def(pos_z[0] - pos_p[0]) < 3) &&  (pos_p[2] < -10 * pos_p[1]) && (pos_p[1] > - pole / 3))
				{
					pos_o_store[i] = pos_omega;
					delta_p_z_2[i] = abs_def(pos_z[1] - pos_p[1]);
					delta_temp[i] = delta_p_z_2[i];
					i++;
				}
		}

		if (i == 0)
		{
			*pKp =0;
			*pKd =0;
			*vKp =0;
			*vKi =0;
			return 1;
		}

//		quick_sort(delta_temp, i);
//		for (k = 0; k < i; k++)
//		{
//			if (delta_p_z_2[k] == delta_temp[i-1])
//			{
//				pos_omega = pos_o_store[k];
//			}
//		}

		//edit:
		quick_sortInt(pos_o_store, i);
		pos_omega = pos_o_store[i/2];

		*pKp = J * pos_omega * pos_omega;
		*pKd = 2* J * pos_epsilon * pos_omega - B;

	return 0;
}


/////////////////////////////////////////////

void LOOP_AutoTunning(void){			 	//Ham nay da ok,
	  static double sumAI,sumVI,sumDutyI, aM,vM, dutyM;
	  static uint16_t countN=0 ,countT=0;
		
	  if(asStateMechine.asLoopControlStatus ==CLOOP_STARTING){	//Init for Tuning
	  	 countT=0;
       asStateMechine.asLoopControlStatus = CLOOP_PROCESS;
			 sumAI= 0;sumVI=0;sumDutyI=0;
       asMotorInfor.aB =0;
       asMotorInfor.aJ =0;
			
			 
			
			 countN=0;
			 countT=0;

			 DCM_UpdateDuty(800 , 1 );		 //////////////////////////
			 //return;
	  }
		if(asStateMechine.asLoopControlStatus == CLOOP_PROCESS){
					if(++countT <2000 && countN==800){
					 DCM_UpdateDuty( 0, 3 );
					 return ;
					}  
					if(countN==800){
						DCM_UpdateDuty(800 , 0 );			 //Tim thong so chieu nguoc lai:
					}
					if(asStateMechine.asLoopControlStatus == CLOOP_PROCESS){
							countN++;
							if( countN<1600){	
									sumDutyI += 24;//0.8*3.3 (gia tri ly thuyet);		 //%duty   Theo kinh nghiem thi gia tri nay bao nhieu cung dc, no chi lam cho he j va b  thay doi theo mot ham nao do ma thoi
							
									sumAI +=  absD(asRuningInforCurrent.aAccel );
									sumVI +=	absD(asRuningInforCurrent.aVelocity) ;
									if(countN==650 ){			 //lay gia tri tai thoi diem bat ky
										 aM = absD(asRuningInforCurrent.aAccel );
										 vM = absD(asRuningInforCurrent.aVelocity) ;
										 dutyM = 24;					//0.8*3.3;
									}
								}
					 else if(countN==1600 ){				  //Tunning finished:
								 DCM_UpdateDuty( 0, 3 ); 
								 asMotorInfor.aB = (double)((aM*sumDutyI - dutyM*sumAI)/(aM*sumVI - vM*sumAI));
								 asMotorInfor.aJ = ((double)dutyM - asMotorInfor.aB*vM)/aM;
						 
								 
									memset(&asMotorCurrInfor,0,sizeof(asMotorCurrInfor));
						 
						     Tunning_Position_Veloctiy_Controller(asMotorInfor.aJ, asMotorInfor.aB,&asMotorCurrInfor.aKp_V,&asMotorCurrInfor.aKi_V,&asMotorCurrInfor.aKp_P,&asMotorCurrInfor.aKd_P,30*asMotorInfor.aB/asMotorInfor.aJ);
						 
								 asStateMechine.asLoopControlStatus = CLOOP_FINISH;			   
					}
				}	
			} 
}
 
void PV_eControl(void){


}

double wFilter(double wfilter){
 	static uint8_t index;
	static double wArray[5];
	if(index==4){
	 	index =0;
	}
	wArray[index++] = wfilter;

	return (wArray[0]+wArray[1]+wArray[2]+wArray[3])/4 ;

}
void LoopEnable(){
	aLoopEnable=1;
}

// Phien: velocity control
void Current_Control(unsigned char mode)
{
	
	

	if (mode == POS_MODE)
	{
		asRuningInforCurrent.aEstimatesValue = PID_Position_Current_Control();
	}
	
	cur_ref = PI_Velocity_Current_Control(mode);
	aCurrentErr = cur_ref - asRuningInforCurrent.aCurrent;
	
	uP= asMotorCurrInfor.aKp_I*aCurrentErr;
	uI+= asMotorCurrInfor.aKi_I*(aCurrentErr)*DT;
	

//Khau gioi han bo I:
	if(uI<-UI_MAX){
      uI= -UI_MAX;
  }
	else if(uI>UI_MAX){
	   	uI = UI_MAX;
	}	
	
	PID = uP+uI;
	
/////////////////////////////////////
  if(PID>1000){
	   	PID =1000; //gioi han
	}
	if(PID<-1000)	PID=-1000;						//khau tich phan nen phai nhan them DT =0.001s;
//kiem tra sensor:

	if( PID>0){
			 if(SENSOR_RIGHT){
					 DCM_UpdateDuty( 0, 1 );
					 return;
			 }
			 DCM_UpdateDuty( PID, 1 );
	}
	else if(PID<=0) {
			if(SENSOR_LEFT){	
					DCM_UpdateDuty( 0, 0 );
					return;
			}
			DCM_UpdateDuty( -1*PID, 0 );
   }	
}

// Phien: current control for velocity control
float PI_Velocity_Current_Control(unsigned char mode)
{
	static float pid,a0,a1,a2;
	
    if(asStateMechine.asLoopControlStatus == CLOOP_STARTING){
					asStateMechine.asLoopControlStatus=CLOOP_PROCESS;  
          denta = absD(asRuningInforSet.aAccel )*DT;    
          asRuningInforCurrent.aEstimatesValue =  asRuningInforCurrent.aVelocity ;
			
			  a0 = asMotorCurrInfor.aKp_V + asMotorCurrInfor.aKi_V*DT/2;// +asMotorCurrInfor.aKd_V/DT;
				a1 = -asMotorCurrInfor.aKp_V + asMotorCurrInfor.aKi_V*DT/2;// -2*asMotorCurrInfor.aKd_V/DT;
				//a2 = asMotorCurrInfor.aKd_V/DT;
    }
		//Read command from other periperal:
		if(asStateMechine.asCmType == CM_ANALOG){
				//Chieu DK phu thuoc vao chan aMotorDir:
				if(EXT_DIR_STATE()){
						asRuningInforSet.aVelocity 	= asMotorInfor.aK3*READ_VelocitySetByADC();
				}
				else{
						asRuningInforSet.aVelocity 	= -1*asMotorInfor.aK3*READ_VelocitySetByADC();
				}
			//Cho van toc set bang khong neu tin hieu set qua nho:
				if(absD(asRuningInforSet.aVelocity)<1){
						asRuningInforSet.aVelocity =0;
				}										   
		}
	  else if(asStateMechine.asCmType == CM_PULSE){
				
		}
		asStateMechine.asLoopControlStatus=CLOOP_FINISH; 
//	 if (mode == VEL_MODE)	// Velocity control mode
//	 {
//		 	//Ram velocity:
//			if( asRuningInforCurrent.aEstimatesValue<asRuningInforSet.aVelocity )
//			{
//          asRuningInforCurrent.aEstimatesValue += denta;
//          if(asRuningInforCurrent.aEstimatesValue>asRuningInforSet.aVelocity )
//					{
//             asRuningInforCurrent.aEstimatesValue=asRuningInforSet.aVelocity ;
//						 asStateMechine.asLoopControlStatus=CLOOP_FINISH; 
//          }
//			}
//			else if(asRuningInforCurrent.aEstimatesValue>asRuningInforSet.aVelocity )
//			{
//				asRuningInforCurrent.aEstimatesValue-= denta;
//				if(asRuningInforCurrent.aEstimatesValue<asRuningInforSet.aVelocity )
//					{
//             asRuningInforCurrent.aEstimatesValue=asRuningInforSet.aVelocity ;
//						 asStateMechine.asLoopControlStatus=CLOOP_FINISH; 
//					}
//			}	
//	 }
//	 
//	 else
//	 {
//		  asStateMechine.asLoopControlStatus=CLOOP_FINISH; 
//	 }
	 //Tam de o day them:
   a0 = asMotorCurrInfor.aKp_V + asMotorCurrInfor.aKi_V*DT/2;// +asMotorCurrInfor.aKd_V/DT;
   a1 = -asMotorCurrInfor.aKp_V + asMotorCurrInfor.aKi_V*DT/2;//
	 //Calculate PI:
	 aVelocityOldErr = aVelocityErr;
	 aVelocityErr =  asRuningInforSet.aVelocity -  asRuningInforCurrent.aVelocity;
	 
	 //pid += a0*aVelocityErr + a1*aVelocityOldErr;// + a2*aPositionOlderErr;
	 asRuningInforCurrent.aEstimatesValue=asRuningInforSet.aVelocity ;
		pid = asRuningInforSet.aVelocity;
		if (pid > CUR_MAX)
		{
			pid = CUR_MAX;
		}
		return pid;
		      		  
	//uP= asMotorCurrInfor.aKp_V *aVelocityErr;
	//uI+= asMotorCurrInfor.aKi_V*(aVelocityErr)*DT;

//Khau gioi han bo I:
	//if(uI<-UI_MAX){
     // uI= -UI_MAX;
 // }
	//else if(uI>UI_MAX){
	 //  	uI = UI_MAX;
	//}	
	
		//PID = uP+uI;
/////////////////////////////////////
  //if(PID>CUR_MAX){
	   	//PID =CUR_MAX; //gioi han
	//}
	//if(PID<-CUR_MAX)	PID=-CUR_MAX;						//khau tich phan nen phai nhan them DT =0.001s;

	//return PID;
}

//// Phien: Position control with current control
float PID_Position_Current_Control(void)
{
	static char stopped =0;
	static double interW;	  //su dung de noi suy gia toc:
	static uint32_t index;
	static float pid,a0,a1,a2;
	
     if(asStateMechine.asLoopControlStatus ==CLOOP_STARTING){
				double s12;
	      asStateMechine.asLoopControlStatus = CLOOP_PROCESS;
				interW =absD(asRuningInforCurrent.aVelocity );
				stopped=0;
			 
			  a0 = asMotorCurrInfor.aKp_P + asMotorCurrInfor.aKi_P*DT/2 +asMotorCurrInfor.aKd_P/DT;
				a1 = -asMotorCurrInfor.aKp_P + asMotorCurrInfor.aKi_P*DT/2 -2*asMotorCurrInfor.aKd_P/DT;
				a2 = asMotorCurrInfor.aKd_P/DT;
			 
				asRuningInforCurrent.aEstimatesValue = asRuningInforCurrent.aPosition ;
				s12 = absD(asRuningInforSet.aPosition -asRuningInforCurrent.aPosition ) - asRuningInforSet.aVelocity *asRuningInforSet.aVelocity /asRuningInforSet.aAccel ;
        if(s12>0){
					index = (uint32_t)((asRuningInforSet.aVelocity /asRuningInforSet.aAccel  + s12/asRuningInforSet.aVelocity )/DT);
				}
				else{
					index = (uint32_t)((sqrt(absD(asRuningInforSet.aPosition -asRuningInforCurrent.aPosition )/asRuningInforSet.aAccel ))/DT);
				}
     }
//Noi suy gia toc và van toc khi dung mang hoac USB dieu khien:
     if((asRuningInforSet.aPosition !=asRuningInforCurrent.aEstimatesValue && asStateMechine.asCmType != CM_PULSE)){
///////////// /////////////////////     	  //noi suy gia toc:	
					if(index){//tang toc:
							interW+= asRuningInforSet.aAccel *DT;
							if(interW> asRuningInforSet.aVelocity ) interW = asRuningInforSet.aVelocity ;
							index--;
						}
					else{  //giam toc:
							interW-= asRuningInforSet.aAccel *DT;
							if(interW< VELOCITY_MIN) interW = VELOCITY_MIN;
					}
	////////////////////////////////////		  //Noi suy van toc:
					denta = interW*DT   ;
					if((asRuningInforSet.aPosition  >= asRuningInforCurrent.aPosition )){
							asRuningInforCurrent.aEstimatesValue+=denta;
							if(asRuningInforCurrent.aEstimatesValue>asRuningInforSet.aPosition )
							{
								asRuningInforCurrent.aEstimatesValue=asRuningInforSet.aPosition ;				
							}
					 }
					 else if((asRuningInforSet.aPosition <= asRuningInforCurrent.aPosition ))
					 {  
						  asRuningInforCurrent.aEstimatesValue -= denta;
							if(asRuningInforCurrent.aEstimatesValue < asRuningInforSet.aPosition )
							{
								asRuningInforCurrent.aEstimatesValue=asRuningInforSet.aPosition ;
							}
					 }
      }
      else if(asRuningInforSet.aPosition ==asRuningInforCurrent.aEstimatesValue && asStateMechine.asCmType != CM_PULSE){
         //Tien trinh dieu khien theo uoc luoc da hoan tat:
					asStateMechine.asLoopControlStatus = CLOOP_FINISH;
      }
      else{    //Control by Pulse:
				 asRuningInforCurrent.aEstimatesValue = (aExternCounterValue*6.283185307)/asDataHwInfor.aBaudrate;
      }
			
			aPositionOlderErr = aPositionOldErr;
	    aPositionOldErr = aPositionErr;
      aPositionErr = asRuningInforCurrent.aEstimatesValue-asRuningInforCurrent.aPosition;
			
//xac dinh chieu va kiem tra sensor:
      if((aPositionErr)>0){
	    	aMotorDir =1;
			if(SENSOR_RIGHT && stopped==0){
				 asRuningInforSet.aPosition   = asRuningInforCurrent.aPosition ;	//dung tai cho khong cho tiep tuc quay nua.
				 stopped=1;
				 return 0;
			}
	  }
      else if(aPositionErr<0){
	   		aMotorDir =0;
			if(SENSOR_LEFT && stopped==0){
				 asRuningInforSet.aPosition = asRuningInforCurrent.aPosition ;	//dung tai cho khong cho tiep tuc quay nua.	
				 stopped=1;
				 return 0;
			}
	  }
			
		pid += a0*aPositionErr + a1*aPositionOldErr + a2*aPositionOlderErr;
		if (pid > VEL_MAX)
		{
			pid = VEL_MAX;
		}
		return pid;
}


void DRIVER_Control(void){
	  static double temp;				
		if(aLoopEnable==0){
			return;
		}
		
		ENCODER_Read();
		SOURCE_Read();
		//Check Current:
		ERROR_CurrentCheck();
		//Check Motor Error:
//		if(ERROR_MotorCheck()){
//			asRuningInforCurrent.aEstimatesValue = asRuningInforCurrent.aPosition;
//			DC_DISABLE();
//		}
		
		asRuningInforOlder.aPosition = asRuningInforCurrent.aPosition ;    //Luu ket qua w lai
		asRuningInforOlder.aVelocity= asRuningInforCurrent.aVelocity ;
		//asRuningInforOlder.aAccel = 		asRuningInforCurrent.aAccel;
	
		asRuningInforCurrent.aPosition  = (EncoderCounterValue*1.570796327)/(asDataHwInfor.aEncoderLine);//(Encoder2_Value*1.5708)/(numberPullPerRound);//((double)Encoder2_Value/(numberPullPerRound*4))*6.283185;    //DV goc la rad 2*Pi = 6.283185.
		//Filter velocity:    
		temp  = wFilter(asRuningInforCurrent.aPosition -asRuningInforOlder.aPosition)/DT;                       
	  asRuningInforCurrent.aVelocity += asDataHwInfor.aWFilter*DT*(temp -asRuningInforCurrent.aVelocity);                    //(asRuningInforCurrent.aVelocity : w filter : Bo loc nhieu low Pass, 0.01 la chu ky lay mau.
	
	//Filter Accel:
		temp = (asRuningInforCurrent.aVelocity-asRuningInforOlder.aVelocity)/DT; 
    asRuningInforCurrent.aAccel  += asDataHwInfor.aAcelFilter*DT*(temp - asRuningInforCurrent.aAccel) ;  												 //ko loc nhieu gia toc:	
		
		ADC_Read();
		if(asStateMechine.aStatus == STATUS_RUN && asRuningInforCurrent.aListError==0){
					
					if(asStateMechine.asLoopControlStatus != CLOOP_STOOP){
								 switch(asStateMechine.asRunningMode){
										case MODE_TURNING:
											 LOOP_AutoTunning();
											 break;
										case MODE_SF_POSITION:
											if(asStateMechine.asCmType == CM_PULSE){
													ReadExternalCounter();
											 }
											StateFeedback_PositionControl();
										  break;
										case MODE_PI_VELOCITY:
											 PI_ControlVelocity();
										   break;
										case MODE_PID_POSITION:
											 if(asStateMechine.asCmType == CM_PULSE){
													ReadExternalCounter();
											 }
											 PID_PositionControl();
											 break;
										case MODE_SMART_POSITION:
											 if(asStateMechine.asCmType == CM_PULSE){
													ReadExternalCounter();
											 }
											 PidPiPi_PositionControl();
											 break;
										default:
											 break;
							
								 }
						}
						else {	//dung pwm
							DCM_UpdateDuty( 0, aMotorDir );
						}
		}
		else{
			DCM_UpdateDuty( 0, 3 );
		}
}