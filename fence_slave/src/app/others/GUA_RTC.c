//******************************************************************************          
//name:         GUA_RTC.c          
//introduce:    ナ﹚?RTC??     
//author:       并并ナ        
//email:        897503845@qq.com     
//QQ group      ナBLEぇCC2640(557278427)   
//changetime:   2016.09.04  
//******************************************************************************  
#include "GUA_RTC.h"  
  
//******************************************************************************          
//name:             GUA_RTC_Init         
//introduce:        ナRTC﹍て        
//parameter:        none        
//return:           none       
//author:           并并ナ               
//email:            897503845@qq.com   
//QQ group          ナBLEぇCC2640(557278427)                
//changetime:       2016.09.04                   
//******************************************************************************  
void GUA_RTC_Init(void)  
{  
  //﹍てUTC.  
  UTC_init();   
}  
//******************************************************************************          
//name:             GUA_RTC_Set         
//introduce:        ナRTC?竚??ㄧ?        
//parameter:        pGUA_Timer???        
//return:           none       
//author:           并并ナ               
//email:            897503845@qq.com   
//QQ group          ナBLEぇCC2640(557278427)                
//changetime:       2016.09.04                   
//******************************************************************************  
void GUA_RTC_Set(UTCTimeStruct *pGUA_Timer)  
{  
  UTCTimeStruct SetTime;   
  UTCTime SetTime_seconds;    
    
  //???誹  
  SetTime.year = pGUA_Timer->year;                
  SetTime.month = pGUA_Timer->month - 1;          
  SetTime.day = pGUA_Timer->day - 1;             
  SetTime.hour = pGUA_Timer->hour - 1;            
  SetTime.minutes = pGUA_Timer->minutes;         
  SetTime.seconds = pGUA_Timer->seconds;   
  
  //??????  
  SetTime_seconds = UTC_convertUTCSecs(&SetTime);  
      
  //?竚??  
  UTC_setClock(SetTime_seconds);    
}  
  
//******************************************************************************          
//name:             GUA_RTC_Get         
//introduce:        ナRTC???ㄧ?        
//parameter:        pGUA_Timer???        
//return:           none       
//author:           并并ナ               
//email:            897503845@qq.com   
//QQ group          ナBLEぇCC2640(557278427)                
//changetime:       2016.09.04                   
//******************************************************************************  
void GUA_RTC_Get(UTCTimeStruct *pGUA_Timer)  
{  
  UTCTimeStruct GetTime;  
    
  //??玡?誹  
  UTC_convertUTCTime(&GetTime, UTC_getClock());  
      
  //ъ??誹  
  pGUA_Timer->year = GetTime.year;                        
  pGUA_Timer->month = GetTime.month + 1;                  
  pGUA_Timer->day = GetTime.day + 1;                      
  pGUA_Timer->hour = GetTime.hour + 1;                    
  pGUA_Timer->minutes = GetTime.minutes;                  
  pGUA_Timer->seconds = GetTime.seconds;                  
}  
   