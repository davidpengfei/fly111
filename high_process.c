#include "mpu6050.h"
#include "attitude_process.h"
#include "high_process.h"
#include "sys.h"

_st_Height Height;

 float pos_correction;
 float acc_correction;
 float vel_correction;
/****气压计三阶互补滤波方案——参考开源飞控APM****/
//#define TIME_CONTANST_ZER       1.5f
const float TIME_CONTANST_ZER=4.0f;
#define K_ACC_ZER 	        (1.0f / (TIME_CONTANST_ZER * TIME_CONTANST_ZER * TIME_CONTANST_ZER))
#define K_VEL_ZER	        (3.0f / (TIME_CONTANST_ZER * TIME_CONTANST_ZER))//20															// XY????·′à??μêy,3.0
#define K_POS_ZER               (3.0f / TIME_CONTANST_ZER)
float high_test;

void Strapdown_INS_High(float high)
{
			float dt;
			float Altitude_Estimate=0;
			static uint16_t delay_cnt;
			static float high_offset;
	    {
			  static	float last_time;
				float now_time;
				now_time = micros();
				dt = now_time - last_time;
				dt /=1000;
				last_time = now_time;
			
			}
			if(delay_cnt<500)//刚刚上电时 对气压计地面补偿
			{
				pos_correction = acc_correction = vel_correction = 0;
				high_offset = high;
				delay_cnt++;
				return;
			}
			high -= high_offset;

			const uint8_t High_Delay_Cnt=2;//150ms
      Altitude_Estimate=high;//高度观测量

			high_test = high;
      //由观测量（气压计）得到状态误差
			static float History_Z[High_Delay_Cnt+1]; 
			float temp;
      temp= Altitude_Estimate- History_Z[High_Delay_Cnt];//气压计(超声波)与SINS估计量的差，单位cm
      //三路积分反馈量修正惯导
      acc_correction +=temp* K_ACC_ZER*dt ;//加速度矫正量
      vel_correction +=temp* K_VEL_ZER*dt ;//速度矫正量
      pos_correction +=temp* K_POS_ZER*dt ;//位置矫正量
      //加速度计矫正后更新
			static float last_accZ;
			static float Current_accZ; //当前Z轴上的阿加速度
      last_accZ=Current_accZ;//上一次加速度量
			//三阶加速度矫正
      Current_accZ = Attitude.acc_yaw_sensor + acc_correction;
      //速度增量矫正后更新，用于更新位置,由于步长h=0.005,相对较长，不建议用更高阶段，因为加速度信号非平滑
     
			float speed_Z;
			static float t_high;
			static float t_speed;
      speed_Z =+(last_accZ+Current_accZ)*dt/2.0f;//①v = at
      //原始位置更新
      t_high += (Height.Speed+0.5f*speed_Z)*dt;//②s = v0t+ 1/2at*t   根据①at = v  得到s = v0t + 1/2 v*t
      //三阶位置矫正后更新
			Height.High = t_high + pos_correction;
      //垂直原始速度更新
      t_speed +=speed_Z;
      //三阶垂直速度矫正垂直速度矫正后更新
      Height.Speed = t_speed + vel_correction;

			//---------------------------------------------------------------
			static uint16_t Save_Cnt=0;
      Save_Cnt++;//数据存储周期
      if(Save_Cnt>=1)//5ms
      {
        for(uint8_t Cnt=High_Delay_Cnt;Cnt>0;Cnt--)//5ms滑动一次
        {
					History_Z[Cnt]=History_Z[Cnt-1];
        }
        History_Z[0]=Height.High; //
        Save_Cnt=0;
      }
}







