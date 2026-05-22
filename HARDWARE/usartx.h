#ifndef __USRATX_H
#define __USRATX_H 

#include "stdio.h"
#include "sys.h"
#include "system.h"

#define HEADER_0 0xA5
#define HEADER_1 0x5A
#define Length_ 0x6C

#define POINT_PER_PACK 32

typedef struct PointData
{
	uint8_t distance_h;
	uint8_t distance_l;
	uint8_t Strong;

}LidarPointStructDef;

typedef struct PackData
{
	uint8_t header_0;
	uint8_t header_1;
	uint8_t ver_len;
	
	uint8_t speed_h;
	uint8_t speed_l;
	uint8_t start_angle_h;
	uint8_t start_angle_l;	
	LidarPointStructDef point[POINT_PER_PACK];
	uint8_t end_angle_h;
	uint8_t end_angle_l;
	uint8_t crc;
}LiDARFrameTypeDef;

typedef struct PointDataProcess_
{
	uint16_t distance;
	float angle;
}PointDataProcessDef;

extern PointDataProcessDef PointDataProcess[1200];//更新225个数据
extern LiDARFrameTypeDef Pack_Data;
extern PointDataProcessDef Dataprocess[1200];//用于小车避障、跟随、走直线、ELE雷达避障的雷达数据

extern float Distance_KP,Distance_KD,Distance_KI;		//距离调整PID参数
extern float Follow_KP,Follow_KD,Follow_KI;  //转向PID
extern float Follow_KP_Akm,Follow_KD_Akm,Follow_KI_Akm;

extern float Diff_Along_Distance_KP,Diff_Along_Distance_KD,Diff_Along_Distance_KI;	//距离调整PID参数
extern float Akm_Along_Distance_KP,Akm_Along_Distance_KD,Akm_Along_Distance_KI;	//距离调整PID参数
extern float FourWheel_Along_Distance_KP,FourWheel_Along_Distance_KD,FourWheel_Along_Distance_KI;	//距离调整PID参数
extern float Along_Distance_KP,Along_Distance_KD,Along_Distance_KI;		//距离调整PID参数


//void data_task(void *pvParameters);

void CAN_SEND(void);
void uart1_init(u32 bound);
void uart2_init(u32 bound);
void uart5_init(u32 bound);

int USART1_IRQHandler(void);
int USART2_IRQHandler(void);
int UART5_IRQHandler(void);
void data_process(void);

float Vz_to_Akm_Angle(float Vx, float Vz);
float XYZ_Target_Speed_transition(u8 High,u8 Low);
void usart1_send(u8 data);
void usart2_send(u8 data);
void usart5_send(u8 data);

//u8 Check_Sum(unsigned char Count_Number,unsigned char Mode);
u8 AT_Command_Capture(u8 uart_recv);
void _System_Reset_(u8 uart_recv);

#endif

