/*
 * gps.c
 *
 *  Created on: Feb 19, 2023
 *      Author: arisery
 */
#include "gps.h"
#include "parse.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "dma.h"
#define PI 3.1415926
#define R 6371.0

_SaveData Save_Data;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
uint8_t USART_RX_BUF[USART_REC_LEN];
char flag = 0, flag_p = 0;
struct position end_p ={ .lat = 28.647666, .lon = 115.826511 }, start_p ={ .lat = 28.649099, .lon = 115.827379 }, now;
double dist, distance_offset, N_a, E_a;
//如果硬件出错
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart1)
	{//输出“error”，系统复位
		printf("error\r\n");
		MX_DMA_Init();
		MX_USART1_UART_Init();
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
		__HAL_DMA_ENABLE(&hdma_usart1_rx);
		HAL_UART_Receive_DMA(&huart1, USART_RX_BUF, 1600);
		NVIC_SystemReset(); // 复位
	}

}
void parseGpsBuffer()
{
	char *p;

	char string[10];
	double A, N, E;
	//确认接收到有效数据
	if (Save_Data.isGetData)
	{
		//将标志位置0
		Save_Data.isGetData = false;
		printf("**************\r\n");

		p = strchr(Save_Data.GPS_Buffer, ',');
		p += 1;
		p = strchr(p, ',');

		memcpy(string, p, 10);

		 //获取卫星数据失败
		if (string[1] == 'V')
		{

			printf("data wrong.\r\n");
			sscanf(Save_Data.GPS_Buffer + 6, "%lf,", &A);
			printf("A:%d\r\n", (int) A);
			memset(USART_RX_BUF, 0, 1600);
		}
		//获取卫星数据成功
		else if (string[1] == 'A')
		{
			//获取坐标
			sscanf(Save_Data.GPS_Buffer + 6, "%lf,A,%lf,N,%lf,", &A, &N_a,
					&E_a);
			N_a /= 100.0f;
			E_a /= 100.0f;
			N = (int) N_a + (N_a - (int) N_a) / 0.6;
			E = (int) E_a + (E_a - (int) E_a) / 0.6;
			//将wgs坐标转换为gcj坐标
			wgs2gcj(N, E, &now.lat, &now.lon);
			printf("A:%d\tnN:%lf\tE:%lf\r\n", (int) A, now.lat, now.lon);
			//计算当前与目标的距离
			dist = distance(end_p.lat, end_p.lon, now.lat, now.lon);
			printf("distance:%lf\r\n", dist * 1000.f);
			//计算与中心连线的偏差
			distance_offset=Get_OffsetDistance(start_p, end_p, now);
			printf("offset distance :%lf \r\n",distance_offset);
			//清空接收缓冲区
			memset(USART_RX_BUF, 0, 1600);
		}

		flag_p = 1;
	}
}

//将角度转换为弧度
double deg2rad(double deg)
{
	return deg * PI / 180;
}

//计算两点经纬度之间的距离
double distance(double lat1, double lon1, double lat2, double lon2)
{
	//将经纬度转换为弧度
	lat1 = deg2rad(lat1);
	lon1 = deg2rad(lon1);
	lat2 = deg2rad(lat2);
	lon2 = deg2rad(lon2);

	//计算余弦值
	double cos_value = cos(lat1) * cos(lat2) * cos(lon1 - lon2)
			+ sin(lat1) * sin(lat2);

	//防止余弦值超出[-1, 1]的范围
	if (cos_value > 1)
	{
		cos_value = 1;
	}
	if (cos_value < -1)
	{
		cos_value = -1;
	}

	//计算反余弦值和距离
	double arc_value = acos(cos_value);
	double dist = R * arc_value;

	return dist;
}

void CLR_Buf(void)                           // 串口缓存清理
{
	memset(USART_RX_BUF, 0, USART_REC_LEN);      //清空
}
double areaOfTriangle(double a, double b, double c)
{
    double p = (a + b + c) / 2;
    double area = sqrt(p * (p - a) * (p - b) * (p - c));
    return area;
}
//计算点now与点start和end连线的最短距离
double Get_OffsetDistance(struct position start,struct position end,struct position now)
{
	double dist;
	double a=distance(start.lat, start.lon, end.lat,end.lon)*1000.0;
	double b=distance(end.lat, end.lon, now.lat,now.lon)*1000.0;
	double c=distance(start.lat, start.lon, now.lat,now.lon)*1000.0;
	 double area = areaOfTriangle(a, b, c);
	dist =2.0 * area / a;
	return dist;
}
