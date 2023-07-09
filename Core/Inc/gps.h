/*
 * gps.h
 *
 *  Created on: Feb 19, 2023
 *      Author: arisery
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_
#include "main.h"

#define false 0
#define true 1
//定义数组长度
#define USART_REC_LEN  			1600
#define GPS_Buffer_Length 200
#define UTCTime_Length 11
#define latitude_Length 11
#define N_S_Length 2
#define longitude_Length 12
#define E_W_Length 2

typedef struct SaveData
{
	char GPS_Buffer[GPS_Buffer_Length];
	char isGetData;		//是否获取到GPS数据
	char isParseData;	//是否解析完成
	char UTCTime[UTCTime_Length];		//UTC时间
	char latitude[latitude_Length];		//纬度
	char N_S[N_S_Length];		//N/S
	char longitude[longitude_Length];		//经度
	char E_W[E_W_Length];		//E/W
	char isUsefull;		//定位信息是否有效
} _SaveData;

struct position
{
    /* data */
    double lat; // 维度
    double lon; // 经度
};
void parseGpsBuffer();
void printGpsBuffer();
double distance(double lat1, double lon1, double lat2, double lon2);
void CLR_Buf(void);
void clrStruct();
double areaOfTriangle(double a, double b, double c);
double Get_OffsetDistance(struct position start,struct position end,struct position now);
#endif /* INC_GPS_H_ */
