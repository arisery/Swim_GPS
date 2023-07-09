/*
 * parse.h
 *
 *  Created on: 2023年3月15日
 *      Author: arisery
 */

#ifndef INC_PARSE_H_
#define INC_PARSE_H_

int wgs2bd(double lat, double lon, double* pLat, double* pLon); // WGS84=>BD09 地球坐标系=>百度坐标系
int gcj2bd(double lat, double lon, double* pLat, double* pLon);	// GCJ02=>BD09 火星坐标系=>百度坐标系
int bd2gcj(double lat, double lon, double* pLat, double* pLon);	// BD09=>GCJ02 百度坐标系=>火星坐标系
int wgs2gcj(double lat, double lon, double* pLat, double* pLon);// WGS84=>GCJ02 地球坐标系=>火星坐标系
int gcj2wgs(double lat, double lon, double* pLat, double* pLon);// GCJ02=>WGS84 火星坐标系=>地球坐标系(粗略)
int bd2wgs(double lat, double lon, double* pLat, double* pLon);	// BD09=>WGS84 百度坐标系=>地球坐标系(粗略)
int gcj2wgs_Exactly(double lat, double lon, double* wgs_Lat, double* wgs_lon);// GCJ02=>WGS84 火星坐标系=>地球坐标系（精确）
int bd2wgs_Exactly(double lat, double lon, double* pLat, double* pLon);// BD09=>WGS84 百度坐标系=>地球坐标系(精确)
double *OffSet(double lat, double lon);	// 偏移量
double transformLat(double x, double y);// 纬度偏移量
double transformLon(double x, double y);// 经度偏移量

#endif /* INC_PARSE_H_ */
