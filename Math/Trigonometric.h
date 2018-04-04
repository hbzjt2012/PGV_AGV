#pragma once

#ifdef __cplusplus
extern "C" {
#endif

//sin值查表
//提供0-90°的sin数值表值，间隔0.1°
//超出该范围的角度值由角度变换获得
//输入为角度
float Sin_Lookup(float angle);
//cos值查表
//提供0-90°的cos数值表值，间隔0.1°
//超出该范围的角度值由角度变换获得
//输入为角度
float Cos_Lookup(float angle);
//tan值查表
//提供0-45°的tan数值表值，间隔0.1°
//超出该范围的角度值由角度变换获得
//输入为角度
float Tan_Lookup(float angle);

//arcsin
//利用sin值查表（二分法查表）
//返回角度放大了10倍
int ArcSin_Lookup(float value);
//arccos
//利用cos值查表（二分法查表）
//返回角度放大了10倍
int ArcCos_Lookup(float value);
//arctan值
//利用tan值查表（二分法查表）
//返回角度放大了10倍
int ArcTan_Lookup(float x, float y);

#ifdef __cplusplus
}
#endif