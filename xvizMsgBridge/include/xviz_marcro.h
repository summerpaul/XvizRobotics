/**
 * @Author: Xia Yunkai
 * @Date:   2024-01-02 08:45:08
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-01-20 11:23:52
 */
#include <stdint.h>

#ifndef __MARCRO_H__
#define __MARCRO_H__

#define XVIZ_MAX(a, b) (((a) > (b)) ? (a) : (b))
#define XVIZ_MIN(a, b) (((a) < (b)) ? (a) : (b))
#define XVIZ_CLAMP(v, min, max) (XVIZ_MAX(XVIZ_MIN(v, max), min))
#define XVIZ_PI (3.1415926f)  // pi
#define XVIZ_2PI (6.2831853f) // 2*pi
#define XVIZ_PI2 (1.5707963f) // pi/2
#define XVIZ_RPI (0.31831f)   // 1/pi
// 角度弧度互转
#define XVIZ_ANG2RAD(x) ((x) * HATA_PI * 0.0055556f) // 角度转弧度，转完后变成浮点型
#define XVIZ_RAD2ANG(x) ((x) * HATA_RPI * 180)       // 弧度转角度



#endif /* __MARCRO_H__ */
