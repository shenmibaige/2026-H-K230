/**
 * @file oled.h
 * @brief 0.96英寸 OLED 显示屏驱动程序头文件
 */

#ifndef __OLED_H
#define __OLED_H 

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdlib.h>

/* 命令/数据标志 */
#define OLED_CMD  0   // 写命令
#define OLED_DATA 1   // 写数据

/* 类型别名 */
typedef unsigned char u8;
typedef unsigned int  u32;

/************************** 基本功能函数 **************************/

void OLED_ClearPoint(u8 x, u8 y);       ///< 清除一个像素点
void OLED_ColorTurn(u8 i);              ///< 反色显示 (0:正常,1:反色)
void OLED_DisplayTurn(u8 i);            ///< 屏幕旋转 (0:正常,1:180°)
void OLED_WR_Byte(u8 dat, u8 mode);     ///< 写一个字节 (命令或数据)
void OLED_DisPlay_On(void);             ///< 点亮屏幕
void OLED_DisPlay_Off(void);            ///< 熄灭屏幕 (省电)
void OLED_Refresh(void);                ///< 刷新显存到屏幕
void OLED_RefreshBegin(void);           ///< 开始分页刷新（不阻塞整屏）
uint8_t OLED_RefreshStep(void);         ///< 刷新一页，完成整屏时返回1
uint8_t OLED_RefreshBusy(void);         ///< 分页刷新是否仍在进行
void OLED_ClearBuffer(void);            ///< 只清空显存，不刷新屏幕
void OLED_Clear(void);                  ///< 清屏
void OLED_Init(void);                   ///< 初始化 OLED

/************************** 图形绘制 **************************/

void OLED_DrawPoint(u8 x, u8 y);        ///< 画点
void OLED_DrawLine(u8 x1, u8 y1, u8 x2, u8 y2);   ///< 画线
void OLED_DrawCircle(u8 x, u8 y, u8 r);           ///< 画圆

/************************** 字符/字符串显示 **************************/

/**
 * @brief 显示单个 ASCII 字符
 * @param x,y   左上角坐标
 * @param chr   字符 (如 'A')
 * @param size1 字号 (12/16/24)
 */
void OLED_ShowChar(u8 x, u8 y, u8 chr, u8 size1);

/**
 * @brief 显示 ASCII 字符串 (自动换行)
 * @param chr   字符串指针
 */
void OLED_ShowString(u8 x, u8 y, u8 *chr, u8 size1);

/**
 * @brief 显示数字 (固定长度，不足补零)
 * @param num   数值
 * @param len   总位数
 */
void OLED_ShowNum(u8 x, u8 y, u32 num, u8 len, u8 size1);

/**
 * @brief 显示汉字 (基于字模库索引)
 * @param num   汉字在字模库中的编号 (0 开始)
 * @param size1 字号 (16/24/32/64)
 */
void OLED_ShowChinese(u8 x, u8 y, u8 num, u8 size1);

/************************** 图片显示 **************************/

void OLED_WR_BP(u8 x, u8 y);                       ///< 设置页/列地址
void OLED_ShowPicture(u8 x0, u8 y0, u8 x1, u8 y1, u8 BMP[]);  ///< 显示图片

/************************** 格式化打印 (printf 风格) **************************/

/**
 * @brief 格式化打印到 OLED (支持 %d, %u, %x, %s, %c, %%, %f)
 * @param x,y   起始坐标
 * @param size1 字号 (12/16/24)
 * @param fmt   格式字符串，例如:
 *              - "%d"       有符号十进制
 *              - "%5d"      宽度5，右对齐空格填充
 *              - "%05d"     宽度5，前导零填充
 *              - "%.2f"     浮点数，保留2位小数 (默认6位)
 * @param ...   可变参数
 * @note  自动换行并刷新屏幕
 */
void OLED_Printf(u8 x, u8 y, u8 size1, const char *fmt, ...);
void OLED_DrawPrintf(u8 x, u8 y, u8 size1, const char *fmt, ...);

#endif
