/**
 * @file oled.c
 * @brief 0.96英寸 OLED 显示屏驱动程序 (SSD1306, I2C 接口)
 * @note  基于 MSPM0 硬件 I2C，支持显示 ASCII、汉字、图形、格式化打印等
 */

#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"
#include <stdarg.h>
#include <string.h>

/* 显存缓冲区：128列 × 8页 (每页8像素) */
u8 OLED_GRAM[144][8];

/* 外部延时函数（由用户提供，单位ms） */
extern void delay_ms(uint32_t ms);

/************************** 底层 I2C 与基本控制 **************************/

/**
 * @brief 设置 OLED 反色显示
 * @param i 0:正常显示(黑底白字); 1:反色显示(白底黑字)
 */
void OLED_ColorTurn(u8 i)
{
	if(i==0) OLED_WR_Byte(0xA6,OLED_CMD);
	if(i==1) OLED_WR_Byte(0xA7,OLED_CMD);
}

/**
 * @brief 设置屏幕旋转180度
 * @param i 0:正常方向; 1:旋转180度
 */
void OLED_DisplayTurn(u8 i)
{
	if(i==0)
	{
		OLED_WR_Byte(0xC8,OLED_CMD);
		OLED_WR_Byte(0xA1,OLED_CMD);
	}
	if(i==1)
	{
		OLED_WR_Byte(0xC0,OLED_CMD);
		OLED_WR_Byte(0xA0,OLED_CMD);
	}
}

/**
 * @brief 通过 I2C 向 OLED 写一个字节（命令或数据）
 * @param dat  要发送的字节
 * @param mode OLED_CMD(0) 或 OLED_DATA(1)
 * @note  硬件 I2C 方式，地址固定为 0x3C，每次发送两个字节：控制字节 + 数据
 */
void OLED_WR_Byte(uint8_t dat, uint8_t mode)
{
    uint8_t txData[2];
    txData[0] = mode ? 0x40 : 0x00;   // 控制字节: 0x40=数据, 0x00=命令
    txData[1] = dat;

    while (!(DL_I2C_getControllerStatus(OLED_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_fillControllerTXFIFO(OLED_INST, txData, 2);
    DL_I2C_startControllerTransfer(OLED_INST, 0x3C, DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    while (!(DL_I2C_getControllerStatus(OLED_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS));
    while (!(DL_I2C_getControllerStatus(OLED_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
}

/**
 * @brief 开启 OLED 显示 (点亮屏幕)
 */
void OLED_DisPlay_On(void)
{
	OLED_WR_Byte(0x8D,OLED_CMD);   // 电荷泵使能
	OLED_WR_Byte(0x14,OLED_CMD);   // 开启电荷泵
	OLED_WR_Byte(0xAF,OLED_CMD);   // 点亮屏幕
}

/**
 * @brief 关闭 OLED 显示 (熄灭屏幕，但仍保持显存内容)
 */
void OLED_DisPlay_Off(void)
{
	OLED_WR_Byte(0x8D,OLED_CMD);
	OLED_WR_Byte(0x10,OLED_CMD);   // 关闭电荷泵
	OLED_WR_Byte(0xAF,OLED_CMD);
}

/**
 * @brief 将显存 OLED_GRAM 全部刷新到 OLED 屏幕
 * @note  页寻址模式，每页 128 字节，共 8 页
 */
static uint8_t g_oledRefreshPage = 8u;

void OLED_RefreshBegin(void)
{
	g_oledRefreshPage = 0u;
}

uint8_t OLED_RefreshBusy(void)
{
	return (g_oledRefreshPage < 8u) ? 1u : 0u;
}

uint8_t OLED_RefreshStep(void)
{
	u8 n;
	u8 page;

	if (g_oledRefreshPage >= 8u) {
		return 1u;
	}
	page = g_oledRefreshPage;
	OLED_WR_Byte((u8)(0xb0u + page), OLED_CMD); // 设置页地址
	OLED_WR_Byte(0x00u, OLED_CMD);              // 低列起始地址
	OLED_WR_Byte(0x10u, OLED_CMD);              // 高列起始地址
	for (n = 0u; n < 128u; ++n) {
		OLED_WR_Byte(OLED_GRAM[n][page], OLED_DATA);
	}
	g_oledRefreshPage++;
	return (g_oledRefreshPage >= 8u) ? 1u : 0u;
}

void OLED_Refresh(void)
{
	OLED_RefreshBegin();
	while (OLED_RefreshBusy() != 0u) {
		(void)OLED_RefreshStep();
	}
}

/**
 * @brief 清屏 (清空显存并刷新屏幕)
 */
void OLED_ClearBuffer(void)
{
	u8 i,n;
	for(i=0;i<8;i++)
	   for(n=0;n<128;n++)
			 OLED_GRAM[n][i]=0;
	
}

void OLED_Clear(void)
{
	OLED_ClearBuffer();
	OLED_Refresh();
}

/************************** 图形绘制函数 **************************/

/**
 * @brief 在显存中画一个点
 * @param x 列坐标 (0~127)
 * @param y 行坐标 (0~63)
 */
void OLED_DrawPoint(u8 x,u8 y)
{
	u8 i,m,n;
	i=y/8;      // 页索引
	m=y%8;      // 页内位偏移
	n=1<<m;
	OLED_GRAM[x][i]|=n;
}

/**
 * @brief 在显存中清除一个点
 * @param x 列坐标 (0~127)
 * @param y 行坐标 (0~63)
 */
void OLED_ClearPoint(u8 x,u8 y)
{
	u8 i,m,n;
	i=y/8;
	m=y%8;
	n=1<<m;
	OLED_GRAM[x][i]=~OLED_GRAM[x][i];
	OLED_GRAM[x][i]|=n;
	OLED_GRAM[x][i]=~OLED_GRAM[x][i];
}

/**
 * @brief 画线 (支持横线、竖线、斜线)
 */
void OLED_DrawLine(u8 x1,u8 y1,u8 x2,u8 y2)
{
	u8 i,k,k1,k2;
	if((x1<0)||(x2>128)||(y1<0)||(y2>64)||(x1>x2)||(y1>y2))return;
	if(x1==x2)    // 竖线
	{
		for(i=0;i<(y2-y1);i++) OLED_DrawPoint(x1,y1+i);
	}
	else if(y1==y2)   // 横线
	{
		for(i=0;i<(x2-x1);i++) OLED_DrawPoint(x1+i,y1);
	}
	else      // 斜线 (简单 Bresenham 近似)
	{
		k1=y2-y1;
		k2=x2-x1;
		k=k1*10/k2;
		for(i=0;i<(x2-x1);i++) OLED_DrawPoint(x1+i,y1+i*k/10);
	}
}

/**
 * @brief 画圆 (Bresenham 算法)
 */
void OLED_DrawCircle(u8 x,u8 y,u8 r)
{
	int a = 0, b = r, num;
	while(2 * b * b >= r * r)
	{
		OLED_DrawPoint(x + a, y - b);
		OLED_DrawPoint(x - a, y - b);
		OLED_DrawPoint(x - a, y + b);
		OLED_DrawPoint(x + a, y + b);
		OLED_DrawPoint(x + b, y + a);
		OLED_DrawPoint(x + b, y - a);
		OLED_DrawPoint(x - b, y - a);
		OLED_DrawPoint(x - b, y + a);
		a++;
		num = (a * a + b * b) - r*r;
		if(num > 0) { b--; a--; }
	}
}

/************************** 字符与字符串显示 **************************/

/**
 * @brief 显示单个 ASCII 字符
 * @param x     字符左上角 X 坐标
 * @param y     字符左上角 Y 坐标
 * @param chr   要显示的字符 (如 'A')
 * @param size1 字体大小 (12/16/24)，对应字模表
 * @note  字模表为 asc2_1206 / asc2_1608 / asc2_2412
 */
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size1)
{
	u8 i,m,temp,size2,chr1;
	u8 y0=y;
	size2=(size1/8+((size1%8)?1:0))*(size1/2);   // 字符所占字节数
	chr1=chr-' ';   // 字模偏移量 (ASCII 表从空格开始)
	for(i=0;i<size2;i++)
	{
		if(size1==12) {temp=asc2_1206[chr1][i];}
		else if(size1==16) {temp=asc2_1608[chr1][i];}
		else if(size1==24) {temp=asc2_2412[chr1][i];}
		else return;
		for(m=0;m<8;m++)
		{
			if(temp&0x80) OLED_DrawPoint(x,y);
			else OLED_ClearPoint(x,y);
			temp<<=1;
			y++;
			if((y-y0)==size1)   // 一列扫描完成
			{
				y=y0;
				x++;
				break;
			}
		}
	}
}

/**
 * @brief 显示字符串 (仅支持 ASCII 可见字符)
 * @param x, y   起始坐标
 * @param chr    字符串指针
 * @param size1  字体大小
 * @note  自动换行：超出屏幕宽度或遇到 '\0' 结束
 */
void OLED_ShowString(u8 x,u8 y,u8 *chr,u8 size1)
{
	while((*chr>=' ')&&(*chr<='~'))
	{
		OLED_ShowChar(x,y,*chr,size1);
		x+=size1/2;                // 字符宽度 = 高度/2
		if(x>128-size1)            // 超出右边界换行
		{
			x=0;
			y+=size1;
		}
		chr++;
	}
}

/**
 * @brief 幂运算 (用于显示数字的位数)
 */
u32 OLED_Pow(u8 m,u8 n)
{
	u32 result=1;
	while(n--) result*=m;
	return result;
}

/**
 * @brief 显示数字 (指定长度，高位不足补零)
 * @param len  显示的总位数 (例如 len=3, num=5 会显示 "005")
 */
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size1)
{
	u8 t,temp;
	for(t=0;t<len;t++)
	{
		temp=(num/OLED_Pow(10,len-t-1))%10;
		if(temp==0) OLED_ShowChar(x+(size1/2)*t,y,'0',size1);
		else OLED_ShowChar(x+(size1/2)*t,y,temp+'0',size1);
	}
}

/**
 * @brief 显示汉字 (基于预定义的字模库索引)
 * @param num   汉字在字模库中的编号 (0 开始)
 * @param size1 字号 (16/24/32/64)
 * @note  字模库变量：Hzk1(16), Hzk2(24), Hzk3(32), Hzk4(64)
 */
void OLED_ShowChinese(u8 x,u8 y,u8 num,u8 size1)
{
	u8 i,m,n=0,temp,chr1;
	u8 x0=x,y0=y;
	u8 size3=size1/8;   // 汉字点阵占用的字节数 (例如 16x16 占 32 字节)
	while(size3--)
	{
		chr1=num*size1/8+n;
		n++;
		for(i=0;i<size1;i++)
		{
			if(size1==16) {temp=Hzk1[chr1][i];}
			else if(size1==24) {temp=Hzk2[chr1][i];}
			else if(size1==32) {temp=Hzk3[chr1][i];}
			else if(size1==64) {temp=Hzk4[chr1][i];}
			else return;
			for(m=0;m<8;m++)
			{
				if(temp&0x01) OLED_DrawPoint(x,y);
				else OLED_ClearPoint(x,y);
				temp>>=1;
				y++;
			}
			x++;
			if((x-x0)==size1) {x=x0; y0=y0+8;}
			y=y0;
		}
	}
}

/**
 * @brief 设置写数据的起始页和列地址 (用于图片显示)
 * @param x 列地址 (0~127)
 * @param y 页地址 (0~7)
 */
void OLED_WR_BP(u8 x,u8 y)
{
	OLED_WR_Byte(0xb0+y,OLED_CMD);
	OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
	OLED_WR_Byte((x&0x0f)|0x01,OLED_CMD);
}

/**
 * @brief 显示图片 (位图数据)
 * @param x0,x1  图片的 X 坐标范围 (左右边界)
 * @param y0,y1  图片的页地址范围 (上下边界)
 * @param BMP    位图数据数组
 * @note  图片数据需提前取模，按页格式存储
 */
void OLED_ShowPicture(u8 x0,u8 y0,u8 x1,u8 y1,u8 BMP[])
{
	u32 j=0;
	u8 x=0,y=0;
	if(y%8==0) y=0;
	else y+=1;
	for(y=y0;y<y1;y++)
	{
		 OLED_WR_BP(x0,y);
		 for(x=x0;x<x1;x++)
		 {
			 OLED_WR_Byte(BMP[j],OLED_DATA);
			 j++;
		 }
	}
}

/************************** OLED_Printf 格式化输出 (支持 %d, %u, %x, %s, %c, %%, %f, 宽度/零填充) **************************/

/**
 * @brief 内部函数: 输出一个字符并自动移动光标 (处理换行、回车、自动换行)
 * @param ch        要显示的字符
 * @param size1     字体大小 (用于计算宽度和换行)
 * @param cur_x,cur_y 当前光标位置 (指针，会被更新)
 */
static void OLED_PrintChar(u8 x, u8 y, u8 ch, u8 size1, u8 *cur_x, u8 *cur_y)
{
    // 注意: 参数 x,y 未使用, 实际使用传入的 cur_x, cur_y 指针
    (void)x; (void)y; // 避免编译警告
    if (ch == '\n') {
        *cur_x = 0;
        *cur_y += size1;
    } else if (ch == '\r') {
        *cur_x = 0;
    } else {
        OLED_ShowChar(*cur_x, *cur_y, ch, size1);
        *cur_x += size1 / 2;                      // 字符宽度
        if (*cur_x > 128 - size1) {               // 超过右边界则换行
            *cur_x = 0;
            *cur_y += size1;
        }
    }
}

/**
 * @brief 内部函数: 输出一个整数 (支持宽度、前导零填充、十六进制)
 * @param num       要显示的数字
 * @param base      进制 (10 或 16)
 * @param is_signed 是否为有符号数 (影响负数处理)
 * @param width     总宽度 (包括符号和数字) , 0 表示不限制
 * @param zero_fill 是否使用 '0' 填充 (否则空格填充)
 */
static void OLED_PrintNum(u8 *cur_x, u8 *cur_y, u8 size1, int32_t num, u8 base, u8 is_signed, int width, u8 zero_fill)
{
    char buf[20];
    char num_str[20];
    int i = 0, len;
    uint32_t unum;
    u8 negative = 0;

    if (is_signed && num < 0) {
        negative = 1;
        unum = (uint32_t)(-num);
    } else {
        unum = (uint32_t)num;
    }

    // 进制转换 (逆序存入 buf)
    do {
        uint8_t digit = unum % base;
        buf[i++] = (digit < 10) ? ('0' + digit) : ('a' + digit - 10);
        unum /= base;
    } while (unum);

    // 反转得到正向字符串
    int j = 0;
    while (i) {
        num_str[j++] = buf[--i];
    }
    num_str[j] = '\0';
    len = j;

    int total_width = width;
    int sign_len = negative ? 1 : 0;
    int fill_len = (total_width > len + sign_len) ? (total_width - len - sign_len) : 0;

    // 先输出负号 (如果有)
    if (negative) {
        OLED_PrintChar(*cur_x, *cur_y, '-', size1, cur_x, cur_y);
    }

    // 输出填充字符
    char fill_char = zero_fill ? '0' : ' ';
    for (int f = 0; f < fill_len; f++) {
        OLED_PrintChar(*cur_x, *cur_y, fill_char, size1, cur_x, cur_y);
    }

    // 输出数字部分
    for (int idx = 0; idx < len; idx++) {
        OLED_PrintChar(*cur_x, *cur_y, num_str[idx], size1, cur_x, cur_y);
    }
}

/**
 * @brief 内部函数: 输出浮点数 (支持精度，如 %.2f)
 * @param num       浮点数 (double)
 * @param precision 小数位数 (默认为 6)
 * @note  浮点数宽度/零填充暂未实现，可后续扩展
 */
static void OLED_PrintFloat(u8 *cur_x, u8 *cur_y, u8 size1, double num, int precision)
{
    if (num < 0) {
        OLED_PrintChar(*cur_x, *cur_y, '-', size1, cur_x, cur_y);
        num = -num;
    }
    uint32_t int_part = (uint32_t)num;
    double frac_part = num - int_part;

    // 输出整数部分 (无宽度控制)
    OLED_PrintNum(cur_x, cur_y, size1, (int32_t)int_part, 10, 0, 0, 0);

    OLED_PrintChar(*cur_x, *cur_y, '.', size1, cur_x, cur_y);
    if (precision > 0) {
        // 计算 10^precision
        uint32_t multiplier = 1;
        for (int i = 0; i < precision; i++) multiplier *= 10;
        // 四舍五入
        uint32_t frac_val = (uint32_t)(frac_part * multiplier + 0.5);
        // 处理进位 (简单忽略，实际可改进)
        if (frac_val >= multiplier) {
            frac_val = 0;
        }
        // 输出小数部分，不足 precision 位前补零
        uint32_t divisor = multiplier / 10;
        for (int i = 0; i < precision; i++) {
            uint8_t digit = (frac_val / divisor) % 10;
            OLED_PrintChar(*cur_x, *cur_y, '0' + digit, size1, cur_x, cur_y);
            divisor /= 10;
        }
    }
}

/**
 * @brief 格式化打印到 OLED 屏幕 (类似 printf)
 * @param x     起始 X 坐标 (0~127)
 * @param y     起始 Y 坐标 (0~63)
 * @param size1 字体大小 (12/16/24)
 * @param fmt   格式化字符串，支持的格式说明符:
 *              %d   - 有符号十进制整数，支持宽度和零填充，如 %5d, %05d
 *              %u   - 无符号十进制整数
 *              %x   - 十六进制 (小写)
 *              %s   - 字符串
 *              %c   - 单个字符
 *              %%   - 输出百分号
 *              %f   - 浮点数，支持精度如 %.2f (默认6位小数)
 * @param ...   可变参数
 * @note  自动换行、自动刷新屏幕；浮点数暂不支持宽度和零填充。
 * @example OLED_Printf(0, 0, 16, "Value: %05d", 3);   // 输出 "Value: 00003"
 */
static void OLED_VPrintf(u8 x, u8 y, u8 size1, const char *fmt, va_list args)
{
    u8 cur_x = x, cur_y = y;

    for (const char *p = fmt; *p; p++) {
        if (*p == '%') {
            p++;
            // 1. 解析标志: 仅支持 '0' (零填充)
            u8 zero_fill = 0;
            if (*p == '0') {
                zero_fill = 1;
                p++;
            }
            // 2. 解析宽度 (数字)
            int width = 0;
            while (*p >= '0' && *p <= '9') {
                width = width * 10 + (*p - '0');
                p++;
            }
            // 3. 解析精度 (.数字)
            int precision = -1;
            if (*p == '.') {
                p++;
                precision = 0;
                while (*p >= '0' && *p <= '9') {
                    precision = precision * 10 + (*p - '0');
                    p++;
                }
            }
            // 4. 解析格式符
            switch (*p) {
                case 'd':
                    OLED_PrintNum(&cur_x, &cur_y, size1, va_arg(args, int32_t), 10, 1, width, zero_fill);
                    break;
                case 'u':
                    OLED_PrintNum(&cur_x, &cur_y, size1, (int32_t)va_arg(args, uint32_t), 10, 0, width, zero_fill);
                    break;
                case 'x':
                    OLED_PrintNum(&cur_x, &cur_y, size1, (int32_t)va_arg(args, uint32_t), 16, 0, width, zero_fill);
                    break;
                case 's': {
                    char *s = va_arg(args, char*);
                    while (*s) {
                        OLED_PrintChar(cur_x, cur_y, *s++, size1, &cur_x, &cur_y);
                    }
                    break;
                }
                case 'c':
                    OLED_PrintChar(cur_x, cur_y, (char)va_arg(args, int), size1, &cur_x, &cur_y);
                    break;
                case '%':
                    OLED_PrintChar(cur_x, cur_y, '%', size1, &cur_x, &cur_y);
                    break;
                case 'f': {
                    double fval = va_arg(args, double);
                    if (precision < 0) precision = 6;
                    OLED_PrintFloat(&cur_x, &cur_y, size1, fval, precision);
                    break;
                }
                default:   // 未知格式符，原样输出该字符
                    OLED_PrintChar(cur_x, cur_y, *p, size1, &cur_x, &cur_y);
                    break;
            }
        } else {
            OLED_PrintChar(cur_x, cur_y, *p, size1, &cur_x, &cur_y);
        }
    }
}

void OLED_DrawPrintf(u8 x, u8 y, u8 size1, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    OLED_VPrintf(x, y, size1, fmt, args);
    va_end(args);
}

void OLED_Printf(u8 x, u8 y, u8 size1, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    OLED_VPrintf(x, y, size1, fmt, args);
    va_end(args);
    OLED_Refresh();   // 兼容旧接口：输出完成后立即刷新屏幕
}

/**
 * @brief OLED 初始化函数
 * @note  根据 SSD1306 数据手册配置寄存器，打开显示并清屏
 */
void OLED_Init(void)
{
	delay_ms(100);   // 等待屏幕内部复位完成
	OLED_WR_Byte(0xAE,OLED_CMD); // 关闭显示
	OLED_WR_Byte(0x00,OLED_CMD); // 低列地址
	OLED_WR_Byte(0x10,OLED_CMD); // 高列地址
	OLED_WR_Byte(0x40,OLED_CMD); // 起始行地址
	OLED_WR_Byte(0x81,OLED_CMD); // 对比度设置
	OLED_WR_Byte(0xCF,OLED_CMD); // 对比度值
	OLED_WR_Byte(0xA1,OLED_CMD); // 段重映射 (正常)
	OLED_WR_Byte(0xC8,OLED_CMD); // COM 扫描方向 (正常)
	OLED_WR_Byte(0xA6,OLED_CMD); // 正常显示
	OLED_WR_Byte(0xA8,OLED_CMD); // 复用率设置
	OLED_WR_Byte(0x3f,OLED_CMD); // 1/64 duty
	OLED_WR_Byte(0xD3,OLED_CMD); // 显示偏移
	OLED_WR_Byte(0x00,OLED_CMD); // 无偏移
	OLED_WR_Byte(0xd5,OLED_CMD); // 时钟分频
	OLED_WR_Byte(0x80,OLED_CMD);
	OLED_WR_Byte(0xD9,OLED_CMD); // 预充电周期
	OLED_WR_Byte(0xF1,OLED_CMD);
	OLED_WR_Byte(0xDA,OLED_CMD); // COM 引脚配置
	OLED_WR_Byte(0x12,OLED_CMD);
	OLED_WR_Byte(0xDB,OLED_CMD); // VCOMH 电压
	OLED_WR_Byte(0x40,OLED_CMD);
	OLED_WR_Byte(0x20,OLED_CMD); // 寻址模式设置
	OLED_WR_Byte(0x02,OLED_CMD); // 页寻址模式
	OLED_WR_Byte(0x8D,OLED_CMD); // 电荷泵设置
	OLED_WR_Byte(0x14,OLED_CMD); // 使能电荷泵
	OLED_WR_Byte(0xA4,OLED_CMD); // 正常显示 (不忽略显存)
	OLED_WR_Byte(0xA6,OLED_CMD); // 非反色
	OLED_WR_Byte(0xAF,OLED_CMD); // 开启显示
	OLED_Clear();                 // 清屏
}
