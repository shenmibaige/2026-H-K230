#include "pid_q2.h"

void Q2_PID_Reset(Q2_PID_t *p)
{
	if (p == 0)
	{
		return;
	}

	p->Target = 0.0f;
	p->Actual = 0.0f;
	p->Out = 0.0f;
	p->Error0 = 0.0f;
	p->Error1 = 0.0f;
	p->ErrorInt = 0.0f;
}

void Q2_PID_SetTarget(Q2_PID_t *p, float target)
{
	if ((p == 0) || (p->Target == target))
	{
		return;
	}

	p->Target = target;
	p->ErrorInt = 0.0f;
	p->Error0 = target - p->Actual;
	p->Error1 = p->Error0;
}

void Q2_PID_SetActualMagnitude(Q2_PID_t *p, int16_t encoder_delta)
{
	if (p == 0)
	{
		return;
	}

	/*
	 * 两个电机因镜像安装而采用相反的电气正转方向。速度环控制的是轮速
	 * 幅值，不能让编码器安装极性把正常前进解释为负速度并持续推高 PWM。
	 */
	if (encoder_delta < 0)
	{
		p->Actual = -(float)encoder_delta;
	}
	else
	{
		p->Actual = (float)encoder_delta;
	}
}

/**
  * 函   数：PID计算及结构体变量值更新
  * 参   数：Q2_PID_t * 指定结构体的地址
  * 返 回 值：无
  */
void Q2_PID_Update(Q2_PID_t *p)
{
	float previousOut;

	if (p == 0)
	{
		return;
	}

	previousOut = p->Out;

	/*获取本次误差和上次误差*/
	p->Error1 = p->Error0;					//获取上次误差
	p->Error0 = p->Target - p->Actual;		//获取本次误差，目标值减实际值，即为误差值
	
	/*外环误差积分（累加）*/
	/*如果Ki不为0，才进行误差积分，这样做的目的是便于调试*/
	/*因为在调试时，我们可能先把Ki设置为0，这时积分项无作用，误差消除不了，误差积分会累积到很大的值*/
	/*后续一旦Ki不为0，那么因为误差积分已经累积到很大的值了，这就导致积分项疯狂输出，不利于调试*/
	if (p->Ki != 0)					//如果Ki不为0
	{
		p->ErrorInt += p->Error0;	//进行误差积分
		/*
		 * 积分抗饱和。新灰度位置 PID 显式给出积分误差上下限，使积分
		 * 输出限制为理论 500 mm 圆弧所需的 11 PWM；旧调用方未配置
		 * 显式范围时仍回退到 OutMax/Ki。
		 */
		if (p->ErrorIntMax > p->ErrorIntMin)
		{
			if (p->ErrorInt > p->ErrorIntMax)
			{
				p->ErrorInt = p->ErrorIntMax;
			}
			if (p->ErrorInt < p->ErrorIntMin)
			{
				p->ErrorInt = p->ErrorIntMin;
			}
		}
		else
		{
			float intLimit = p->OutMax / p->Ki;
			if (intLimit < 0.0f)
			{
				intLimit = -intLimit;
			}
			if (p->ErrorInt > intLimit)  p->ErrorInt = intLimit;
			if (p->ErrorInt < -intLimit) p->ErrorInt = -intLimit;
		}
	}
	else							//否则
	{
		p->ErrorInt = 0;			//误差积分直接清零
	}
	
	/*PID计算*/
	/*使用位置式PID公式，计算得到输出值*/
	p->Out = p->Kp * p->Error0
		   + p->Ki * p->ErrorInt
		   + p->Kd * (p->Error0 - p->Error1);
	
	/*输出限幅*/
	if (p->Out > p->OutMax) {p->Out = p->OutMax;}	//限制输出值最大为结构体指定的OutMax
	if (p->Out < p->OutMin) {p->Out = p->OutMin;}	//限制输出值最小为结构体指定的OutMin

	/*
	 * 直线加速受限，避免一启动就冲到满 PWM；减速允许更快，保证进入弯道时
	 * 内轮能在数个 5 ms 周期内建立足够差速。
	 */
	if ((p->OutRiseMax > 0.0f) &&
	    (p->Out > (previousOut + p->OutRiseMax)))
	{
		p->Out = previousOut + p->OutRiseMax;
	}
	if ((p->OutFallMax > 0.0f) &&
	    (p->Out < (previousOut - p->OutFallMax)))
	{
		p->Out = previousOut - p->OutFallMax;
	}
}
