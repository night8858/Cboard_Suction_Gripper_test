#include "pid.h"
#include "main.h"
#include "math.h"
#include "string.h"
#include <stdint.h>
#include "stm32f4xx_hal.h"


#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }


/*********************濠⒀呭仱閸ｅ搫顕ｅ婊籇闁硅矇鍐ㄧ厬***********************/
/**
  * @brief  增量式PID控制算法
  * @note   计算PID控制的增量部分，并累加得到最终输出
  *         增量式PID的特点是输出与偏差的变化量相关，而不是偏差的绝对值
  * @param  pid: 增量式PID结构体指针，包含PID参数和状态
  * @retval 无
  */
void PID_IncrementMode(s_pid_increase_t *pid)
{
    /* 参数安全检查：确保PID参数为非负数 */
    if(pid->kp < 0) pid->kp = -pid->kp;  // 保证比例系数为正
    if(pid->ki < 0) pid->ki = -pid->ki;  // 保证积分系数为正
    if(pid->kd < 0) pid->kd = -pid->kd;  // 保证微分系数为正
    
    /* 误差阈值处理：当误差过大时，重置误差以避免控制震荡 */
    if(pid->errNow > 5 || pid->errNow < -5)
        pid->errNow = 0;

    /* 计算增量式PID的三个部分 */
    // 比例部分增量：当前误差与上一次误差的差值
    pid->dErrP = pid->errNow - pid->errOld1;
    
    // 积分部分增量：直接使用当前误差
    pid->dErrI = pid->errNow;
    
    // 微分部分增量：当前误差与上一次误差的两倍减去上上次误差的差值
    // 这种形式可以预测误差的变化趋势
    pid->dErrD = pid->errNow - 2 * pid->errOld1 + pid->errOld2;
    
    /* 更新误差历史记录 */
    pid->errOld2 = pid->errOld1;  // 上上次误差 = 上一次误差
    pid->errOld1 = pid->errNow;   // 上一次误差 = 当前误差
    
    /* 计算PID增量输出 */
    pid->dCtrOut = pid->kp * pid->dErrP + pid->ki * pid->dErrI + pid->kd * pid->dErrD;
    
    /* PID增量输出限幅 */
    if(pid->dCtrOut > pid->dOutMAX)
        pid->dCtrOut = pid->dOutMAX;
    else if(pid->dCtrOut < -pid->dOutMAX)
        pid->dCtrOut = -pid->dOutMAX;

    /* 计算总控制输出 */
    // 如果所有PID参数都为0，则重置控制输出为0
    if(pid->kp == 0 && pid->ki == 0 && pid->kd == 0)
        pid->ctrOut = 0;
    else
        // 否则累加增量部分到总输出
        pid->ctrOut += pid->dCtrOut;
    
    /* 总控制输出限幅 */
    if(pid->ctrOut > pid->OutMAX)
        pid->ctrOut = pid->OutMAX;
    else if(pid->ctrOut < -pid->OutMAX)
        pid->ctrOut = -pid->OutMAX;
}

void PID_AbsoluteMode(s_pid_absolute_t *pid)
{

	pid->Perror = pid->NowError;                  //P闁绘粠鍨垫俊顓㈠磻韫囨挻鈻曢柡鍕靛灠缂嶅宕滃鍛剨鐎归潻鎷�
	pid->Ierror += pid->NowError;                 //I闁绘粠鍨垫俊顓㈠磻韫囨挻鈻曢柡鍕靛灟缁楀倿鎮介棃娑欏€靛☉鎾亾闁烩晛鐡ㄧ€垫梻绱掗鐐茬厒闁绘粍婢樺﹢顏堟儍閸曨偂鐒婄€归潻鎷�
	pid->Derror = pid->NowError - pid->LastError; //D闁绘粠鍨垫俊顓㈠磻韫囨挻鈻曢柡鍕靛灠缂嶅宕滃鍛剨鐎瑰壊鍠曠粭灞剧▔婵犲喚鍋ч柛瀣箰濡﹪鎯冮崟顐ｂ枙闁稿﹦銆嬬槐婵嬪础閸愯弓鐒婄€瑰壊鍠栭·鍐煂閿燂拷
	pid->LastError = pid->NowError;               //闁哄洤鐡ㄩ弻濠囧磻韫囨挻鈻�	
	//闂傚嫭鍔曢崺妤冪矓椤栨艾鐎婚柛妯烘瑜板爼宕戣箛鎾粹枙
	if( pid->Ierror >= pid->IerrorLim) pid->Ierror =  pid->IerrorLim;
	else if( pid->Ierror <= -pid->IerrorLim)  pid->Ierror =  -pid->IerrorLim;
	//PID闁诡艾瀚獮鍡涙嚍閸屾繄缈婚柛鎴炴そ閸ｏ拷
	pid->Pout = pid->Kp * pid->Perror;
	pid->Iout = pid->Ki * pid->Ierror;
	pid->Dout = pid->Kd * pid->Derror;
	//PID闁诡剚妲掔欢顓㈠礄濞差亜娅�
	pid->PIDout = pid->Pout + pid->Iout + pid->Dout;
	//闂傚嫭鍔曢崺妗篒D闁诡剚妲掔欢顓㈠礄濞差亜娅�
	if(pid->PIDout > pid->PIDoutMAX) pid->PIDout = pid->PIDoutMAX;
	else if(pid->PIDout < -pid->PIDoutMAX) pid->PIDout = -pid->PIDoutMAX;
}
/**
 * @brief 缂備焦绻傞顔碱嚕瀵ゆ换D閻犱緤绱曢悾锟�(缂佸鍨伴崹搴ㄥ礆閸℃瑯鐎�)
 * @param s_pid_absolute_t *pid
 * @param float integral_apart_val
 * @return float PIDout
 */
void PID_AbsoluteMode_integral_apart(s_pid_absolute_t *pid,float integral_apart_val)
{
    //PID闁诡艾瀚獮鍡涙嚍閸屾矮鐒婄€归潻鎷�
	pid->Perror = pid->NowError;                  //P闁绘粠鍨垫俊顓㈠磻韫囨挻鈻曢柡鍕靛灠缂嶅宕滃鍛剨鐎归潻鎷�
    if(fabs(pid->NowError)<integral_apart_val)
	    pid->Ierror += pid->NowError;                 //I闁绘粠鍨垫俊顓㈠磻韫囨挻鈻曢柡鍕靛灟缁楀倿鎮介棃娑欏€靛☉鎾亾闁烩晛鐡ㄧ€垫梻绱掗鐐茬厒闁绘粍婢樺﹢顏堟儍閸曨偂鐒婄€归潻鎷�
    else pid->Ierror = 0;
	pid->Derror = pid->NowError - pid->LastError; //D闁绘粠鍨垫俊顓㈠磻韫囨挻鈻曢柡鍕靛灠缂嶅宕滃鍛剨鐎瑰壊鍠曠粭灞剧▔婵犲喚鍋ч柛瀣箰濡﹪鎯冮崟顐ｂ枙闁稿﹦銆嬬槐婵嬪础閸愯弓鐒婄€瑰壊鍠栭·鍐煂閿燂拷
	pid->LastError = pid->NowError;               //闁哄洤鐡ㄩ弻濠囧磻韫囨挻鈻�
	//闂傚嫭鍔曢崺妤冪矓椤栨艾鐎婚柛妯烘瑜板爼宕戣箛鎾粹枙
	if( pid->Ierror >= pid->IerrorLim) pid->Ierror =  pid->IerrorLim;
	else if( pid->Ierror <= -pid->IerrorLim)  pid->Ierror =  -pid->IerrorLim;
	//PID闁诡艾瀚獮鍡涙嚍閸屾繄缈婚柛鎴炴そ閸ｏ拷
	pid->Pout = pid->Kp * pid->Perror;
	pid->Iout = pid->Ki * pid->Ierror;
	pid->Dout = pid->Kd * pid->Derror;
	//PID闁诡剚妲掔欢顓㈠礄濞差亜娅�
	pid->PIDout = pid->Pout + pid->Iout + pid->Dout;
	//闂傚嫭鍔曢崺妗篒D闁诡剚妲掔欢顓㈠礄濞差亜娅�
	if(pid->PIDout > pid->PIDoutMAX) pid->PIDout = pid->PIDoutMAX;
	else if(pid->PIDout < -pid->PIDoutMAX) pid->PIDout = -pid->PIDoutMAX;
}
/**
 * @brief   PID闁告瑥鍊归弳鐔煎礆濠靛棭娼楅柛鏍ㄧ壄缁辨繈宕ｉ娆庣鞍闁衡偓閹勮含闁告帗绻傞～鎰板礌閺嵮冩瘣闁轰線顣﹂懙鎴︽晬鐏炶偐鐦嶉柛娆樺灟娴滄帡寮ㄩ幆褎韬€甸偊浜為獮鍡涙煂閿燂拷
 * @param 	PID_AbsoluteType *pid
 * @param   float kp
 * @param   float ki
 * @param   float kd
 * @param   float errILim
 * @param   float MaxOutCur		
 * @return None
 */
void pid_abs_param_init(s_pid_absolute_t *pid, float kp, float ki, float kd, float errILim, float MaxOutCur)
{
	memset(pid,0,sizeof(s_pid_absolute_t));
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;
	pid->IerrorLim = errILim;
	pid->PIDoutMAX = MaxOutCur;
}
/**
 * @brief   PID闁告瑥鍊归弳鐔烘導鐎ｎ亖鍋撶涵椋庣闁告瑯鍨禍鎺楀绩閹勮含闁告帗绻傞～鎰板礌閺嵮冩瘣闁轰線顣﹂懙鎴︽晬鐏炶偐鐦嶉柛娆樺灟娴滄帡寮ㄩ幆褎韬€甸偊浜為獮鍡涙煂瀹€瀣闁瑰瓨鍨归弫銈夊级閵夛附鏉归柛鎺撴緲閹﹪鎮抽鐐叉閻犲鍟抽惁顖炲矗閸屾稒娈�
 * @param 	PID_AbsoluteType *pid
 * @param   float kp
 * @param   float ki
 * @param   float kd
 * @param   float errILim
 * @param   float MaxOutCur		
 * @return None
 */
void pid_abs_evaluation(s_pid_absolute_t *pid, float kp, float ki, float kd, float errILim, float MaxOutCur)
{
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;
	pid->IerrorLim = errILim;
	pid->PIDoutMAX = MaxOutCur;
}

/**
 * @brief   闁告娲滈獮鍝朓D
 * @param 	s_pid_absolute_t *single_pid
 * @param   float get
 * @param   float targeti		
 * @return  float pid_output
 * @attention None
 */
float motor_single_loop_PID(s_pid_absolute_t *single_pid , float target , float get)
{
	static float pid_output;
	single_pid->NowError = (float)(target - get);
	PID_AbsoluteMode(single_pid);
	pid_output = single_pid->PIDout;

	return pid_output;
}
/**
 * @brief   濞戞捁灏欐鍢滻D
 * @param 	s_pid_absolute_t *pos_pid
 * @param   s_pid_absolute_t *spd_pid
 * @param   float externGet
 * @param   float externSet
 * @param   float internGet		
 * @return  pid_output(float)
 * @attention None
 */
float motor_double_loop_PID(s_pid_absolute_t *pos_pid, s_pid_absolute_t *spd_pid, float externGet, float externSet, float internGet)
{
	static float pid_output;
	static float out_st;

	pos_pid->NowError = (float)externSet - (float)externGet;
	PID_AbsoluteMode(pos_pid);
	out_st = pos_pid->PIDout;

	spd_pid->NowError = out_st - (float)internGet;
	PID_AbsoluteMode(spd_pid);
	pid_output = spd_pid->PIDout;

	return pid_output;
}
/**
 * @brief 濞戞捁灏欐鍢滻D(闂侇偆鍠庣€规娊镇抽婧炴繈宕氶崱妤€鐎荤紒鍌︽嫹)
 * @param s_pid_absolute_t *pos_pid
 * @param s_pid_absolute_t *spd_pid
 * @param float externGet
 * @param float externSet
 * @param float internGet
 * @param float integral_apart_val
 * @return pid_output(float)
 * @attention None
 */
float motor_double_loop_PID_integral_apart(s_pid_absolute_t *pos_pid, s_pid_absolute_t *spd_pid, float externGet, float externSet, \
                                            float internGet,float integral_apart_val)
{
	static float pid_output;
	static float out_st;

	pos_pid->NowError = (float)externSet - (float)externGet;
    PID_AbsoluteMode(pos_pid);
	out_st = pos_pid->PIDout;

	spd_pid->NowError = out_st - (float)internGet;
	PID_AbsoluteMode_integral_apart(spd_pid,integral_apart_val);
	pid_output = spd_pid->PIDout;

	return pid_output;
}