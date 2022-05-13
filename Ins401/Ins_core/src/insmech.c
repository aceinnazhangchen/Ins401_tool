#include "insmech.h"

#include <stdio.h>
#include<string.h>
#include "lcstruct.h"
#include "cmatrix.h"
#include "earth.h"
#include "orientation.h"


#include "stdio.h"
#include "stdlib.h"
#include "time.h"
static double temp1[3] = {0.0,0.0,0.0};
static double temp2[3] = {0.0,0.0,0.0};
static double temp3[3] = {0.0,0.0,0.0};
static double q1[4] = {0.0,0.0,0.0,0.0};
static double q2[4] = {0.0,0.0,0.0,0.0};
static double q3[4] = {0.0,0.0,0.0,0.0};
static double C1[3][3] = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
static double C2[3][3] = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
static double C3[3][3] = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
static double eye33[3][3] = { {1.0,0.0,0.0},{0.0,1.0,0.0},{0.0,0.0,1.0} };
//static float eye33float[3][3] = {{1.0,0.0,0.0},{0.0,1.0,0.0},{0.0,0.0,1.0}};

static double dvel_b_prev[3] ;
static double dtheta_b_prev[3] ;
static double dvel_b_cur[3];
static double dtheta_b_cur[3];

//  float x[15] 改成  float x[16], ymj
// 已分析
int8_t KF_feedback(Par* mPar, float x[16], Nav* mNav, int IsBiasFeedBack, GnssInsSystem *mGnssInsSystem)
{
	int ret = -1;
	int i = 0;
    // 位置反馈, 误差相当于X = c系 - n系(真)
    // 北东位置误差转为经纬度误差
	double d_lat = (double)(x[0]) / ((double)(mNav->height) + mPar->Rm);
	double d_lon = (double)(x[1]) / (double)(mPar->Rn + mNav->height) / cos(mNav->lat);
    // 参考Wn_en公式,这里的误差相当于就是在n系下的速度,转为n系下的角度误差，Wn_cn, theta_cn
	dpos2rvec(mNav->lat, d_lat, d_lon, temp1);
	for (i = 0; i < 3; i++)
	{
		temp2[i] = -temp1[i]; // 求反，相当于把theta_cn变为theta_nc
	}
	rvec2quat(temp2, q1); // 等效旋转矢量转为四元数，q1为q_nc
	quatprod(mNav->q_ne, q1, q2); // 相当于补偿q_ne, q2为真q_ne = q_ce*q_nc
	quat2pos(q2, temp3); // 把四元数转成经纬度
	mNav->lat = temp3[0]; // 得到补偿后的纬度，经度，高度
	mNav->lon = temp3[1];
	mNav->height = mNav->height + x[2]; // NED向下为正, 高度向上为正
	memcpy(mNav->q_ne, q2, 4*sizeof(double)); // 更新补偿过后的q_ne

	GetSkewSymmetricMatrixOfVector(temp1, *C1); // theta_cn的反对称阵
	MatrixAdd(*eye33, *C1, 3, 3, *C2); // 1 + theta_cn

    // 速度反馈
    // 先减去速度误差, v_c = v_n(估计) - 误差
	temp2[0] = (double)(mNav->vn - x[3]);
	temp2[1] = (double)(mNav->ve - x[4]);
	temp2[2] = (double)(mNav->vd - x[5]);
    // v_n = (1 + theta_cn) * v_c
	MatrixMutiply(*C2, temp2, 3, 3, 1, temp3);
	mNav->vn = (float)temp3[0]; // 得到补偿后的速度
	mNav->ve = (float)temp3[1];
	mNav->vd = (float)temp3[2];

    // 姿态反馈
    // 姿态误差
	temp2[0] = (double)x[6];
	temp2[1] = (double)x[7];
	temp2[2] = (double)x[8] ;
    // 姿态误差再补偿位置误差带来的差异
	MatrixAdd(temp2, temp1, 3, 1, temp3);
	rvec2quat(temp3, q1); // 等效旋转矢量转为四元数, q_pn
	quatprod(q1, mNav->q_bn, q2); // q_bn = q_pn * q_bp(估计的q_bn)
	memcpy(mNav->q_bn, q2, 4 * sizeof(double));

	quat2dcm(mNav->q_bn, mNav->c_bn);
	dcm2euler(mNav->c_bn, temp3);
	mNav->roll = (float)temp3[0];
	mNav->pitch = (float)temp3[1];
	mNav->heading = (float)temp3[2];

    // IMU bias反馈
	if (IsBiasFeedBack)
	{
		mNav->sensorbias.bias_gyro_x += x[9];
		mNav->sensorbias.bias_gyro_y += x[10];
		mNav->sensorbias.bias_gyro_z += x[11];
		mNav->sensorbias.bias_acc_x += x[12];
		mNav->sensorbias.bias_acc_y += x[13];
		mNav->sensorbias.bias_acc_z += x[14];
	}

	// odo scale反馈
	if (16 == mGnssInsSystem->mKalmanStruct.n)
	{
		if (mGnssInsSystem-> mImuData.timestamp - mGnssInsSystem->lastGNSSLCTime < 1.2
			&& fabs(mGnssInsSystem->mOdoData.vehicle_speed) > 5.0
			&& fabs(mGnssInsSystem->mNav.wnb_n[2]) < 0.03
			&&mGnssInsSystem->PreGNSSQUALITYMode == 4
			&&mGnssInsSystem->GNSSQUALITYMode == 4)
		{
			mNav->Odo_scale += x[15];
		}
	}

    // 误差清0
	memset(x, 0, mGnssInsSystem->mKalmanStruct.n * sizeof(float));
    ret = 1;
	return ret;
}

// 已分析，赋值imu数据到INS
int8_t DataChangeLC(const ImuData* mImuData, INS *cins)
{
	int ret = -1;
	cins->timestamp = mImuData->timestamp;
	cins->timestamped = mImuData->timestamped;
	cins->gyro_x = mImuData->Gyrox;
	cins->gyro_y = mImuData->Gyroy;
	cins->gyro_z = mImuData->Gyroz;
	cins->acc_x = mImuData->Accx;
	cins->acc_y = mImuData->Accy;
	cins->acc_z = mImuData->Accz;
	ret = 1;
	return ret;
}

// 已分析
int8_t compensate( const Sensorbias* bias, INS* ins)
{
	ins->acc_x -= bias->bias_acc_x;
	ins->acc_y -= bias->bias_acc_y;
	ins->acc_z -= bias->bias_acc_z;
	ins->gyro_x -= bias->bias_gyro_x;
	ins->gyro_y -= bias->bias_gyro_y;
	ins->gyro_z -= bias->bias_gyro_z;
	return 1;
}

static double zeta[3];
static double mid_v[3] = { 0.0 };
static double dv_fb[3] = { 0.0 };
static double dv_fn[3] = { 0.0 };

// 已分析
int8_t INS_MECH(const INS* ins_pre, const INS* ins_cur, const Nav* nav, Nav* nav1, Par* mPar)
{
	unsigned char i;
	//Calculat earthhparents  use navparements
	double dt = ins_cur->timestamp - ins_pre->timestamp; // IMU实际的时间步长
	//Date prepare
	dvel_b_prev[0] = ins_pre->acc_x * dt;
	dvel_b_prev[1] = ins_pre->acc_y * dt ;
	dvel_b_prev[2] = ins_pre->acc_z * dt ;
	dtheta_b_prev[0] = ins_pre->gyro_x* dt;
	dtheta_b_prev[1] = ins_pre->gyro_y * dt;
	dtheta_b_prev[2] = ins_pre->gyro_z* dt ;
	dvel_b_cur[0] = ins_cur->acc_x * dt;
	dvel_b_cur[1] =  ins_cur->acc_y * dt;
	dvel_b_cur[2] =  ins_cur->acc_z * dt ;
	dtheta_b_cur[0] = ins_cur->gyro_x* dt;
	dtheta_b_cur[1] = ins_cur->gyro_y* dt;
	dtheta_b_cur[2] = ins_cur->gyro_z * dt ;


	CrossProduct(dtheta_b_cur, dvel_b_cur, temp1); // temp1 = dtheta_b_cur x dvel_b_cur
	CrossProduct(dtheta_b_prev, dvel_b_cur, temp2); // temp2 = dtheta_b_prev x dvel_b_cur
	CrossProduct(dvel_b_prev, dtheta_b_cur, temp3); // temp3 = dvel_b_prev x dtheta_b_cur
	for (int i = 0; i < 3; i++)
	{
		dv_fb[i] = dvel_b_cur[i] + 0.5*temp1[i] + (temp2[i] + temp3[i]) / 12; // 双子样比力速度增量
	}

	mPar->g[0] = 0.0;
	mPar->g[1] = 0.0;
	UpdateGravity(&(nav->lat), &(mPar->g[2])); // 更新当地重力
	UpdateMN(&(nav->lat), &mPar->Rm, &mPar->Rn); // 更新Rm,Rn

	//V*************************Velcocity Updata 速度更新 ***************************************
	//position extrapolation
	//earth and angular rate updateing
	mid_v[0] =(double)(nav->vn) + 0.5 * nav->dv_n[0]; // 速度外推
	mid_v[1] =(double)(nav->ve) + 0.5 * nav->dv_n[1];
	mid_v[2] =(double)(nav->vd) + 0.5 * nav->dv_n[2];

	mPar->w_ie[0] = WGS84.wie * cos(nav->lat); // 地球自转角速度在n系下的投影
	mPar->w_ie[1] = 0.0;
	mPar->w_ie[2] = -WGS84.wie * sin(nav->lat);

	mPar->w_en[0] = (double)(nav->ve) / (mPar->Rn + nav->height); // n系角速度在n系下的投影
	mPar->w_en[1] = -(double)(nav->vn) / (mPar->Rm + nav->height);
	mPar->w_en[2] = -(double)(nav->ve) * tan(nav->lat) / (mPar->Rn + (double)(nav->height));
	// navigation frame rotation vector

	for (i = 0; i < 3; i++)
	{
		temp1[i] = 0.5*(mPar->w_ie[i] + mPar->w_en[i]) * dt; // t_k-1 到 t_k的旋转矢量
	}
	GetSkewSymmetricMatrixOfVector(temp1, *C1);
	MatrixSub(*eye33, *C1, 3, 3, *C2); // I-(0.5theta_X)
	MatrixMutiply(*C2, *nav->c_bn, 3, 3, 3, *C3); // (I-(0.5theta_X))*c_bn(t_k-1上一时刻的)
	MatrixMutiply(*C3, dv_fb, 3, 3, 1, dv_fn); // 比力引起的速度变化=(I-(0.5theta_X))*c_bn(t_k-1上一时刻的)*双子样比力速度增量


    // 向心加速度、哥氏加速度和地球重力带来的速度变化
	for (i = 0; i < 3; i++)
	{
		temp1[i] = 2 * mPar->w_ie[i] + mPar->w_en[i]; 
	}
	CrossProduct(temp1, mid_v, temp2);
	for (i = 0; i < 3; i++)
	{
		nav1->dv_n[i] = dv_fn[i] + (mPar->g[i] - temp2[i])*dt; // 最终的速度变化
		nav1->a_n[i] = nav1->dv_n[i] / dt; // 加速度
	}
	nav1->vn = nav->vn + (float)(nav1->dv_n[0]); // 更新当前速度
	nav1->ve = nav->ve + (float)(nav1->dv_n[1]);
	nav1->vd = nav->vd + (float)(nav1->dv_n[2]);

	//**************************Postion Updata 位置更新 **************************
	mid_v[0] = 0.5*(double)(nav1->vn + nav->vn); // 由于速度已经计算出，可以得到中间时刻的速度
	mid_v[1] = 0.5*(double)(nav1->ve + nav->ve);
	mid_v[2] = 0.5*(double)(nav1->vd + nav->vd);
	mPar->w_en[0] = mid_v[1] / (mPar->Rn + (double)(nav->height)); 
	mPar->w_en[1] = -mid_v[0] / (mPar->Rm + (double)(nav->height));
	mPar->w_en[2] = -mid_v[1] * tan(nav->lat) / (mPar->Rn + (double)(nav->height));
	for (i = 0; i < 3; i++)
	{
		zeta[i] = (mPar->w_ie[i] + mPar->w_en[i]) * dt; // n系的角度变化
	}
	rvec2quat(zeta, q1);  // q_n(k-1)n(k)
	temp3[0] = 0.0;
	temp3[1] = 0.0;
	temp3[2] = -WGS84.wie*dt;

	rvec2quat(temp3, q2); // q_e(k)e(k-1)
	quatprod(nav->q_ne, q1, q3); // q_e(k-1)n(k-1)
	quatprod(q2, q3, nav1->q_ne);
	norm_quat(nav1->q_ne); // 正规化
	quat2pos(nav1->q_ne, temp1);  // 得到更新后的经纬度

	nav1->lat = temp1[0]; 
	nav1->lon = temp1[1];
	nav1->height = nav->height - mid_v[2] * dt; // 用中间时刻的速度更新高程
	//*************************Attitude Updata 姿态更新 *****************************

	//Calculate rotational and sculling motion
	CrossProduct(dtheta_b_prev, dtheta_b_cur, temp1); // 二子样圆锥误差
	for (i = 0; i < 3; ++i)
	{
		temp2[i] = dtheta_b_cur[i] + temp1[i] / 12;
	}
	rvec2quat(temp2, q2); // q_b(k-1)b(k)

	for (i = 0; i < 3; i++)
	{
		temp3[i] = -zeta[i];
	}
	rvec2quat(temp3, q1);  // q_n(k)n(k-1)

	quatprod(nav->q_bn, q2, q3);
	quatprod(q1, q3, nav1->q_bn); // q_n(k)b(k)
	norm_quat(nav1->q_bn); // 正规化
	quat2dcm(nav1->q_bn, nav1->c_bn);
	dcm2euler(nav1->c_bn, temp1); // 得到更新后的姿态
	MatrixTranspose(*(nav1->c_bn), 3, 3, *(nav1->c_nb));

	nav1->roll = (float)temp1[0];
	nav1->pitch = (float)temp1[1];
	nav1->heading = (float)temp1[2];

	for (i = 0; i < 3; i++)
	{
		mPar->f_n[i] = dv_fn[i] / dt; // n系下比例加速度
		mPar->f_b[i] = dvel_b_cur[i] / dt; // b系下比例加速度
		mPar->w_b[i] = dtheta_b_cur[i] / dt; // b系下角速度
		temp2[i] = (mPar->w_ie[i] + mPar->w_en[i]); // w_n
	}
	MatrixMutiply(*(nav1->c_bn), mPar->w_b, 3, 3, 1, temp1); // w_b
	for (i = 0; i < 3; i++)
	{
		nav1->wnb_n[i] = temp1[i] - temp2[i]; // w_b - w_n
	}
    MatrixMutiply(*(nav1->c_nb), nav1->wnb_n, 3, 3, 1, nav1->wnb_b);

	return 1;
}

/***************************************************************************************
Function: KF_predict_16PHI
Description: ;Calculate PHI(transition matrix)
Input :dt  time increment
	   nav  Navigation information
Output:PHI  transition matrix
Return :
Others:
********************************************************************************************/
// 已分析
int8_t KF_predict_16PHI(const float dt, const Nav *mNav, const Par *mPar, const ImuSensor *mImuSensor, const int16_t n,float* PHI)
{
	double R = sqrt(mPar->Rm* mPar->Rn);

	//PHI = 15x15/16*16 transition matrix
	memset(PHI, 0, n * n * sizeof(float));
	//Postion error dynamics
	//pos to pos F_1
	for (int i = 0; i < 3; i++)
	{
		temp1[i] = -mPar->w_en[i];
	}
	GetSkewSymmetricMatrixOfVector(temp1, *C1);
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			PHI[i*n + j] =(float)(eye33[i][j] + C1[i][j] * dt);
		}
	}
	//pos to vel F_2
	for (int i = 0; i < 3; i++)
	{
		for (int j = 3; j < 6; j++)
		{
			PHI[i*n + j] = (float)(eye33[i][j - 3] * dt);
		}
	}
	//Velocity error dynamics
	//vel to pose F_3
	PHI[3*n + 0] = -(float)(mPar->g[2] / (R + (double)mNav->height)*dt);
	PHI[4*n + 1] = PHI[3 * n];
	PHI[5*n + 2] = -2 * PHI[3 * n];
	//vel to vel F_4
	for (int i = 0; i < 3; i++)
	{
		temp2[i] = -2 * mPar->w_ie[i] - mPar->w_en[i];
	}
	GetSkewSymmetricMatrixOfVector(temp2, *C2);
	for (int i = 3; i < 6; i++)
	{
		for (int j = 3; j < 6; j++)
		{
			PHI[i*n + j] = (float)(eye33[i - 3][j - 3] + C2[i - 3][j - 3] * dt);
		}
	}
	//vel to att F_5
	GetSkewSymmetricMatrixOfVector(mPar->f_n, *C3);
	for (int i = 3; i < 6; i++)
	{
		for (int j = 6; j < 9; j++)
		{
			PHI[i*n + j] = (float)(C3[i - 3][j - 6] * dt);
		}
	}
	//vel to acc bias F_6
	for (int i = 3; i < 6; i++)
	{
		for (int j = 12; j < 15; j++)
		{
			PHI[i*n + j] = (float)(mNav->c_bn[i - 3][j - 12] * dt);
		}
	}
	//attitude error dynamic F_8
	for (int i = 0; i < 3; i++)
	{
		temp3[i] = -mPar->w_ie[i] - mPar->w_en[i];
	}
	GetSkewSymmetricMatrixOfVector(temp3, *C3);
	for (int i = 6; i < 9; i++)
	{
		for (int j = 6; j < 9; j++)
		{
			PHI[i*n + j] = (float)(eye33[i - 6][j - 6] + C3[i - 6][j - 6] * dt);
		}
	}
	//att to gyro bias F_9
	for (int i = 6; i < 9; i++)
	{
		for (int j = 9; j < 12; j++)
		{
			PHI[i*n + j] = -(float)(mNav->c_bn[i - 6][j - 9] * dt);
		}
	}

	//gyro bias F_11
	PHI[9 * n + 9] = mImuSensor->bg_model[0];
	PHI[10 * n + 10] = mImuSensor->bg_model[1];
	PHI[11 * n + 11] = mImuSensor->bg_model[2];
	//acc bias F_12
	PHI[12*n+12] = mImuSensor->ba_model[0];
	PHI[13*n+13] = mImuSensor->ba_model[1];
	PHI[14*n+14] = mImuSensor->ba_model[2];
	if (16 == n)
	{
		PHI[15 * n + 15] = 1;
	}
	return 1;
}
/*******************************************************************************************
Function: KF_predict
Description: ;
Input :x    state vect,old
	   P    Covariance,old
	   PHI  transition matrix
	   Q    the continuous-time system noise
	   dt   time increment
Output:x1  state vect new
	   P1  Covariance new
Return :
Others:
*************************************************************************************************/
// 已分析
int8_t  KF_predict(const int16_t n,const float* PHI, const float* Q, const float dt, float* x, float* P, UpdataStruct* updataStruct)
{
	float Qd[StateX2];
	float M1[StateX2],M2[StateX2];

		PHI_Q(n,PHI, Q, M1);// M1 = PHI*Q 
		PHIQ_QPHIT(n, M1, dt, Qd); // Qd = 0.5*(M1+M1T)*dt
		//if (n == 16) { Qd[255] = Q[15]; };
	
		PHI_P(n, PHI, (updataStruct->Q), M1); // M1 = PHI*updataStruct->Q
		PHIP_PHIT(n, M1, PHI, M2);  // M2 = M1*PHI_T
		MatrixAddfloat(M2, Qd, n, n, (updataStruct->Q)); 
        // updataStruct->Q =  M2 + Qd = M1*PHI_T + 0.5*(M1+M1T)*dt
        //                 = PHI* updataStruct->Q * PHI_T + 0.5*(PHI*Q+PHI*Q_T)*dt

		PHI_P(n, PHI, (updataStruct->PHI), M1); // M1 = PHI * (updataStruct->PHI)
		memcpy(updataStruct->PHI, M1, n * n * sizeof(float)); // updataStruct->PHI = M1



		PHI_P(n, PHI, P, M1); // M1 = PHI * P
		PHIP_PHIT(n, M1, PHI, M2); // M2 = M1*PHI_T = PHI * P * PHI_T + 
		MatrixAddfloat(M2, Qd, n, n, P); // P = PHI * P * PHI_T + 0.5*(PHI*Q+PHI*Q_T)*dt
	
	return 1;
}



