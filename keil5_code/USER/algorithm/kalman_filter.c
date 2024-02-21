#include "kalman_filter.h"

float matrix_value1;
float matrix_value2;

void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I)
{
  mat_init(&F->xhat,2,1,(float *)I->xhat_data);//k-1时刻状态
  mat_init(&F->xhatminus,2,1,(float *)I->xhatminus_data);//当前时刻估计状态
  mat_init(&F->z,2,1,(float *)I->z_data);//状态参数储存
  mat_init(&F->A,2,2,(float *)I->A_data);//转移矩阵
//	mat_init(&F->B,2,1,(float *)I->B_data);
  mat_init(&F->H,2,2,(float *)I->H_data);//单位矩阵
  mat_init(&F->Q,2,2,(float *)I->Q_data);//过程协方差矩阵
  mat_init(&F->R,2,2,(float *)I->R_data);//测量协方差矩阵
  mat_init(&F->P,2,2,(float *)I->P_data);//状态协方差矩阵
  mat_init(&F->Pminus,2,2,(float *)I->Pminus_data);
  mat_init(&F->K,2,2,(float *)I->K_data);//
  mat_init(&F->AT,2,2,(float *)I->AT_data);
  mat_trans(&F->A, &F->AT);
  mat_init(&F->HT,2,2,(float *)I->HT_data);
  mat_trans(&F->H, &F->HT);
//  matrix_value2 = F->A.pData[1];
}


// xhatminus==x(k|k-1)  xhat==X(k-1|k-1)
// Pminus==p(k|k-1)     P==p(k-1|k-1)    AT==A'
// HT==H'   K==kg(k)    I=1
//

/**
  *@param 卡尔曼参数结构体
  *@param 角度
  *@param 速度
*/
float *kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2)
{
  float TEMP_data[4] = {0, 0, 0, 0};
  float TEMP_data21[2] = {0, 0};
  mat TEMP,TEMP21;

  mat_init(&TEMP,2,2,(float *)TEMP_data);//
  mat_init(&TEMP21,2,1,(float *)TEMP_data21);//

  F->z.pData[0] = signal1;//z(k)  位置
  F->z.pData[1] = signal2;//z(k)  速度
  
  //1. xhat'(k)= A xhat(k-1)
  mat_mult(&F->A, &F->xhat, &F->xhatminus);//  x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)   矩阵相乘  参数一 A矩阵地址  参数二 B矩阵地址 参数三 存放矩阵地址
  
	//  mat_mult(&F->B, ,&TEMP33);
  //2. P'(k) = A P(k-1) AT + Q
  mat_mult(&F->A, &F->P, &F->Pminus);//   p(k|k-1) = A*p(k-1|k-1)*A'+Q
  mat_mult(&F->Pminus, &F->AT, &TEMP);//  p(k|k-1) = A*p(k-1|k-1)*A'+Q
  mat_add(&TEMP, &F->Q, &F->Pminus);//    p(k|k-1) = A*p(k-1|k-1)*A'+Q   //矩阵相加

  //3. K(k) = P'(k) HT / (H P'(k) HT + R)
  mat_mult(&F->H, &F->Pminus, &F->K);//  kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
  mat_mult(&F->K, &F->HT, &TEMP);//      kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
  mat_add(&TEMP, &F->R, &F->K);//        kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)

  mat_inv(&F->K, &F->P);//
  mat_mult(&F->Pminus, &F->HT, &TEMP);//
  mat_mult(&TEMP, &F->P, &F->K);//

  //4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
  mat_mult(&F->H, &F->xhatminus, &TEMP21);//      x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  mat_sub(&F->z, &TEMP21, &F->xhat);//            x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  mat_mult(&F->K, &F->xhat, &TEMP21);//           x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  mat_add(&F->xhatminus, &TEMP21, &F->xhat);//    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))

  //5. P(k) = (1-K(k)H)P'(k)
  mat_mult(&F->K, &F->H, &F->P);//            p(k|k) = (I-kg(k)*H)*P(k|k-1)
  mat_sub(&F->Q, &F->P, &TEMP);//
  mat_mult(&TEMP, &F->Pminus, &F->P);

  matrix_value1 = F->xhat.pData[0];
  matrix_value2 = F->xhat.pData[1];

  F->filtered_value[0] = F->xhat.pData[0];
  F->filtered_value[1] = F->xhat.pData[1];
  return F->filtered_value;
}
