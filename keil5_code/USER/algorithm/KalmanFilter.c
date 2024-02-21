#include "KalmanFilter.h"

void kalman_mat_init(KALMAN_FILTER_MAT *mat, KALMAN_FILTER_DATA *data)
{
	mat_init(&mat->X, 2, 1, data->X_data);
	mat_init(&mat->P, 2, 2, data->P_data);
	mat_init(&mat->F, 2, 2, data->F_data);
	mat_init(&mat->Q, 2, 2, data->Q_data);
	mat_init(&mat->H, 1, 2, data->H_data);
	mat_init(&mat->R, 1, 1, data->R_data);
	mat_init(&mat->K, 2, 1, data->K_data);
	mat_init(&mat->Z, 1, 1, data->Z_data);
	
	
}

