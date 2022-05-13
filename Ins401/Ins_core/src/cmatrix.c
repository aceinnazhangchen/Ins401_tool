#include <math.h>
#include <string.h>

#include "cmatrix.h"

#ifndef PI
#define PI (3.1415926535897932384626433832795)
#endif

#define SMD(i) ((i)*((i)+1)/2)
#define SMI(i, j)	((i) > (j) ? (SMD(i) + (j)) : (SMD(j) + (i)))
#define NX_INS 15          //15 
#define IsOdo 0

//PHI*P
void PHI_P(const int16_t n,const float *PHI,const float *P, float *PHIP)
{
	unsigned char i, j;
	const float *phi;
	float *c;
	memset(PHIP, 0, sizeof(float)*n*n);

	phi = PHI;
	c = PHIP;
	for (j = 0; j < 3; ++j)
	{
		for (i = 0; i < n; ++i)
		{
			c[i] = phi[0] * P[i] + phi[1] * P[i + n] + phi[2] * P[i + 2 * n] + phi[3 + j] * P[i + (3 + j)*n];
		}
		phi += n;
		c += n;
	}

	for (j = 0; j < 3; ++j)
	{
		for (i = 0; i < n; ++i)
		{
			c[i] = phi[0] * P[i] + phi[1] * P[i + n] + phi[2] * P[i + 2 * n] + phi[3] * P[i + 3 * n] + phi[4] * P[i + 4 * n] + phi[5] * P[i + 5 * n]
				+ phi[6] * P[i + 6 * n] + phi[7] * P[i + 7 * n] + phi[8] * P[i + 8 * n]
				+ phi[12] * P[i + 12 * n] + phi[13] * P[i + 13 * n] + phi[14] * P[i + 14 * n];
		}
		phi += n;
		c += n;
	}

	for (j = 0; j < 3; ++j)
	{
		for (i = 0; i < n; ++i)
		{
			c[i] = phi[6] * P[i + 6 * n] + phi[7] * P[i + 7 * n] + phi[8] * P[i + 8 * n]
				+ phi[9] * P[i + 9 * n] + phi[10] * P[i + 10 * n] + phi[11] * P[i + 11 * n];
		}
		phi += n;
		c += n;
	}

	for (j = 0; j < 6; ++j)
	{
		for (i = 0; i < n; ++i)
		{
			c[i] = phi[9 + j] * P[i + (9 + j)*n];
		}
		phi += n;
		c += n;
	}

	if (16 == n)
	{
		for (i = 0; i < n; ++i)
		{
			c[i] = P[i + (n-1) * n];
		}
	}
}

//PHI*Q
void PHI_Q(const int16_t n,const float *PHI,const float *Q, float *PHIQ)
{
	unsigned char j;
	const float *phi;
	float *c;

	memset(PHIQ, 0, sizeof(float)*n*n);
	phi = PHI;
	c = PHIQ;
	for (j = 0; j < 3; ++j)
	{
		c[0] = phi[0] * Q[0];
		c[1] = phi[1] * Q[1];
		c[2] = phi[2] * Q[2];
		c[3 + j] = phi[3 + j] * Q[3 + j];
		phi += n;
		c += n;
	}

	for (j = 0; j < 3; ++j)
	{
		c[0] = phi[0] * Q[0];
		c[1] = phi[1] * Q[1];
		c[2] = phi[2] * Q[2];

		c[3] = phi[3] * Q[3];
		c[4] = phi[4] * Q[4];
		c[5] = phi[5] * Q[5];

		c[6] = phi[6] * Q[6];
		c[7] = phi[7] * Q[7];
		c[8] = phi[8] * Q[8];

		c[12] = phi[12] * Q[12];
		c[13] = phi[13] * Q[13];
		c[14] = phi[14] * Q[14]; // fix wrong. ymj

		phi += n;
		c += n;
	}

	for (j = 0; j < 3; ++j)
	{
		c[6] = phi[6] * Q[6];
		c[7] = phi[7] * Q[7];
		c[8] = phi[8] * Q[8];

		c[9] = phi[9] * Q[9];
		c[10] = phi[10] * Q[10];
		c[11] = phi[11] * Q[11];

		phi += n;
		c += n;
	}

	for (j = 0; j < 6; ++j)
	{
		c[9 + j] = phi[j + 9] * Q[j + 9];
		phi += n;
		c += n;
	}

	if (16 == n)
	{
		c[15] = Q[15];
	}
}

//PHIP*PHI_T
void PHIP_PHIT(const int16_t n, const float *PHIP, const float *PHI, float *PHIPPHIT)
{
	unsigned char i, j;
	const float *phi, *phip;
	float *c;
	phi = PHI;
	c = PHIPPHIT;
	phip = PHIP;

	for (i = 0; i < n; ++i)
	{
		phi = PHI;
		for (j = 0; j < 3; ++j)
		{
			c[j] = phip[0] * phi[0] + phip[1] * phi[1] + phip[2] * phi[2] + phip[3 + j] * phi[3 + j];
			phi += n;
		}
		for (j = 3; j < 6; ++j)
		{
			c[j] = phip[0] * phi[0] + phip[1] * phi[1] + phip[2] * phi[2]
				+ phip[3] * phi[3] + phip[4] * phi[4] + phip[5] * phi[5]
				+ phip[6] * phi[6] + phip[7] * phi[7] + phip[8] * phi[8]
				+ phip[12] * phi[12] + phip[13] * phi[13] + phip[14] * phi[14];
			phi += n;
		}
		for (j = 6; j < 9; ++j)
		{
			c[j] = phip[9] * phi[9] + phip[10] * phi[10] + phip[11] * phi[11]
				+ phip[6] * phi[6] + phip[7] * phi[7] + phip[8] * phi[8];
			phi += n;
		}

		for (j = 9; j < 15; ++j)
		{
			c[j] = phip[j] * phi[j];
			phi += n;
		}

		if (16 == n)
		{
			c[15] = phip[15];
		}

		phip += n;
		c += n;
	}
}

//0.5*(PHI*Q+Q*PHIT)*dt
void PHIQ_QPHIT(const int16_t n,float *PHIQ, float dt, float *c)
{
	unsigned char i, j;
	dt = dt * 0.5;
	for (i = 0; i < n; ++i)
	{
		for (j = 0; j < n; ++j)
		{
			c[i + j * n] = dt * (PHIQ[i + j * n] + PHIQ[j + i * n]);
		}
	}
}


	uint8_t MatrixAdd(const double *matrix_a, const double *matrix_b, const int matrix_a_row, const int matrix_a_colume, double *matrix_result)
	{
		for (int i = 0; i < matrix_a_row*matrix_a_colume; i++)
		{
			*(matrix_result + i) = *(matrix_a + i) + *(matrix_b + i);
		}
		return 1;
	}
	uint8_t MatrixSub(const double *matrix_a, const double *matrix_b, const int matrix_a_row, const int matrix_a_colume, double *matrix_result)
	{
		for (int i = 0; i < matrix_a_row*matrix_a_colume; i++)
		{
			*(matrix_result + i) = *(matrix_a + i) - *(matrix_b + i);
		}
		return 1;

	}
	uint8_t MatrixMutiply(const double *matrix_a, const double *matrix_b, const int matrix_a_row, const int matrix_a_column, const int matrix_b_column, double *matrix_result)
	{
		double sum = 0;
		double median = 0;
		for (int i = 0; i < matrix_a_row; i++)
		{
			for (int k = 0; k < matrix_b_column; k++)
			{
				for (int j = 0; j < matrix_a_column; j++)
				{
					median = matrix_a[matrix_a_column*i + j] * matrix_b[matrix_b_column*j + k];
					sum = sum + median;
				}
				matrix_result[matrix_b_column*i + k] = sum;
				sum = 0;
			}
		}
		return 1;

	}
	uint8_t MatrixTranspose(const double *matrix_a, const int matrix_a_row, const int matrix_a_column, double *matrix_result)
	{
		for (int i = 0; i < matrix_a_column; i++)
		{
			for (int j = 0; j < matrix_a_row; j++)
			{
				matrix_result[matrix_a_row*i + j] = matrix_a[matrix_a_column*j + i];
			}
		}
		return 1;

	}


	uint8_t MatrixAddfloat(const float *matrix_a, const float *matrix_b, const int matrix_a_row, const int matrix_a_colume, float *matrix_result)
	{
		for (int i = 0; i < matrix_a_row*matrix_a_colume; i++)
		{
			*(matrix_result + i) = *(matrix_a + i) + *(matrix_b + i);
		}
		return 1;
	}
	uint8_t MatrixSubfloat(const float *matrix_a, const float *matrix_b, const int matrix_a_row, const int matrix_a_colume, float *matrix_result)
	{
		for (int i = 0; i < matrix_a_row*matrix_a_colume; i++)
		{
			*(matrix_result + i) = *(matrix_a + i) - *(matrix_b + i);
		}
		return 1;

	}
	uint8_t MatrixMutiplyfloat(const float *matrix_a, const float *matrix_b, const int matrix_a_row, const int matrix_a_column, const int matrix_b_column, float *matrix_result)
	{
		float sum = 0;
		float median = 0;
		for (int i = 0; i < matrix_a_row; i++)
		{
			for (int k = 0; k < matrix_b_column; k++)
			{
				for (int j = 0; j < matrix_a_column; j++)
				{
					median = matrix_a[matrix_a_column*i + j] * matrix_b[matrix_b_column*j + k];
					sum = sum + median;
				}
				matrix_result[matrix_b_column*i + k] = sum;
				sum = 0;
			}
		}
		return 1;

	}
	uint8_t MatrixTransposefloat(const float *matrix_a, const int matrix_a_row, const int matrix_a_column, float *matrix_result)
	{
		for (int i = 0; i < matrix_a_column; i++)
		{
			for (int j = 0; j < matrix_a_row; j++)
			{
				matrix_result[matrix_a_row*i + j] = matrix_a[matrix_a_column*j + i];
			}
		}
		return 1;

	}
	uint8_t MatrixMutiplyfloatd(const float *matrix_a, const double *matrix_b, const int matrix_a_row, const int matrix_a_column, const int matrix_b_column, double *matrix_result)
	{
		double sum = 0;
		double median = 0;
		for (int i = 0; i < matrix_a_row; i++)
		{
			for (int k = 0; k < matrix_b_column; k++)
			{
				for (int j = 0; j < matrix_a_column; j++)
				{
					median =(double)(matrix_a[matrix_a_column*i + j]) * matrix_b[matrix_b_column*j + k];
					sum = sum + median;
				}
				matrix_result[matrix_b_column*i + k] = sum;
				sum = 0;
			}
		}
		return 1;

	}

	/*************************************************
	Function: MatrixCholosky
	Description:����Cholosky�ֽ�
	Input:
	Output:lower_tri_matrix
	Return:
	Others:
	*************************************************/
	uint8_t MatrixCholosky(const double *matrix, const int matrix_row, double *lower_tri_matrix)
	{
		int i, j, k, l, u;
		double sum = 0.0;
		if (matrix[0] <= 0.0)
		{
#ifdef DEBUG
			printf(" The matrix can't be decomposed with Cholosky decomposition method.\n");
#endif
		}
		lower_tri_matrix[0] = sqrt(matrix[0]);

		for (i = 1; i < matrix_row; i++)
			lower_tri_matrix[i*matrix_row] = matrix[i*matrix_row] / lower_tri_matrix[0];


		for (k = 1; k < matrix_row; k++)
		{
			l = k * matrix_row + k;
			for (j = 0; j < k; j++)
				sum = sum + lower_tri_matrix[k*matrix_row + j] * lower_tri_matrix[k*matrix_row + j];

			if ((lower_tri_matrix[l] - sum) <= 0.0)
			{
#ifdef DEBUG
				printf(" The matrix can't be decomposed with Cholosky decomposition method.\n");
#endif
			}
			lower_tri_matrix[l] = sqrt(matrix[l] - sum);
			sum = 0.0;
			for (i = k + 1; i < matrix_row; i++)
			{
				u = i * matrix_row + k;
				for (j = 0; j < k; j++)
					sum = sum + lower_tri_matrix[i*matrix_row + j] * lower_tri_matrix[k*matrix_row + j];
				lower_tri_matrix[u] = (matrix[u] - sum) / lower_tri_matrix[l];
				sum = 0.0;
			}
		}

		for (i = 0; i < matrix_row - 1; i++)
		{
			for (j = i + 1; j < matrix_row; j++)
				lower_tri_matrix[i*matrix_row + j] = 0.0;
		}
		return 1;
	}
	/*************************************************
	Function: MatrixInverse
	Description:L��������
	Input:
	Output:matrix_result
	Return:
	Others:
	*************************************************/
//	uint8_t MatrixInverse(const double *lower_tri_matrix, const int matrix_row, double *matrix_result)
//	{
//		int ret = 0;
//		int i, j, k, l, u;
//		double sum;
//		for (i = 0; i < matrix_row; i++)
//		{
//			l = i * matrix_row + i;
//			if (lower_tri_matrix[l] <= 0)
//			{
//#ifdef DEBUG
//				printf(" The matrix does not exist inverse matrix.\n");
//				return -1;
//#endif
//			}
//			matrix_result[l] = 1.0 / lower_tri_matrix[l];
//		}
//
//		for (i = 1; i < matrix_row; i++)
//		{
//			sum = 0.0;
//			for (j = 0; j < i; j++)
//			{
//				for (k = j; k < i; k++)
//				{
//					l = i * matrix_row + k;
//					u = k * matrix_row + j;
//					sum = sum + lower_tri_matrix[l] * matrix_result[u];
//				}
//				matrix_result[i*matrix_row + j] = -matrix_result[i*matrix_row + i] * sum;
//				sum = 0.0;
//			}
//		}
//
//		for (i = 0; i < matrix_row - 1; i++)
//		{
//			for (j = i + 1; j < matrix_row; j++)
//			{
//				l = i * matrix_row + j;
//				matrix_result[l] = 0.0;
//			}
//		}
//		return 1;
//	}
	static unsigned short l, u;
	static char is[25], js[25];
	uint8_t MatrixInverse(uint8_t n, double* a)
	{
		int i, j, k;
		unsigned short v;
		double d, p;
		for (k = 0; k <= n - 1; k++)
		{
			d = 0.0;
			for (i = k; i <= n - 1; i++)
				for (j = k; j <= n - 1; j++)
				{
					l = i * n + j; p = fabs(a[l]);
					if (p > d) { d = p; is[k] = i; js[k] = j; }
				}
			if (is[k] != k)
				for (j = 0; j <= n - 1; j++)
				{
					u = k * n + j; v = is[k] * n + j;
					p = a[u]; a[u] = a[v]; a[v] = p;
				}
			if (js[k] != k)
				for (i = 0; i <= n - 1; i++)
				{
					u = i * n + k; v = i * n + js[k];
					p = a[u]; a[u] = a[v]; a[v] = p;
				}
			l = k * n + k;
			a[l] = 1.0 / a[l];
			for (j = 0; j <= n - 1; j++)
				if (j != k)
				{
					u = k * n + j; a[u] = a[u] * a[l];
				}
			for (i = 0; i <= n - 1; i++)
				if (i != k)
					for (j = 0; j <= n - 1; j++)
						if (j != k)
						{
							u = i * n + j;
							a[u] = a[u] - a[i*n + k] * a[k*n + j];
						}
			for (i = 0; i <= n - 1; i++)
				if (i != k)
				{
					u = i * n + k; a[u] = -a[u] * a[l];
				}
		}
		for (k = n - 1; k >= 0; k--)
		{
			if (js[k] != k)
				for (j = 0; j <= n - 1; j++)
				{
					u = k * n + j; v = js[k] * n + j;
					p = a[u]; a[u] = a[v]; a[v] = p;
				}
			if (is[k] != k)
				for (i = 0; i <= n - 1; i++)
				{
					u = i * n + k; v = i * n + is[k];
					p = a[u]; a[u] = a[v]; a[v] = p;
				}
		}
		return 1;

	}
	// Cross product of two vectors(c = a x b)
	uint8_t CrossProduct(double a[3], double b[3], double c[3])
	{
		c[0] = a[1] * b[2] - b[1] * a[2];
		c[1] = b[0] * a[2] - a[0] * b[2];
		c[2] = a[0] * b[1] - b[0] * a[1];
		return 1;

	}
	uint8_t GetSkewSymmetricMatrixOfVector(const double  mvector[3], double *M)
	{
		M[0] = 0; M[1] = -mvector[2]; M[2] = mvector[1];
		M[3] = mvector[2]; M[4] = 0; M[5] = -mvector[0];
		M[6] = -mvector[1]; M[7] = mvector[0]; M[8] = 0;
		return 1;

	}
	uint8_t quatprod(const double p[4], const double q[4], double r[4])
	{
		r[0] = p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3];
		r[1] = p[1] * q[0] + p[0] * q[1] - p[3] * q[2] + p[2] * q[3];
		r[2] = p[2] * q[0] + p[3] * q[1] + p[0] * q[2] - p[1] * q[3];
		r[3] = p[3] * q[0] - p[2] * q[1] + p[1] * q[2] + p[0] * q[3];
		return 1;

	}
	uint8_t quat2rvec(double q[4], double rot_vec[3])
	{
		int i;
		if (fabs(q[0]) > 1.0E-15)
		{
			double atan_05zata = atan2(sqrt(q[1] * q[1] + q[2] * q[2] + q[3] * q[3]), q[0]);
			double atan2_05zata = atan_05zata * atan_05zata;
			double atan4_05zata = atan2_05zata * atan2_05zata;
			double atan6_05zata = atan4_05zata * atan2_05zata;
			double f = 0.5*(1 - atan2_05zata / 6 + atan4_05zata / 120 - atan6_05zata / 5040);
			for (i = 0; i < 3; ++i)
			{
				rot_vec[i] = q[i + 1] / f;
			}
		}
		else
		{
			for (i = 0; i < 3; ++i)
			{
				rot_vec[i] = q[i + 1] * PI;
			}
		}
		return 1;

	}

	uint8_t rvec2quat(double rot_vec[3], double q[4])
	{
		double mag2, c, s;
		mag2 = rot_vec[0] * rot_vec[0] + rot_vec[1] * rot_vec[1] + rot_vec[2] * rot_vec[2];

		if (mag2 < PI*PI)
		{
			mag2 = 0.25*mag2;

			c = 1.0 - mag2 / 2.0*(1.0 - mag2 / 12.0*(1.0 - mag2 / 30.0));
			s = 1.0 - mag2 / 6.0*(1.0 - mag2 / 20.0*(1.0 - mag2 / 42.0));

			if (c < 0)
			{
				q[0] = -c;
				q[1] = -0.5*s*rot_vec[0];
				q[2] = -0.5*s*rot_vec[1];
				q[3] = -0.5*s*rot_vec[2];
			}
			else
			{
				q[0] = c;
				q[1] = 0.5*s*rot_vec[0];
				q[2] = 0.5*s*rot_vec[1];
				q[3] = 0.5*s*rot_vec[2];
			}
		}
		else
		{
			c = sqrt(mag2);
			s = sin(c / 2);
			mag2 = s / c;

			q[0] = cos(c / 2);
			q[1] = rot_vec[0] * mag2;
			q[2] = rot_vec[1] * mag2;
			q[3] = rot_vec[2] * mag2;
			if (q[0] < 0)
			{
				q[0] = -q[0];
				q[1] = -q[1];
				q[2] = -q[2];
				q[3] = -q[3];
			}
		}
		return 1;

	}
	uint8_t quat2pos(double q[4], double *r)
	{
		r[0] = -2 * atan(q[2] / q[0]) - PI / 2;
		r[1] = 2 * atan2(q[3], q[0]);
		return 1;

	}
	uint8_t norm_quat(double q[4])
	{
		double e = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3] - 1) / 2;

		q[0] = (1 - e)*q[0];
		q[1] = (1 - e)*q[1];
		q[2] = (1 - e)*q[2];
		q[3] = (1 - e)*q[3];
		return 1;

	}
	uint8_t quatinv(const double p[4], double q[4])
	{
		double a = sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2] + p[3] * p[3]);
		if (a < 1.0E-15)
		{
			q[0] = 1.0; q[1] = 0.0; q[2] = 0.0; q[3] = 0.0;
			return -1;
		}
		q[0] = p[0] / a;
		q[1] = -p[1] / a;
		q[2] = -p[2] / a;
		q[3] = -p[3] / a;
		return 1;

	}

