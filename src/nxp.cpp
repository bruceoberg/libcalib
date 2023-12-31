// Copyright (c) 2014, Freescale Semiconductor, Inc.
// All rights reserved.
// vim: set ts=4:
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Freescale Semiconductor, Inc. nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL FREESCALE SEMICONDUCTOR, INC. BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// This is the file that contains the fusion routines.  It is STRONGLY RECOMMENDED
// that the casual developer NOT TOUCH THIS FILE.  The mathematics behind this file
// is extremely complex, and it will be very easy (almost inevitable) that you screw
// it up.
//

#include "libcalib_nxp.h"
#include "matrix.h"

#include <string.h>
#include <math.h>

namespace libcalib
{

namespace _9DOF_GBY_KALMAN
{
	// kalman filter noise variances
	constexpr float FQVA = 2E-6F;				// accelerometer noise g^2 so 1.4mg RMS
	constexpr float FQVM = 0.1F;				// magnetometer noise uT^2
	constexpr float FQVG = 0.3F;				// gyro noise (deg/s)^2
	constexpr float FQWB = 1E-9F;				// gyro offset drift (deg/s)^2: 1E-9 implies 0.09deg/s max at 50Hz
	constexpr float FQWA = 1E-4F;				// linear acceleration drift g^2 (increase slows convergence to g but reduces sensitivity to shake)
	constexpr float FQWD = 0.5F;				// magnetic disturbance drift uT^2 (increase slows convergence to B but reduces sensitivity to magnet)
	// initialization of Qw covariance matrix
	constexpr float FQWINITTHTH  = 2000E-5F;	// th_e * th_e terms
	constexpr float FQWINITBB  = 250E-3F;		// b_e * b_e terms
	constexpr float FQWINITTHB  = 0.0F;			// th_e * b_e terms
	constexpr float FQWINITAA  = 10E-5F;		// a_e * a_e terms (increase slows convergence to g but reduces sensitivity to shake)
	constexpr float FQWINITDD = 600E-3F;		// d_e * d_e terms (increase slows convergence to B but reduces sensitivity to magnet)
	// linear acceleration and magnetic disturbance time constants
	constexpr float FCA = 0.5F;					// linear acceleration decay factor
	constexpr float FCD = 0.5F;					// magnetic disturbance decay factor
} // namespace _9DOF_GBY_KALMAN
using namespace _9DOF_GBY_KALMAN;

// maximum geomagnetic inclination angle tracked by Kalman filter
constexpr float SINDELTAMAX = 0.9063078F;		// sin of max +ve geomagnetic inclination angle: here 65.0 deg
constexpr float COSDELTAMAX = 0.4226183F;		// cos of max +ve geomagnetic inclination angle: here 65.0 deg
constexpr float DEFAULTB = 50.0F;				// default geomagnetic field (uT)
constexpr int X = 0;							// vector components
constexpr int Y = 1;
constexpr int Z = 2;
constexpr float FDEGTORAD = 0.01745329251994F;	// degrees to radians conversion = pi / 180
constexpr float FRADTODEG = 57.2957795130823F;	// radians to degrees conversion = 180 / pi
constexpr float ONEOVER48 = 0.02083333333F;		// 1 / 48
constexpr float ONEOVER3840 = 0.0002604166667F;	// 1 / 3840



void fqAeq1(Quaternion_t *pqA);
void feCompassNED(float fR[][3], float *pfDelta, const float fBc[], const float fGp[]);
void fNEDAnglesDegFromRotationMatrix(float R[][3], float *pfPhiDeg,
	float *pfTheDeg, float *pfPsiDeg, float *pfRhoDeg, float *pfChiDeg);
void fQuaternionFromRotationMatrix(float R[][3], Quaternion_t *pq);
void fQuaternionFromRotationVectorDeg(Quaternion_t *pq, const float rvecdeg[], float fscaling);
void fRotationMatrixFromQuaternion(float R[][3], const Quaternion_t *pq);
void fRotationVectorDegFromQuaternion(Quaternion_t *pq, float rvecdeg[]);
void qAeqAxB(Quaternion_t *pqA, const Quaternion_t *pqB);
void qAeqBxC(Quaternion_t *pqA, const Quaternion_t *pqB, const Quaternion_t *pqC);
// Quaternion_t qconjgAxB(const Quaternion_t *pqA, const Quaternion_t *pqB);
void fqAeqNormqA(Quaternion_t *pqA);
float fasin_deg(float x);
float facos_deg(float x);
float fatan_deg(float x);
float fatan2_deg(float y, float x);
float fatan_15deg(float x);

// function initializes the 9DOF Kalman filter
void Nxp::init()
{
	int8_t i, j;				// loop counters

	// reset the flag denoting that a first 9DOF orientation lock has been achieved
	m_FirstOrientationLock = 0;

	// compute and store useful product terms to save floating point calculations later
	m_Fastdeltat = 1.0F / (float)SENSORFS;
	m_deltat = (float)OVERSAMPLE_RATIO * m_Fastdeltat;
	m_deltatsq = m_deltat * m_deltat;
	m_casq = FCA * FCA;
	m_cdsq = FCD * FCD;
	m_QwbplusQvG = FQWB + FQVG;

	// initialize the fixed entries in the measurement matrix C
	for (i = 0; i < 6; i++) {
		for (j = 0; j < 12; j++) {
			m_C6x12[i][j]= 0.0F;
		}
	}
	m_C6x12[0][6] = m_C6x12[1][7] = m_C6x12[2][8] = 1.0F;
	m_C6x12[3][9] = m_C6x12[4][10] = m_C6x12[5][11] = -1.0F;

	// zero a posteriori orientation, error vector xe+ (thetae+, be+, de+, ae+) and b+ and inertial
	f3x3matrixAeqI(m_RPl);
	fqAeq1(&(m_qPl));
	for (i = X; i <= Z; i++) {
		m_ThErrPl[i] = m_bErrPl[i] = m_aErrSePl[i] = m_dErrSePl[i] = m_bPl[i] = 0.0F;
	}

	// initialize the reference geomagnetic vector (uT, global frame)
	m_DeltaPl = 0.0F;
	// initialize NED geomagnetic vector to zero degrees inclination
	m_mGl[X] = DEFAULTB;
	m_mGl[Y] = 0.0F;
	m_mGl[Z] = 0.0F;

	// initialize noise variances for Qv and Qw matrix updates
	m_QvAA = FQVA + FQWA + FDEGTORAD * FDEGTORAD * m_deltatsq * (FQWB + FQVG);
	m_QvMM = FQVM + FQWD + FDEGTORAD * FDEGTORAD * m_deltatsq * DEFAULTB * DEFAULTB * (FQWB + FQVG);

	// initialize the 12x12 noise covariance matrix Qw of the a priori error vector xe-
	// Qw is then recursively updated as P+ = (1 - K * C) * P- = (1 - K * C) * Qw  and Qw
	// updated from P+
	// zero the matrix Qw
	for (i = 0; i < 12; i++) {
		for (j = 0; j < 12; j++) {
			m_Qw12x12[i][j] = 0.0F;
		}
	}
	// loop over non-zero values
	for (i = 0; i < 3; i++) {
		// theta_e * theta_e terms
		m_Qw12x12[i][i] = FQWINITTHTH;
		// b_e * b_e terms
		m_Qw12x12[i + 3][i + 3] = FQWINITBB;
		// th_e * b_e terms
		m_Qw12x12[i][i + 3] = m_Qw12x12[i + 3][i] = FQWINITTHB;
		// a_e * a_e terms
		m_Qw12x12[i + 6][i + 6] = FQWINITAA;
		// d_e * d_e terms
		m_Qw12x12[i + 9][i + 9] = FQWINITDD;
	}

	// clear the reset flag
	m_resetflag = 0;
}




// 9DOF orientation function implemented using a 12 element Kalman filter
void Nxp::update(
	const AccelSensor_t *Accel,
	const MagSensor_t *Mag,
	const GyroSensor_t *Gyro,
	bool isBCurValid,
	float BCur)
{
	// local scalars and arrays
	float fopp, fadj, fhyp;						// opposite, adjacent and hypoteneuse
	float fsindelta, fcosdelta;					// sin and cos of inclination angle delta
	float rvec[3];								// rotation vector
	float ftmp;									// scratch variable
	float ftmpA12x6[12][6];						// scratch array
	int8_t i, j, k;								// loop counters
	int8_t iMagJamming;							// magnetic jamming flag

	// assorted array pointers
	float *pfPPlus12x12ij;
	float *pfPPlus12x12kj;
	float *pfQw12x12ij;
	float *pfQw12x12ik;
	float *pfQw12x12kj;
	float *pfK12x6ij;
	float *pfK12x6ik;
	float *pftmpA12x6ik;
	float *pftmpA12x6kj;
	float *pftmpA12x6ij;
	float *pfC6x12ik;
	float *pfC6x12jk;

	// working arrays for 6x6 matrix inversion
	float *pfRows[6];
	int8_t iColInd[6];
	int8_t iRowInd[6];
	int8_t iPivot[6];

	// do a reset and return if requested
	if (m_resetflag) {
		init();
		return;
	}

	// *********************************************************************************
	// initial orientation lock to accelerometer and magnetometer eCompass orientation
	// *********************************************************************************

	// do a once-only orientation lock after the first valid magnetic calibration
	if (isBCurValid && !m_FirstOrientationLock) {
		// get the 6DOF orientation matrix and initial inclination angle
		feCompassNED(m_RPl, &(m_DeltaPl), Mag->BcFast, Accel->GpFast);

		// get the orientation quaternion from the orientation matrix
		fQuaternionFromRotationMatrix(m_RPl, &(m_qPl));

		// set the orientation lock flag so this initial alignment is only performed once
		m_FirstOrientationLock = 1;
	}

	// *********************************************************************************
	// calculate a priori rotation matrix
	// *********************************************************************************

	// compute the angular velocity from the averaged high frequency gyro reading.
	// omega[k] = yG[k] - b-[k] = yG[k] - b+[k-1] (deg/s)
	m_Omega[X] = Gyro->Yp[X] - m_bPl[X];
	m_Omega[Y] = Gyro->Yp[Y] - m_bPl[Y];
	m_Omega[Z] = Gyro->Yp[Z] - m_bPl[Z];

	// initialize the a priori orientation quaternion to the previous a posteriori estimate
	m_qMi = m_qPl;

	// integrate the buffered high frequency (typically 200Hz) gyro readings
	for (j = 0; j < OVERSAMPLE_RATIO; j++) {
		// compute the incremental fast (typically 200Hz) rotation vector rvec (deg)
		for (i = X; i <= Z; i++) {
			rvec[i] = (Gyro->YpFast[j][i] - m_bPl[i]) * m_Fastdeltat;
		}

		// compute the incremental quaternion fDeltaq from the rotation vector
		fQuaternionFromRotationVectorDeg(&(m_Deltaq), rvec, 1.0F);

		// incrementally rotate the a priori orientation quaternion fqMi
		// the a posteriori quaternion fqPl is re-normalized later so this update is stable
		qAeqAxB(&(m_qMi), &(m_Deltaq));
	}

	// get the a priori rotation matrix from the a priori quaternion
	fRotationMatrixFromQuaternion(m_RMi, &(m_qMi));

	// *********************************************************************************
	// calculate a priori gyro, accelerometer and magnetometer estimates
	// of the gravity and geomagnetic vectors and errors
	// the most recent 'Fast' measurements are used to reduce phase errors
	// *********************************************************************************

	for (i = X; i <= Z; i++) {
		// compute the a priori gyro estimate of the gravitational vector (g, sensor frame)
		// using an absolute rotation of the global frame gravity vector (with magnitude 1g)
		// NED gravity is along positive z axis
		m_gSeGyMi[i] = m_RMi[i][Z];

		// compute a priori acceleration (a-) (g, sensor frame) from decayed a
		// posteriori estimate (g, sensor frame)
		m_aSeMi[i] = FCA * m_aSePl[i];

		// compute the a priori gravity error vector (accelerometer minus gyro estimates)
		// (g, sensor frame)
		// NED and Windows 8 have positive sign for gravity: y = g - a and g = y + a
		m_gErrSeMi[i] = Accel->GpFast[i] + m_aSeMi[i] - m_gSeGyMi[i];

		// compute the a priori gyro estimate of the geomagnetic vector (uT, sensor frame)
		// using an absolute rotation of the global frame geomagnetic vector (with magnitude m_cal_B uT)
		// NED y component of geomagnetic vector in global frame is zero
		m_mSeGyMi[i] = m_RMi[i][X] * m_mGl[X] + m_RMi[i][Z] * m_mGl[Z];

		// compute the a priori geomagnetic error vector (magnetometer minus gyro estimates)
		// (g, sensor frame)
		m_mErrSeMi[i] = Mag->BcFast[i] - m_mSeGyMi[i];
	}

	// *********************************************************************************
	// update variable elements of measurement matrix C
	// *********************************************************************************

	// update measurement matrix C with -alpha(g-)x and -alpha(m-)x from gyro (g, uT, sensor frame)
	m_C6x12[0][1] = FDEGTORAD * m_gSeGyMi[Z];
	m_C6x12[0][2] = -FDEGTORAD * m_gSeGyMi[Y];
	m_C6x12[1][2] = FDEGTORAD * m_gSeGyMi[X];
	m_C6x12[1][0] = -m_C6x12[0][1];
	m_C6x12[2][0] = -m_C6x12[0][2];
	m_C6x12[2][1] = -m_C6x12[1][2];
	m_C6x12[3][1] = FDEGTORAD * m_mSeGyMi[Z];
	m_C6x12[3][2] = -FDEGTORAD * m_mSeGyMi[Y];
	m_C6x12[4][2] = FDEGTORAD * m_mSeGyMi[X];
	m_C6x12[4][0] = -m_C6x12[3][1];
	m_C6x12[5][0] = -m_C6x12[3][2];
	m_C6x12[5][1] = -m_C6x12[4][2];
	m_C6x12[0][4] = -m_deltat * m_C6x12[0][1];
	m_C6x12[0][5] = -m_deltat * m_C6x12[0][2];
	m_C6x12[1][5] = -m_deltat * m_C6x12[1][2];
	m_C6x12[1][3]= -m_C6x12[0][4];
	m_C6x12[2][3]= -m_C6x12[0][5];
	m_C6x12[2][4]= -m_C6x12[1][5];
	m_C6x12[3][4] = -m_deltat * m_C6x12[3][1];
	m_C6x12[3][5] = -m_deltat * m_C6x12[3][2];
	m_C6x12[4][5] = -m_deltat * m_C6x12[4][2];
	m_C6x12[4][3] = -m_C6x12[3][4];
	m_C6x12[5][3] = -m_C6x12[3][5];
	m_C6x12[5][4] = -m_C6x12[4][5];

	// *********************************************************************************
	// calculate the Kalman gain matrix K
	// K = P- * C^T * inv(C * P- * C^T + Qv) = Qw * C^T * inv(C * Qw * C^T + Qv)
	// Qw is used as a proxy for P- throughout the code
	// P+ is used here as a working array to reduce RAM usage and is re-computed later
	// *********************************************************************************

	// set ftmpA12x6 = P- * C^T = Qw * C^T where Qw and C are both sparse
	// C also has a significant number of +1 and -1 entries
	// ftmpA12x6 is also sparse but not symmetric
	for (i = 0; i < 12; i++) { // loop over rows of ftmpA12x6
		// initialize pftmpA12x6ij for current i, j=0
		pftmpA12x6ij = ftmpA12x6[i];

		for (j = 0; j < 6; j++) { // loop over columns of ftmpA12x6
			// zero ftmpA12x6[i][j]
			*pftmpA12x6ij = 0.0F;

			// initialize pfC6x12jk for current j, k=0
			pfC6x12jk = m_C6x12[j];

			// initialize pfQw12x12ik for current i, k=0
			pfQw12x12ik = m_Qw12x12[i];

			// sum matrix products over inner loop over k
			for (k = 0; k < 12; k++) {
				if ((*pfQw12x12ik != 0.0F) && (*pfC6x12jk != 0.0F)) {
					if (*pfC6x12jk == 1.0F)
						*pftmpA12x6ij += *pfQw12x12ik;
					else if (*pfC6x12jk == -1.0F)
						*pftmpA12x6ij -= *pfQw12x12ik;
					else
						*pftmpA12x6ij += *pfQw12x12ik * *pfC6x12jk;
				}

				// increment pfC6x12jk and pfQw12x12ik for next iteration of k
				pfC6x12jk++;
				pfQw12x12ik++;

			}

			// increment pftmpA12x6ij for next iteration of j
			pftmpA12x6ij++;

		}
	}

	// set symmetric P+ (6x6 scratch sub-matrix) to C * P- * C^T + Qv
	// = C * (Qw * C^T) + Qv = C * ftmpA12x6 + Qv
	// both C and ftmpA12x6 are sparse but not symmetric
	for (i = 0; i < 6; i++) { // loop over rows of P+
		// initialize pfPPlus12x12ij for current i, j=i
		pfPPlus12x12ij = m_PPlus12x12[i] + i;

		for (j = i; j < 6; j++) { // loop over above diagonal columns of P+
			// zero P+[i][j]
			*pfPPlus12x12ij = 0.0F;

			// initialize pfC6x12ik for current i, k=0
			pfC6x12ik = m_C6x12[i];

			// initialize pftmpA12x6kj for current j, k=0
			pftmpA12x6kj = *ftmpA12x6 + j;

			// sum matrix products over inner loop over k
			for (k = 0; k < 12; k++) {
				if ((*pfC6x12ik != 0.0F) && (*pftmpA12x6kj != 0.0F)) {
					if (*pfC6x12ik == 1.0F)
						*pfPPlus12x12ij += *pftmpA12x6kj;
					else if (*pfC6x12ik == -1.0F)
						*pfPPlus12x12ij -= *pftmpA12x6kj;
					else
						*pfPPlus12x12ij += *pfC6x12ik * *pftmpA12x6kj;
				}

				// update pfC6x12ik and pftmpA12x6kj for next iteration of k
				pfC6x12ik++;
				pftmpA12x6kj += 6;

			}

			// increment pfPPlus12x12ij for next iteration of j
			pfPPlus12x12ij++;

		}
	}

	// add in noise covariance terms to the diagonal
	m_PPlus12x12[0][0] +=  m_QvAA;
	m_PPlus12x12[1][1] +=  m_QvAA;
	m_PPlus12x12[2][2] +=  m_QvAA;
	m_PPlus12x12[3][3] +=  m_QvMM;
	m_PPlus12x12[4][4] +=  m_QvMM;
	m_PPlus12x12[5][5] +=  m_QvMM;

	// copy above diagonal elements of P+ (6x6 scratch sub-matrix) to below diagonal
	for (i = 1; i < 6; i++)
		for (j = 0; j < i; j++)
			m_PPlus12x12[i][j] = m_PPlus12x12[j][i];

	// calculate inverse of P+ (6x6 scratch sub-matrix) = inv(C * P- * C^T + Qv) = inv(C * Qw * C^T + Qv)
	for (i = 0; i < 6; i++) {
		pfRows[i] = m_PPlus12x12[i];
	}
	fmatrixAeqInvA(pfRows, iColInd, iRowInd, iPivot, 3);

	// set K = P- * C^T * inv(C * P- * C^T + Qv) = Qw * C^T * inv(C * Qw * C^T + Qv)
	// = ftmpA12x6 * P+ (6x6 sub-matrix)
	// ftmpA12x6 = Qw * C^T is sparse but P+ (6x6 sub-matrix) is not
	// K is not symmetric because C is not symmetric
	for (i = 0; i < 12; i++) { // loop over rows of K12x6
		// initialize pfK12x6ij for current i, j=0
		pfK12x6ij = m_K12x6[i];

		for (j = 0; j < 6; j++) { // loop over columns of K12x6
			// zero the matrix element fK12x6[i][j]
			*pfK12x6ij = 0.0F;

			// initialize pftmpA12x6ik for current i, k=0
			pftmpA12x6ik = ftmpA12x6[i];

			// initialize pfPPlus12x12kj for current j, k=0
			pfPPlus12x12kj = *m_PPlus12x12 + j;

			// sum matrix products over inner loop over k
			for (k = 0; k < 6; k++) {
				if (*pftmpA12x6ik != 0.0F) {
					*pfK12x6ij += *pftmpA12x6ik * *pfPPlus12x12kj;
				}

				// increment pftmpA12x6ik and pfPPlus12x12kj for next iteration of k
				pftmpA12x6ik++;
				pfPPlus12x12kj += 12;

			}

			// increment pfK12x6ij for the next iteration of j
			pfK12x6ij++;

		}
	}

	// *********************************************************************************
	// calculate a posteriori error estimate: xe+ = K * ze-
	// *********************************************************************************

	// first calculate all four error vector components using accelerometer error component only
	// for fThErrPl, fbErrPl, faErrSePl but also magnetometer for fdErrSePl
	for (i = X; i <= Z; i++) {
		m_ThErrPl[i] = m_K12x6[i][0] * m_gErrSeMi[X] +
				m_K12x6[i][1] * m_gErrSeMi[Y] +
				m_K12x6[i][2] * m_gErrSeMi[Z];
		m_bErrPl[i] = m_K12x6[i + 3][0] * m_gErrSeMi[X] +
				m_K12x6[i + 3][1] * m_gErrSeMi[Y] +
				m_K12x6[i + 3][2] * m_gErrSeMi[Z];
		m_aErrSePl[i] = m_K12x6[i + 6][0] * m_gErrSeMi[X] +
				m_K12x6[i + 6][1] * m_gErrSeMi[Y] +
				m_K12x6[i + 6][2] * m_gErrSeMi[Z];
		m_dErrSePl[i] = m_K12x6[i + 9][0] * m_gErrSeMi[X] +
				m_K12x6[i + 9][1] * m_gErrSeMi[Y] +
				m_K12x6[i + 9][2] * m_gErrSeMi[Z] +
				m_K12x6[i + 9][3] * m_mErrSeMi[X] +
				m_K12x6[i + 9][4] * m_mErrSeMi[Y] +
				m_K12x6[i + 9][5] * m_mErrSeMi[Z];
	}

	// set the magnetic jamming flag if there is a significant magnetic error power after calibration
	ftmp = m_dErrSePl[X] * m_dErrSePl[X] + m_dErrSePl[Y] * m_dErrSePl[Y] +
			m_dErrSePl[Z] * m_dErrSePl[Z];
	iMagJamming = (isBCurValid) && (ftmp > (4.0F * BCur * BCur));

	// add the remaining magnetic error terms if there is calibration and no magnetic jamming
	if (isBCurValid && !iMagJamming) {
		for (i = X; i <= Z; i++) {
			m_ThErrPl[i] += m_K12x6[i][3] * m_mErrSeMi[X] +
					m_K12x6[i][4] * m_mErrSeMi[Y] +
					m_K12x6[i][5] * m_mErrSeMi[Z];
			m_bErrPl[i] += m_K12x6[i + 3][3] * m_mErrSeMi[X] +
					m_K12x6[i + 3][4] * m_mErrSeMi[Y] +
					m_K12x6[i + 3][5] * m_mErrSeMi[Z];
			m_aErrSePl[i] += m_K12x6[i + 6][3] * m_mErrSeMi[X] +
					m_K12x6[i + 6][4] * m_mErrSeMi[Y] +
					m_K12x6[i + 6][5] * m_mErrSeMi[Z];
		}
	}

	// *********************************************************************************
	// apply the a posteriori error corrections to the a posteriori state vector
	// *********************************************************************************

	// get the a posteriori delta quaternion
	fQuaternionFromRotationVectorDeg(&(m_Deltaq), m_ThErrPl, -1.0F);

	// compute the a posteriori orientation quaternion fqPl = fqMi * Deltaq(-thetae+)
	// the resulting quaternion may have negative scalar component q0
	qAeqBxC(&(m_qPl), &(m_qMi), &(m_Deltaq));

	// normalize the a posteriori orientation quaternion to stop error propagation
	// the renormalization function ensures that the scalar component q0 is non-negative
	fqAeqNormqA(&(m_qPl));

	// compute the a posteriori rotation matrix from the a posteriori quaternion
	fRotationMatrixFromQuaternion(m_RPl, &(m_qPl));

	// compute the rotation vector from the a posteriori quaternion
	fRotationVectorDegFromQuaternion(&(m_qPl), m_RVecPl);

	// update the a posteriori gyro offset vector b+ and
	// assign the entire linear acceleration error vector to update the linear acceleration
	for (i = X; i <= Z; i++) {
		// b+[k] = b-[k] - be+[k] = b+[k] - be+[k] (deg/s)
		m_bPl[i] -= m_bErrPl[i];
		// a+ = a- - ae+ (g, sensor frame)
		m_aSePl[i] = m_aSeMi[i] - m_aErrSePl[i];
	}

	// compute the linear acceleration in the global frame from the accelerometer measurement (sensor frame).
	// de-rotate the accelerometer measurement from the sensor to global frame using the inverse (transpose)
	// of the a posteriori rotation matrix
	m_aGlPl[X] = m_RPl[X][X] * Accel->GpFast[X] + m_RPl[Y][X] * Accel->GpFast[Y] +
			m_RPl[Z][X] * Accel->GpFast[Z];
	m_aGlPl[Y] = m_RPl[X][Y] * Accel->GpFast[X] + m_RPl[Y][Y] * Accel->GpFast[Y] +
			m_RPl[Z][Y] * Accel->GpFast[Z];
	m_aGlPl[Z] = m_RPl[X][Z] * Accel->GpFast[X] + m_RPl[Y][Z] * Accel->GpFast[Y] +
			m_RPl[Z][Z] * Accel->GpFast[Z];
	// remove gravity and correct the sign if the coordinate system is gravity positive / acceleration negative
	// gravity positive NED
	m_aGlPl[X] = -m_aGlPl[X];
	m_aGlPl[Y] = -m_aGlPl[Y];
	m_aGlPl[Z] = -(m_aGlPl[Z] - 1.0F);

	// update the reference geomagnetic vector using magnetic disturbance error if valid calibration and no jamming
	if (isBCurValid && !iMagJamming) {
		// de-rotate the NED magnetic disturbance error de+ from the sensor to the global reference frame
		// using the inverse (transpose) of the a posteriori rotation matrix
		m_dErrGlPl[X] = m_RPl[X][X] * m_dErrSePl[X] +
				m_RPl[Y][X] * m_dErrSePl[Y] +
				m_RPl[Z][X] * m_dErrSePl[Z];
		m_dErrGlPl[Z] = m_RPl[X][Z] * m_dErrSePl[X] +
				m_RPl[Y][Z] * m_dErrSePl[Y] +
				m_RPl[Z][Z] * m_dErrSePl[Z];

		// compute components of the new geomagnetic vector
		// the north pointing component fadj must always be non-negative
		fopp = m_mGl[Z] - m_dErrGlPl[Z];
		fadj = m_mGl[X] - m_dErrGlPl[X];
		if (fadj < 0.0F) {
			fadj = 0.0F;
		}
		fhyp = sqrtf(fopp * fopp + fadj * fadj);

		// check for the pathological condition of zero geomagnetic field
		if (fhyp != 0.0F) {
			// compute the sine and cosine of the new geomagnetic vector
			ftmp = 1.0F / fhyp;
			fsindelta = fopp * ftmp;
			fcosdelta = fadj * ftmp;

			// limit the inclination angle between limits to prevent runaway
			if (fsindelta > SINDELTAMAX) {
				fsindelta = SINDELTAMAX;
				fcosdelta = COSDELTAMAX;
			} else if (fsindelta < -SINDELTAMAX) {
				fsindelta = -SINDELTAMAX;
				fcosdelta = COSDELTAMAX;
			}

			// compute the new geomagnetic vector (always north pointing)
			m_DeltaPl = fasin_deg(fsindelta);
			m_mGl[X] = BCur * fcosdelta;
			m_mGl[Z] = BCur * fsindelta;
		} // end hyp == 0.0F
	} // end m_isValid

	// *********************************************************************************
	// compute the a posteriori Euler angles from the orientation matrix
	// *********************************************************************************

	// calculate the NED Euler angles
	fNEDAnglesDegFromRotationMatrix(m_RPl, &(m_PhiPl), &(m_ThePl), &(m_PsiPl),
			&(m_RhoPl), &(m_ChiPl));

	// ***********************************************************************************
	// calculate (symmetric) a posteriori error covariance matrix P+
	// P+ = (I12 - K * C) * P- = (I12 - K * C) * Qw = Qw - K * (C * Qw)
	// both Qw and P+ are used as working arrays in this section
	// at the end of this section, P+ is valid but Qw is over-written
	// ***********************************************************************************

	// set P+ (6x12 scratch sub-matrix) to the product C (6x12) * Qw (12x12)
	// where both C and Qw are sparse and C has a significant number of +1 and -1 entries
	// the resulting matrix is sparse but not symmetric
	for (i = 0; i < 6; i++) {
		// initialize pfPPlus12x12ij for current i, j=0
		pfPPlus12x12ij = m_PPlus12x12[i];

		for (j = 0; j < 12; j++) {
			// zero P+[i][j]
			*pfPPlus12x12ij = 0.0F;

			// initialize pfC6x12ik for current i, k=0
			pfC6x12ik = m_C6x12[i];

			// initialize pfQw12x12kj for current j, k=0
			pfQw12x12kj = &m_Qw12x12[0][j];

			// sum matrix products over inner loop over k
			for (k = 0; k < 12; k++) {
				if ((*pfC6x12ik != 0.0F) && (*pfQw12x12kj != 0.0F)) {
					if (*pfC6x12ik == 1.0F)
						*pfPPlus12x12ij += *pfQw12x12kj;
					else if (*pfC6x12ik == -1.0F)
						*pfPPlus12x12ij -= *pfQw12x12kj;
					else
						*pfPPlus12x12ij += *pfC6x12ik * *pfQw12x12kj;
				}

				// update pfC6x12ik and pfQw12x12kj for next iteration of k
				pfC6x12ik++;
				pfQw12x12kj += 12;

			}

			// increment pfPPlus12x12ij for next iteration of j
			pfPPlus12x12ij++;

		}
	}

	// compute P+ = (I12 - K * C) * Qw = Qw - K * (C * Qw) = Qw - K * P+ (6x12 sub-matrix)
	// storing result P+ in Qw and over-writing Qw which is OK since Qw is later computed from P+
	// where working array P+ (6x12 sub-matrix) is sparse but K is not sparse
	// only on and above diagonal terms of P+ are computed since P+ is symmetric
	for (i = 0; i < 12; i++) {
		// initialize pfQw12x12ij for current i, j=i
		pfQw12x12ij = m_Qw12x12[i] + i;

		for (j = i; j < 12; j++) {
			// initialize pfK12x6ik for current i, k=0
			pfK12x6ik = m_K12x6[i];

			// initialize pfPPlus12x12kj for current j, k=0
			pfPPlus12x12kj = *m_PPlus12x12 + j;

			// compute on and above diagonal matrix entry
			for (k = 0; k < 6; k++) {
				// check for non-zero values since P+ (6x12 scratch sub-matrix) is sparse
				if (*pfPPlus12x12kj != 0.0F) {
					*pfQw12x12ij -= *pfK12x6ik * *pfPPlus12x12kj;
				}
				// increment pfK12x6ik and pfPPlus12x12kj for next iteration of k
				pfK12x6ik++;
				pfPPlus12x12kj += 12;
			}

			// increment pfQw12x12ij for next iteration of j
			pfQw12x12ij++;
		}
	}

	// Qw now holds the on and above diagonal elements of P+
	// so perform a simple copy to the all elements of P+
	// after execution of this code P+ is valid but Qw remains invalid
	for (i = 0; i < 12; i++) {
		// initialize pfPPlus12x12ij and pfQw12x12ij for current i, j=i
		pfPPlus12x12ij = m_PPlus12x12[i] + i;
		pfQw12x12ij = m_Qw12x12[i] + i;

		// copy the on-diagonal elements and increment pointers to enter loop at j=i+1
		*(pfPPlus12x12ij++) = *(pfQw12x12ij++);

		// loop over above diagonal columns j copying to below-diagonal elements
		for (j = i + 1; j < 12; j++) {
			*(pfPPlus12x12ij++)= m_PPlus12x12[j][i] = *(pfQw12x12ij++);
		}
	}

	// *********************************************************************************
	// re-create the noise covariance matrix Qw=fn(P+) for the next iteration
	// using the elements of P+ which are now valid
	// Qw was over-written earlier but is here recomputed (all elements)
	// *********************************************************************************

	// zero the matrix Qw
	for (i = 0; i < 12; i++) {
		for (j = 0; j < 12; j++) {
			m_Qw12x12[i][j] = 0.0F;
		}
	}

	// update the covariance matrix components
	for (i = 0; i < 3; i++) {
		// Qw[th-th-] = Qw[0-2][0-2] = E[th-(th-)^T] = Q[th+th+] + deltat^2 * (Q[b+b+] + (Qwb + QvG) * I)
		m_Qw12x12[i][i] = m_PPlus12x12[i][i] + m_deltatsq * (m_PPlus12x12[i + 3][i + 3] + m_QwbplusQvG);

		// Qw[b-b-] = Qw[3-5][3-5] = E[b-(b-)^T] = Q[b+b+] + Qwb * I
		m_Qw12x12[i + 3][i + 3] = m_PPlus12x12[i + 3][i + 3] + FQWB;

		// Qw[th-b-] = Qw[0-2][3-5] = E[th-(b-)^T] = -deltat * (Q[b+b+] + Qwb * I) = -deltat * Qw[b-b-]
		m_Qw12x12[i][i + 3] = m_Qw12x12[i + 3][i] = -m_deltat * m_Qw12x12[i + 3][i + 3];

		// Qw[a-a-] = Qw[6-8][6-8] = E[a-(a-)^T] = ca^2 * Q[a+a+] + Qwa * I
		m_Qw12x12[i + 6][i + 6] = m_casq * m_PPlus12x12[i + 6][i + 6] + FQWA;

		// Qw[d-d-] = Qw[9-11][9-11] = E[d-(d-)^T] = cd^2 * Q[d+d+] + Qwd * I
		m_Qw12x12[i + 9][i + 9] = m_cdsq * m_PPlus12x12[i + 9][i + 9] + FQWD;
	}
}

void Nxp::read(Quaternion_t* q)
{
	*q = m_qPl;
}







// compile time constants that are private to this file
constexpr float SMALLQ0 = 0.01F;		// limit of quaternion scalar component requiring special algorithm
constexpr float CORRUPTQUAT = 0.001F;	// threshold for deciding rotation quaternion is corrupt
constexpr float SMALLMODULUS = 0.01F;	// limit where rounding errors may appear

// Aerospace NED accelerometer 3DOF tilt function computing rotation matrix fR
void f3DOFTiltNED(float fR[][3], float fGp[])
{
	// the NED self-consistency twist occurs at 90 deg pitch

	// local variables
	int16_t i;				// counter
	float fmodGxyz;			// modulus of the x, y, z accelerometer readings
	float fmodGyz;			// modulus of the y, z accelerometer readings
	float frecipmodGxyz;	// reciprocal of modulus
	float ftmp;				// scratch variable

	// compute the accelerometer squared magnitudes
	fmodGyz = fGp[Y] * fGp[Y] + fGp[Z] * fGp[Z];
	fmodGxyz = fmodGyz + fGp[X] * fGp[X];

	// check for freefall special case where no solution is possible
	if (fmodGxyz == 0.0F) {
		f3x3matrixAeqI(fR);
		return;
	}

	// check for vertical up or down gimbal lock case
	if (fmodGyz == 0.0F) {
		f3x3matrixAeqScalar(fR, 0.0F);
		fR[Y][Y] = 1.0F;
		if (fGp[X] >= 0.0F) {
			fR[X][Z] = 1.0F;
			fR[Z][X] = -1.0F;
		} else {
			fR[X][Z] = -1.0F;
			fR[Z][X] = 1.0F;
		}
		return;
	}

	// compute moduli for the general case
	fmodGyz = sqrtf(fmodGyz);
	fmodGxyz = sqrtf(fmodGxyz);
	frecipmodGxyz = 1.0F / fmodGxyz;
	ftmp = fmodGxyz / fmodGyz;

	// normalize the accelerometer reading into the z column
	for (i = X; i <= Z; i++) {
		fR[i][Z] = fGp[i] * frecipmodGxyz;
	}

	// construct x column of orientation matrix
	fR[X][X] = fmodGyz * frecipmodGxyz;
	fR[Y][X] = -fR[X][Z] * fR[Y][Z] * ftmp;
	fR[Z][X] = -fR[X][Z] * fR[Z][Z] * ftmp;

	// // construct y column of orientation matrix
	fR[X][Y] = 0.0F;
	fR[Y][Y] = fR[Z][Z] * ftmp;
	fR[Z][Y] = -fR[Y][Z] * ftmp;
}


// Aerospace NED magnetometer 3DOF flat eCompass function computing rotation matrix fR
void f3DOFMagnetometerMatrixNED(float fR[][3], float fBc[])
{
	// local variables
	float fmodBxy;			// modulus of the x, y magnetometer readings

	// compute the magnitude of the horizontal (x and y) magnetometer reading
	fmodBxy = sqrtf(fBc[X] * fBc[X] + fBc[Y] * fBc[Y]);

	// check for zero field special case where no solution is possible
	if (fmodBxy == 0.0F) {
		f3x3matrixAeqI(fR);
		return;
	}

	// define the fixed entries in the z row and column
	fR[Z][X] = fR[Z][Y] = fR[X][Z] = fR[Y][Z] = 0.0F;
	fR[Z][Z] = 1.0F;

	// define the remaining entries
	fR[X][X] = fR[Y][Y] = fBc[X] / fmodBxy;
	fR[Y][X] = fBc[Y] / fmodBxy;
	fR[X][Y] = -fR[Y][X];
}


// NED: 6DOF e-Compass function computing rotation matrix fR
void feCompassNED(float fR[][3], float *pfDelta, const float fBc[], const float fGp[])
{
	// local variables
	float fmod[3];					// column moduli
	float fmodBc;					// modulus of Bc
	float fGdotBc;					// dot product of vectors G.Bc
	float ftmp;						// scratch variable
	int8_t i, j;						// loop counters

	// set the inclination angle to zero in case it is not computed later
	*pfDelta = 0.0F;

	// place the un-normalized gravity and geomagnetic vectors into the rotation matrix z and x axes
	for (i = X; i <= Z; i++) {
		fR[i][Z] = fGp[i];
		fR[i][X] = fBc[i];
	}

	// set y vector to vector product of z and x vectors
	fR[X][Y] = fR[Y][Z] * fR[Z][X] - fR[Z][Z] * fR[Y][X];
	fR[Y][Y] = fR[Z][Z] * fR[X][X] - fR[X][Z] * fR[Z][X];
	fR[Z][Y] = fR[X][Z] * fR[Y][X] - fR[Y][Z] * fR[X][X];

	// set x vector to vector product of y and z vectors
	fR[X][X] = fR[Y][Y] * fR[Z][Z] - fR[Z][Y] * fR[Y][Z];
	fR[Y][X] = fR[Z][Y] * fR[X][Z] - fR[X][Y] * fR[Z][Z];
	fR[Z][X] = fR[X][Y] * fR[Y][Z] - fR[Y][Y] * fR[X][Z];

	// calculate the rotation matrix column moduli
	fmod[X] = sqrtf(fR[X][X] * fR[X][X] + fR[Y][X] * fR[Y][X] + fR[Z][X] * fR[Z][X]);
	fmod[Y] = sqrtf(fR[X][Y] * fR[X][Y] + fR[Y][Y] * fR[Y][Y] + fR[Z][Y] * fR[Z][Y]);
	fmod[Z] = sqrtf(fR[X][Z] * fR[X][Z] + fR[Y][Z] * fR[Y][Z] + fR[Z][Z] * fR[Z][Z]);

	// normalize the rotation matrix columns
	if (!((fmod[X] == 0.0F) || (fmod[Y] == 0.0F) || (fmod[Z] == 0.0F))) {
		// loop over columns j
		for (j = X; j <= Z; j++) {
			ftmp = 1.0F / fmod[j];
			// loop over rows i
			for (i = X; i <= Z; i++) {
				// normalize by the column modulus
				fR[i][j] *= ftmp;
			}
		}
	} else {
		// no solution is possible to set rotation to identity matrix
		f3x3matrixAeqI(fR);
		return;
	}

	// compute the geomagnetic inclination angle
	fmodBc = sqrtf(fBc[X] * fBc[X] + fBc[Y] * fBc[Y] + fBc[Z] * fBc[Z]);
	fGdotBc = fGp[X] * fBc[X] + fGp[Y] * fBc[Y] + fGp[Z] * fBc[Z];
	if (!((fmod[Z] == 0.0F) || (fmodBc == 0.0F))) {
		*pfDelta = fasin_deg(fGdotBc / (fmod[Z] * fmodBc));
	}
}


// extract the NED angles in degrees from the NED rotation matrix
void fNEDAnglesDegFromRotationMatrix(float R[][3], float *pfPhiDeg,
	float *pfTheDeg, float *pfPsiDeg, float *pfRhoDeg, float *pfChiDeg)
{
	// calculate the pitch angle -90.0 <= Theta <= 90.0 deg
	*pfTheDeg = fasin_deg(-R[X][Z]);

	// calculate the roll angle range -180.0 <= Phi < 180.0 deg
	*pfPhiDeg = fatan2_deg(R[Y][Z], R[Z][Z]);

	// map +180 roll onto the functionally equivalent -180 deg roll
	if (*pfPhiDeg == 180.0F) {
		*pfPhiDeg = -180.0F;
	}

	// calculate the yaw (compass) angle 0.0 <= Psi < 360.0 deg
	if (*pfTheDeg == 90.0F) {
		// vertical upwards gimbal lock case
		*pfPsiDeg = fatan2_deg(R[Z][Y], R[Y][Y]) + *pfPhiDeg;
	} else if (*pfTheDeg == -90.0F) {
		// vertical downwards gimbal lock case
		*pfPsiDeg = fatan2_deg(-R[Z][Y], R[Y][Y]) - *pfPhiDeg;
	} else {
		// general case
		*pfPsiDeg = fatan2_deg(R[X][Y], R[X][X]);
	}

	// map yaw angle Psi onto range 0.0 <= Psi < 360.0 deg
	if (*pfPsiDeg < 0.0F) {
		*pfPsiDeg += 360.0F;
	}

	// check for rounding errors mapping small negative angle to 360 deg
	if (*pfPsiDeg >= 360.0F) {
		*pfPsiDeg = 0.0F;
	}

	// for NED, the compass heading Rho equals the yaw angle Psi
	*pfRhoDeg = *pfPsiDeg;

	// calculate the tilt angle from vertical Chi (0 <= Chi <= 180 deg)
	*pfChiDeg = facos_deg(R[Z][Z]);

	return;
}


// computes normalized rotation quaternion from a rotation vector (deg)
void fQuaternionFromRotationVectorDeg(Quaternion_t *pq, const float rvecdeg[], float fscaling)
{
	float fetadeg;			// rotation angle (deg)
	float fetarad;			// rotation angle (rad)
	float fetarad2;			// eta (rad)^2
	float fetarad4;			// eta (rad)^4
	float sinhalfeta;		// sin(eta/2)
	float fvecsq;			// q1^2+q2^2+q3^2
	float ftmp;				// scratch variable

	// compute the scaled rotation angle eta (deg) which can be both positve or negative
	fetadeg = fscaling * sqrtf(rvecdeg[X] * rvecdeg[X] + rvecdeg[Y] * rvecdeg[Y] + rvecdeg[Z] * rvecdeg[Z]);
	fetarad = fetadeg * FDEGTORAD;
	fetarad2 = fetarad * fetarad;

	// calculate the sine and cosine using small angle approximations or exact
	// angles under sqrt(0.02)=0.141 rad is 8.1 deg and 1620 deg/s (=936deg/s in 3 axes) at 200Hz and 405 deg/s at 50Hz
	if (fetarad2 <= 0.02F) {
		// use MacLaurin series up to and including third order
		sinhalfeta = fetarad * (0.5F - ONEOVER48 * fetarad2);
	} else if  (fetarad2 <= 0.06F) {
		// use MacLaurin series up to and including fifth order
		// angles under sqrt(0.06)=0.245 rad is 14.0 deg and 2807 deg/s (=1623deg/s in 3 axes) at 200Hz and 703 deg/s at 50Hz
		fetarad4 = fetarad2 * fetarad2;
		sinhalfeta = fetarad * (0.5F - ONEOVER48 * fetarad2 + ONEOVER3840 * fetarad4);
	} else {
		// use exact calculation
		sinhalfeta = (float)sinf(0.5F * fetarad);
	}

	// compute the vector quaternion components q1, q2, q3
	if (fetadeg != 0.0F) {
		// general case with non-zero rotation angle
		ftmp = fscaling * sinhalfeta / fetadeg;
		pq->q1 = rvecdeg[X] * ftmp;		// q1 = nx * sin(eta/2)
		pq->q2 = rvecdeg[Y] * ftmp;		// q2 = ny * sin(eta/2)
		pq->q3 = rvecdeg[Z] * ftmp;		// q3 = nz * sin(eta/2)
	} else {
		// zero rotation angle giving zero vector component
		pq->q1 = pq->q2 = pq->q3 = 0.0F;
	}

	// compute the scalar quaternion component q0 by explicit normalization
	// taking care to avoid rounding errors giving negative operand to sqrt
	fvecsq = pq->q1 * pq->q1 + pq->q2 * pq->q2 + pq->q3 * pq->q3;
	if (fvecsq <= 1.0F) {
		// normal case
		pq->q0 = sqrtf(1.0F - fvecsq);
	} else {
		// rounding errors are present
		pq->q0 = 0.0F;
	}
}

// compute the orientation quaternion from a 3x3 rotation matrix
void fQuaternionFromRotationMatrix(float R[][3], Quaternion_t *pq)
{
	float fq0sq;			// q0^2
	float recip4q0;			// 1/4q0

	// the quaternion is not explicitly normalized in this function on the assumption that it
	// is supplied with a normalized rotation matrix. if the rotation matrix is normalized then
	// the quaternion will also be normalized even if the case of small q0

	// get q0^2 and q0
	fq0sq = 0.25F * (1.0F + R[X][X] + R[Y][Y] + R[Z][Z]);
	pq->q0 = sqrtf(fabs(fq0sq));

	// normal case when q0 is not small meaning rotation angle not near 180 deg
	if (pq->q0 > SMALLQ0) {
		// calculate q1 to q3 (general case)
		recip4q0 = 0.25F / pq->q0;
		pq->q1 = recip4q0 * (R[Y][Z] - R[Z][Y]);
		pq->q2 = recip4q0 * (R[Z][X] - R[X][Z]);
		pq->q3 = recip4q0 * (R[X][Y] - R[Y][X]);
	} else {
		// special case of near 180 deg corresponds to nearly symmetric matrix
		// which is not numerically well conditioned for division by small q0
		// instead get absolute values of q1 to q3 from leading diagonal
		pq->q1 = sqrtf(fabs(0.5F * (1.0F + R[X][X]) - fq0sq));
		pq->q2 = sqrtf(fabs(0.5F * (1.0F + R[Y][Y]) - fq0sq));
		pq->q3 = sqrtf(fabs(0.5F * (1.0F + R[Z][Z]) - fq0sq));

		// correct the signs of q1 to q3 by examining the signs of differenced off-diagonal terms
		if ((R[Y][Z] - R[Z][Y]) < 0.0F) pq->q1 = -pq->q1;
		if ((R[Z][X] - R[X][Z]) < 0.0F) pq->q2 = -pq->q2;
		if ((R[X][Y] - R[Y][X]) < 0.0F) pq->q3 = -pq->q3;
	}
}

// compute the rotation matrix from an orientation quaternion
void fRotationMatrixFromQuaternion(float R[][3], const Quaternion_t *pq)
{
	float f2q;
	float f2q0q0, f2q0q1, f2q0q2, f2q0q3;
	float f2q1q1, f2q1q2, f2q1q3;
	float f2q2q2, f2q2q3;
	float f2q3q3;

	// calculate products
	f2q = 2.0F * pq->q0;
	f2q0q0 = f2q * pq->q0;
	f2q0q1 = f2q * pq->q1;
	f2q0q2 = f2q * pq->q2;
	f2q0q3 = f2q * pq->q3;
	f2q = 2.0F * pq->q1;
	f2q1q1 = f2q * pq->q1;
	f2q1q2 = f2q * pq->q2;
	f2q1q3 = f2q * pq->q3;
	f2q = 2.0F * pq->q2;
	f2q2q2 = f2q * pq->q2;
	f2q2q3 = f2q * pq->q3;
	f2q3q3 = 2.0F * pq->q3 * pq->q3;

	// calculate the rotation matrix assuming the quaternion is normalized
	R[X][X] = f2q0q0 + f2q1q1 - 1.0F;
	R[X][Y] = f2q1q2 + f2q0q3;
	R[X][Z] = f2q1q3 - f2q0q2;
	R[Y][X] = f2q1q2 - f2q0q3;
	R[Y][Y] = f2q0q0 + f2q2q2 - 1.0F;
	R[Y][Z] = f2q2q3 + f2q0q1;
	R[Z][X] = f2q1q3 + f2q0q2;
	R[Z][Y] = f2q2q3 - f2q0q1;
	R[Z][Z] = f2q0q0 + f2q3q3 - 1.0F;
}

// function calculate the rotation vector from a rotation matrix
void fRotationVectorDegFromRotationMatrix(float R[][3], float rvecdeg[])
{
	float ftrace;			// trace of the rotation matrix
	float fetadeg;			// rotation angle eta (deg)
	float fmodulus;			// modulus of axis * angle vector = 2|sin(eta)|
	float ftmp;				// scratch variable

	// calculate the trace of the rotation matrix = 1+2cos(eta) in range -1 to +3
	// and eta (deg) in range 0 to 180 deg inclusive
	// checking for rounding errors that might take the trace outside this range
	ftrace = R[X][X] + R[Y][Y] + R[Z][Z];
	if (ftrace >= 3.0F) {
		fetadeg = 0.0F;
	} else if (ftrace <= -1.0F) {
		fetadeg = 180.0F;
	} else {
		fetadeg = acosf(0.5F * (ftrace - 1.0F)) * FRADTODEG;
	}

	// set the rvecdeg vector to differences across the diagonal = 2*n*sin(eta)
	// and calculate its modulus equal to 2|sin(eta)|
	// the modulus approaches zero near 0 and 180 deg (when sin(eta) approaches zero)
	rvecdeg[X] = R[Y][Z] - R[Z][Y];
	rvecdeg[Y] = R[Z][X] - R[X][Z];
	rvecdeg[Z] = R[X][Y] - R[Y][X];
	fmodulus = sqrtf(rvecdeg[X] * rvecdeg[X] + rvecdeg[Y] * rvecdeg[Y] + rvecdeg[Z] * rvecdeg[Z]);

	// normalize the rotation vector for general, 0 deg and 180 deg rotation cases
	if (fmodulus > SMALLMODULUS) {
		// general case away from 0 and 180 deg rotation
		ftmp = fetadeg / fmodulus;
		rvecdeg[X] *= ftmp;	// set x component to eta(deg) * nx
		rvecdeg[Y] *= ftmp;	// set y component to eta(deg) * ny
		rvecdeg[Z] *= ftmp;	// set z component to eta(deg) * nz
	} else if (ftrace >= 0.0F) {
		// near 0 deg rotation (trace = 3): matrix is nearly identity matrix
		// R[Y][Z]-R[Z][Y]=2*nx*eta(rad) and similarly for other components
		ftmp = 0.5F * FRADTODEG;
		rvecdeg[X] *= ftmp;
		rvecdeg[Y] *= ftmp;
		rvecdeg[Z] *= ftmp;
	} else {
		// near 180 deg (trace = -1): matrix is nearly symmetric
		// calculate the absolute value of the components of the axis-angle vector
		rvecdeg[X] = 180.0F * sqrtf(fabs(0.5F * (R[X][X] + 1.0F)));
		rvecdeg[Y] = 180.0F * sqrtf(fabs(0.5F * (R[Y][Y] + 1.0F)));
		rvecdeg[Z] = 180.0F * sqrtf(fabs(0.5F * (R[Z][Z] + 1.0F)));

		// correct the signs of the three components by examining the signs of differenced off-diagonal terms
		if ((R[Y][Z] - R[Z][Y]) < 0.0F) rvecdeg[X] = -rvecdeg[X];
		if ((R[Z][X] - R[X][Z]) < 0.0F) rvecdeg[Y] = -rvecdeg[Y];
		if ((R[X][Y] - R[Y][X]) < 0.0F) rvecdeg[Z] = -rvecdeg[Z];

	}
}

// computes rotation vector (deg) from rotation quaternion
void fRotationVectorDegFromQuaternion(Quaternion_t *pq, float rvecdeg[])
{
	float fetarad;			// rotation angle (rad)
	float fetadeg;			// rotation angle (deg)
	float sinhalfeta;		// sin(eta/2)
	float ftmp;				// scratch variable

	// calculate the rotation angle in the range 0 <= eta < 360 deg
	if ((pq->q0 >= 1.0F) || (pq->q0 <= -1.0F)) {
		// rotation angle is 0 deg or 2*180 deg = 360 deg = 0 deg
		fetarad = 0.0F;
		fetadeg = 0.0F;
	} else {
		// general case returning 0 < eta < 360 deg
		fetarad = 2.0F * acosf(pq->q0);
		fetadeg = fetarad * FRADTODEG;
	}

	// map the rotation angle onto the range -180 deg <= eta < 180 deg
	if (fetadeg >= 180.0F) {
		fetadeg -= 360.0F;
		fetarad = fetadeg * FDEGTORAD;
	}

	// calculate sin(eta/2) which will be in the range -1 to +1
	sinhalfeta = (float)sinf(0.5F * fetarad);

	// calculate the rotation vector (deg)
	if (sinhalfeta == 0.0F) {
		// the rotation angle eta is zero and the axis is irrelevant
		rvecdeg[X] = rvecdeg[Y] = rvecdeg[Z] = 0.0F;
	} else {
		// general case with non-zero rotation angle
		ftmp = fetadeg / sinhalfeta;
		rvecdeg[X] = pq->q1 * ftmp;
		rvecdeg[Y] = pq->q2 * ftmp;
		rvecdeg[Z] = pq->q3 * ftmp;
	}
}

// function compute the quaternion product qA * qB
void qAeqBxC(Quaternion_t *pqA, const Quaternion_t *pqB, const Quaternion_t *pqC)
{
	pqA->q0 = pqB->q0 * pqC->q0 - pqB->q1 * pqC->q1 - pqB->q2 * pqC->q2 - pqB->q3 * pqC->q3;
	pqA->q1 = pqB->q0 * pqC->q1 + pqB->q1 * pqC->q0 + pqB->q2 * pqC->q3 - pqB->q3 * pqC->q2;
	pqA->q2 = pqB->q0 * pqC->q2 - pqB->q1 * pqC->q3 + pqB->q2 * pqC->q0 + pqB->q3 * pqC->q1;
	pqA->q3 = pqB->q0 * pqC->q3 + pqB->q1 * pqC->q2 - pqB->q2 * pqC->q1 + pqB->q3 * pqC->q0;
}

// function compute the quaternion product qA = qA * qB
void qAeqAxB(Quaternion_t *pqA, const Quaternion_t *pqB)
{
	Quaternion_t qProd;

	// perform the quaternion product
	qProd.q0 = pqA->q0 * pqB->q0 - pqA->q1 * pqB->q1 - pqA->q2 * pqB->q2 - pqA->q3 * pqB->q3;
	qProd.q1 = pqA->q0 * pqB->q1 + pqA->q1 * pqB->q0 + pqA->q2 * pqB->q3 - pqA->q3 * pqB->q2;
	qProd.q2 = pqA->q0 * pqB->q2 - pqA->q1 * pqB->q3 + pqA->q2 * pqB->q0 + pqA->q3 * pqB->q1;
	qProd.q3 = pqA->q0 * pqB->q3 + pqA->q1 * pqB->q2 - pqA->q2 * pqB->q1 + pqA->q3 * pqB->q0;

	// copy the result back into qA
	*pqA = qProd;
}

// function normalizes a rotation quaternion and ensures q0 is non-negative
void fqAeqNormqA(Quaternion_t *pqA)
{
	float fNorm;					// quaternion Norm

	// calculate the quaternion Norm
	fNorm = sqrtf(pqA->q0 * pqA->q0 + pqA->q1 * pqA->q1 + pqA->q2 * pqA->q2 + pqA->q3 * pqA->q3);
	if (fNorm > CORRUPTQUAT) {
		// general case
		fNorm = 1.0F / fNorm;
		pqA->q0 *= fNorm;
		pqA->q1 *= fNorm;
		pqA->q2 *= fNorm;
		pqA->q3 *= fNorm;
	} else {
		// return with identity quaternion since the quaternion is corrupted
		pqA->q0 = 1.0F;
		pqA->q1 = pqA->q2 = pqA->q3 = 0.0F;
	}

	// correct a negative scalar component if the function was called with negative q0
	if (pqA->q0 < 0.0F) {
		pqA->q0 = -pqA->q0;
		pqA->q1 = -pqA->q1;
		pqA->q2 = -pqA->q2;
		pqA->q3 = -pqA->q3;
	}
}

// set a quaternion to the unit quaternion
void fqAeq1(Quaternion_t *pqA)
{
	pqA->q0 = 1.0F;
	pqA->q1 = pqA->q2 = pqA->q3 = 0.0F;
}



// function returns an approximation to angle(deg)=asin(x) for x in the range -1 <= x <= 1
// and returns -90 <= angle <= 90 deg
// maximum error is 10.29E-6 deg
float fasin_deg(float x)
{
	// for robustness, check for invalid argument
	if (x >= 1.0F) return 90.0F;
	if (x <= -1.0F) return -90.0F;

	// call the atan which will return an angle in the correct range -90 to 90 deg
	// this line cannot fail from division by zero or negative square root since |x| < 1
	return (fatan_deg(x / sqrtf(1.0F - x * x)));
}

// function returns an approximation to angle(deg)=acos(x) for x in the range -1 <= x <= 1
// and returns 0 <= angle <= 180 deg
// maximum error is 14.67E-6 deg
float facos_deg(float x)
{
	// for robustness, check for invalid arguments
	if (x >= 1.0F) return 0.0F;
	if (x <= -1.0F) return 180.0F;

	// call the atan which will return an angle in the incorrect range -90 to 90 deg
	// these lines cannot fail from division by zero or negative square root
	if (x == 0.0F) return 90.0F;
	if (x > 0.0F) return fatan_deg((sqrtf(1.0F - x * x) / x));
	return 180.0F + fatan_deg((sqrtf(1.0F - x * x) / x));
}

// function returns angle in range -90 to 90 deg
// maximum error is 9.84E-6 deg
float fatan_deg(float x)
{
	float fangledeg;			// compute computed (deg)
	int8_t ixisnegative;		// argument x is negative
	int8_t ixexceeds1;			// argument x is greater than 1.0
	int8_t ixmapped;			// argument in range tan(15 deg) to tan(45 deg)=1.0

constexpr float TAN15DEG = 0.26794919243F;		// tan(15 deg) = 2 - sqrt(3)
constexpr float TAN30DEG = 0.57735026919F;		// tan(30 deg) = 1/sqrt(3)

	// reset all flags
	ixisnegative = ixexceeds1 = ixmapped = 0;

	// test for negative argument to allow use of tan(-x)=-tan(x)
	if (x < 0.0F) {
		x = -x;
		ixisnegative = 1;
	}

	// test for argument above 1 to allow use of atan(x)=pi/2-atan(1/x)
	if (x > 1.0F) {
		x = 1.0F / x;
		ixexceeds1 = 1;
	}

	// at this point, x is in the range 0 to 1 inclusive
	// map argument onto range -tan(15 deg) to tan(15 deg)
	// using tan(angle-30deg) = (tan(angle)-tan(30deg)) / (1 + tan(angle)tan(30deg))
	// tan(15deg) maps to tan(-15 deg) = -tan(15 deg)
	// 1. maps to (sqrt(3) - 1) / (sqrt(3) + 1) = 2 - sqrt(3) = tan(15 deg)
	if (x > TAN15DEG) {
		x = (x - TAN30DEG)/(1.0F + TAN30DEG * x);
		ixmapped = 1;
	}

	// call the atan estimator to obtain -15 deg <= angle <= 15 deg
	fangledeg = fatan_15deg(x);

	// undo the distortions applied earlier to obtain -90 deg <= angle <= 90 deg
	if (ixmapped) fangledeg += 30.0F;
	if (ixexceeds1) fangledeg = 90.0F - fangledeg;
	if (ixisnegative) fangledeg = -fangledeg;

	return (fangledeg);
}

// function returns approximate atan2 angle in range -180 to 180 deg
// maximum error is 14.58E-6 deg
float fatan2_deg(float y, float x)
{
	// check for zero x to avoid division by zero
	if (x == 0.0F) {
		// return 90 deg for positive y
		if (y > 0.0F) return 90.0F;
		// return -90 deg for negative y
		if (y < 0.0F) return -90.0F;
		// otherwise y= 0.0 and return 0 deg (invalid arguments)
		return 0.0F;
	}

	// from here onwards, x is guaranteed to be non-zero
	// compute atan2 for quadrant 1 (0 to 90 deg) and quadrant 4 (-90 to 0 deg)
	if (x > 0.0F) return (fatan_deg(y / x));
	// compute atan2 for quadrant 2 (90 to 180 deg)
	if ((x < 0.0F) && (y > 0.0F)) return (180.0F + fatan_deg(y / x));
	// compute atan2 for quadrant 3 (-180 to -90 deg)
	return (-180.0F + fatan_deg(y / x));
}

// approximation to inverse tan function (deg) for x in range
// -tan(15 deg) to tan(15 deg) giving an output -15 deg <= angle <= 15 deg
// using modified Pade[3/2] approximation
float fatan_15deg(float x)
{
	float x2;			// x^2

constexpr float PADE_A = 96.644395816F;	// theoretical Pade[3/2] value is 5/3*180/PI=95.49296
constexpr float PADE_B = 25.086941612F;	// theoretical Pade[3/2] value is 4/9*180/PI=25.46479
constexpr float PADE_C = 1.6867633134F;	// theoretical Pade[3/2] value is 5/3=1.66667

	// compute the approximation to the inverse tangent
	// the function is anti-symmetric as required for positive and negative arguments
	x2 = x * x;
	return (x * (PADE_A + x2 * PADE_B) / (PADE_C + x2));
}

} // namespace libcalib
