#include "libcalib/fitter.h"

#include "matrix.h"
#include "sphere.h"

#include <float.h>
#include <math.h>
#include <string.h>

namespace libcalib
{

namespace Sphere
{

using namespace Quality;

// pigeonhole invariant: when the buffer is full, at least one region has >1 sample
static_assert(CFitter::s_cSampMax > REGION_Max);

constexpr int X = 0;							// vector components
constexpr int Y = 1;
constexpr int Z = 2;
constexpr float s_sOneThird = 0.33333333F;		// one third
constexpr float s_sOneSixth = 0.166666667F;		// one sixth
constexpr int s_cSampMin4INV = 40;				// minimum number of measurements for 4 element calibration
constexpr int s_cSampMin7EIG = 100;				// minimum number of measurements for 7 element calibration
constexpr int s_cSampMin10EIG = 150;			// minimum number of measurements for 10 element calibration
constexpr float s_sBFitMin = 22.0F;				// minimum geomagnetic field B (uT) for valid calibration
constexpr float s_sBFitMax = 67.0F;				// maximum geomagnetic field B (uT) for valid calibration

// CSampleSet

CFitter::CSampleSet::CSampleSet()
: m_aSamp()
, m_cSamp(0)
, m_mpRegionCSamp()
, m_idNext(0)
{
}

void CFitter::CSampleSet::AddSample(CFitter * pFitter, const SSample & samp)
{
	int iSampDest = -1;

	if (m_cSamp < s_cSampMax)
	{
		iSampDest = m_cSamp;
	}

	// when the buffer is full, first check for field-strength outliers to replace

	if (iSampDest < 0)
	{
		iSampDest = pFitter->ISampFieldOutlier(samp.m_region);
	}

	// if still full, pick eldest from our most populated region.

	if (iSampDest < 0)
	{
		// pigeonhole: at least one region has >1 sample
		REGION regionEvict = RegionMostPopulated();
		iSampDest = ISampOldestInRegion(regionEvict);
	}

	// still full? hard to imagine but ok.

	if (iSampDest < 0)
	{
		iSampDest = 0;
	}

	if (iSampDest < m_cSamp)
	{
		--m_mpRegionCSamp[m_aSamp[iSampDest].m_region];
	}
	else
	{
		++m_cSamp;
	}

	m_aSamp[iSampDest] = samp;
	m_aSamp[iSampDest].m_id = m_idNext++;
	
	++m_mpRegionCSamp[samp.m_region];
}

REGION CFitter::CSampleSet::RegionMostPopulated() const
{
	REGION regionBest = REGION_Nil;
	int cSampBest = 1; // must be strictly greater than 1

	for (int iRegion = REGION_Min; iRegion < REGION_Max; iRegion++)
	{
		REGION region = REGION(iRegion);

		if (m_mpRegionCSamp[region] > cSampBest)
		{
			cSampBest = m_mpRegionCSamp[region];
			regionBest = region;
		}
	}

	return regionBest;
}

int CFitter::CSampleSet::ISampOldestInRegion(REGION region) const
{
	int iSampOldest = -1;
	SSample::ID idOldest = ~SSample::ID(0); // max value

	for (int i = 0; i < m_cSamp; i++)
	{
		if (m_aSamp[i].m_region == region && m_aSamp[i].m_id < idOldest)
		{
			idOldest = m_aSamp[i].m_id;
			iSampOldest = i;
		}
	}

	return iSampOldest;
}


CFitter::CFitter()
: m_cal()
, m_solver(SOLVER_Nil)
, m_samps()
, m_calNext()
, m_errFitNext(s_errMax)
, m_errFitNextAged(s_errMax)
, m_A()
, m_invA()
, m_matA()
, m_matB()
, m_vecA()
, m_vecB()
, m_discard_count(0)
, m_new_wait_count(0)
, m_errGaps(s_errMax)
, m_errVariance(s_errMax)
, m_errWobble(s_errMax)
, m_errFit(s_errMax)
{
}

int CFitter::ISampFieldOutlier(REGION regionIncoming)
{
	// When enough data is collected (gaps error is low), assume we
	// have a pretty good coverage and the field stregth is known.
	float errGaps = m_errGaps;
	if (errGaps < 25.0f && FHasSolution())
	{
		// occasionally look for points farthest from average field strength
		// always rate limit assumption-based data purging, but allow the
		// rate to increase as the angular coverage improves.
		if (errGaps < 1.0f)
		{
			errGaps = 1.0f;
		}

		if (++m_discard_count > int(errGaps * 10.0f))
		{
			int iSampOutlier = -1;
			float dUtBOutlier = 0.0f;
			REGION regionOutlier = REGION_Nil;
			
			int iSamp = 0;
			for (const auto & samp : m_samps)
			{
				float dB = fabsf(samp.m_sB - m_cal.m_sB);
				
				if (dB > dUtBOutlier)
				{
					dUtBOutlier = dB;
					iSampOutlier = iSamp;
					regionOutlier = samp.m_region;
				}

				++iSamp;
			}

			if (iSampOutlier >= 0)
			{
				// only evict this outlier if its region is at least as populated
				// as the region we're about to add to — evictions must not make
				// coverage less uniform than additions
				if (m_samps.CSampFromRegion(regionOutlier) >=
					m_samps.CSampFromRegion(regionIncoming))
				{
					m_discard_count = 0;
					return iSampOutlier;
				}
				// candidate would depopulate a relatively sparse region —
				// fall through to region-based eviction below
			}
		}
	}
	else
	{
		m_discard_count = 0;
	}

	return -1;
}


void CFitter::AddSample(const SPoint & pntRaw, SPoint * pPntCal)
{
	SSample samp(pntRaw, m_cal);

	if (pPntCal)
	{
		*pPntCal = samp.m_pntCal;
	}

	m_samps.AddSample(this, samp);

	// NOTE: we do not update our quality metrics on every new sample.
	//	instead, we update them after every new calibration is accepted
	//	or when specifically asked to.

	ResetQuality();
}

// run the magnetic calibration


bool CFitter::FHasNewCalibration(float * pSMagChange)
{
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
	// This file contains magnetic calibration functions.  It is STRONGLY RECOMMENDED
	// that the casual developer NOT TOUCH THIS FILE.  The mathematics behind this file
	// is extremely complex, and it will be very easy (almost inevitable) that you screw
	// it up.
	//
	// Haha - This file has been edited!  Please do not blame or pester NXP (formerly
	//        Freescale) about the "almost inevitable" issues!

	SOLVER solver = SOLVER_Nil;		// magnetic solver used

	// only do the calibration occasionally

	if (++m_new_wait_count < s_new_wait_count_max)
		return false;

	m_new_wait_count = 0;

	if (m_samps.m_cSamp < s_cSampMin4INV)
		return false;

	if (m_solver != SOLVER_Nil) {
		// age the existing fit error to avoid one good calibration locking out future updates
		m_errFitNextAged *= 1.02f;
	}

	// is enough data collected
	if (m_samps.m_cSamp < s_cSampMin7EIG)
	{
		solver = SOLVER_4Inv;
		UpdateCalibration4INV(); // 4 element matrix inversion calibration
		m_errFitNext = fmax(m_errFitNext, 12.0f);
	}
	else if (m_samps.m_cSamp < s_cSampMin10EIG)
	{
		solver = SOLVER_7Eig;
		UpdateCalibration7EIG(); // 7 element eigenpair calibration
		m_errFitNext = fmax(m_errFitNext, 7.5f);
	}
	else
	{
		solver = SOLVER_10Eig;
		UpdateCalibration10EIG(); // 10 element eigenpair calibration
	}

	// the trial geomagnetic field must be in range (earth is 22uT to 67uT)

	if ((m_calNext.m_sB >= s_sBFitMin) && (m_calNext.m_sB <= s_sBFitMax))
	{
		// always accept the calibration under any of these conditions:
		//  1: no previous calibration exists
		bool fNoSolver = (m_solver == SOLVER_Nil);
		//  2: the calibration fit is reduced
		bool fReducedFit = (m_errFitNext <= m_errFitNextAged);
		//  3: an improved solver was used giving a good trial calibration (4% or under)
		bool fBetterSolverAndFit = ((solver > m_solver) && (m_errFitNext <= 4.0F));
		
		if (fNoSolver || fReducedFit || fBetterSolverAndFit)
		{
			// accept the new calibration solution
			//printf("new magnetic cal, B=%.2f uT\n", m_calNext.m_sB);
			m_solver = solver;
			m_errFit = m_errFitNext;
			m_errFitNextAged = fmax(m_errFitNext, 2.0f);
			
			float dVecV[3];
			for (int i = X; i <= Z; i++)
			{
				dVecV[i] = m_calNext.m_vecV[i] - m_cal.m_vecV[i];
			}
			
			m_cal = m_calNext;

			if (pSMagChange)
			{
				const float & x = dVecV[0];
				const float & y = dVecV[1];
				const float & z = dVecV[2];

				*pSMagChange = sqrtf(x * x + y * y + z * z);
			}

			// re-apply calibration to all our samples and update our quality metrics

			m_samps.Recalibrate(m_cal);

			ForceUpdateQuality();

			return true; // indicates new calibration applied
		}
	}
	return false;
}



// 4 element calibration using 4x4 matrix inverse
void CFitter::UpdateCalibration4INV()
{
	float fBp2;					// fBp[X]^2+fBp[Y]^2+fBp[Z]^2
	float fSumBp4;				// sum of fBp2
	float fscaling;				// set to FUTPERCOUNT * FMATRIXSCALING
	float fE;					// error function = r^T.r
	int i, j, k;				// loop counters

	// working arrays for 4x4 matrix inversion
	float *pfRows[4];
	int8_t iColInd[4];
	int8_t iRowInd[4];
	int8_t iPivot[4];

	// compute fscaling to reduce multiplications later
	fscaling = 1.0F / Mag::s_sBDefault;

	// the trial inverse soft iron matrix m_cal.m_matWInv always equals
	// the identity matrix for 4 element calibration
	m_calNext.m_matWInv = SMatrix3();

	// zero fSumBp4=Y^T.Y, m_vecB=X^T.Y (4x1) and on and above
	// diagonal elements of m_matA=X^T*X (4x4)
	fSumBp4 = 0.0F;
	for (i = 0; i < 4; i++) {
		m_vecB[i] = 0.0F;
		for (j = i; j < 4; j++) {
			m_matA[i][j] = 0.0F;
		}
	}

	// the offsets are guaranteed to be set from the first element but to avoid compiler error
	const SPoint & BpOffset = m_samps.m_aSamp[0].m_pntRaw;

	// use from MINEQUATIONS up to MAXEQUATIONS entries from magnetic buffer to compute matrices
	for (const auto & samp : m_samps)
	{
		const SPoint & BpCur = samp.m_pntRaw;

		// store scaled and offset fBp[XYZ] in m_vecA[0-2] and fBp[XYZ]^2 in m_vecA[3-5]
		for (k = X; k <= Z; k++) {
			m_vecA[k] = (BpCur[k] - BpOffset[k]) * fscaling;
			m_vecA[k + 3] = m_vecA[k] * m_vecA[k];
		}

		// calculate fBp2 = Bp[X]^2 + Bp[Y]^2 + Bp[Z]^2 (scaled uT^2)
		fBp2 = m_vecA[3] + m_vecA[4] + m_vecA[5];

		// accumulate fBp^4 over all measurements into fSumBp4=Y^T.Y
		fSumBp4 += fBp2 * fBp2;

		// now we have fBp2, accumulate m_vecB[0-2] = X^T.Y =sum(Bp2.Bp[XYZ])
		for (k = X; k <= Z; k++) {
			m_vecB[k] += m_vecA[k] * fBp2;
		}

		//accumulate m_vecB[3] = X^T.Y =sum(fBp2)
		m_vecB[3] += fBp2;

		// accumulate on and above-diagonal terms of m_matA = X^T.X ignoring m_matA[3][3]
		m_matA[0][0] += m_vecA[X + 3];
		m_matA[0][1] += m_vecA[X] * m_vecA[Y];
		m_matA[0][2] += m_vecA[X] * m_vecA[Z];
		m_matA[0][3] += m_vecA[X];
		m_matA[1][1] += m_vecA[Y + 3];
		m_matA[1][2] += m_vecA[Y] * m_vecA[Z];
		m_matA[1][3] += m_vecA[Y];
		m_matA[2][2] += m_vecA[Z + 3];
		m_matA[2][3] += m_vecA[Z];
	}

	// set the last element of the measurement matrix to the number of buffer elements used
	m_matA[3][3] = float(m_samps.m_cSamp);

	// use above diagonal elements of symmetric m_matA to set both m_matB and m_matA to X^T.X
	for (i = 0; i < 4; i++) {
		for (j = i; j < 4; j++) {
			m_matB[i][j] = m_matB[j][i]
				= m_matA[j][i] = m_matA[i][j];
		}
	}

	// calculate in situ inverse of m_matB = inv(X^T.X) (4x4) while m_matA still holds X^T.X
	for (i = 0; i < 4; i++) {
		pfRows[i] = m_matB[i];
	}
	fmatrixAeqInvA(pfRows, iColInd, iRowInd, iPivot, 4);

	// calculate m_vecA = solution beta (4x1) = inv(X^T.X).X^T.Y = m_matB * m_vecB
	for (i = 0; i < 4; i++) {
		m_vecA[i] = 0.0F;
		for (k = 0; k < 4; k++) {
			m_vecA[i] += m_matB[i][k] * m_vecB[k];
		}
	}

	// calculate P = r^T.r = Y^T.Y - 2 * beta^T.(X^T.Y) + beta^T.(X^T.X).beta
	// = fSumBp4 - 2 * m_vecA^T.m_vecB + m_vecA^T.m_matA.m_vecA
	// first set P = Y^T.Y - 2 * beta^T.(X^T.Y) = SumBp4 - 2 * m_vecA^T.m_vecB
	fE = 0.0F;
	for (i = 0; i < 4; i++) {
		fE += m_vecA[i] * m_vecB[i];
	}
	fE = fSumBp4 - 2.0F * fE;

	// set m_vecB = (X^T.X).beta = m_matA.m_vecA
	for (i = 0; i < 4; i++) {
		m_vecB[i] = 0.0F;
		for (k = 0; k < 4; k++) {
			m_vecB[i] += m_matA[i][k] * m_vecA[k];
		}
	}

	// complete calculation of P by adding beta^T.(X^T.X).beta = m_vecA^T * m_vecB
	for (i = 0; i < 4; i++) {
		fE += m_vecB[i] * m_vecA[i];
	}

	// compute the hard iron vector (in uT but offset and scaled by FMATRIXSCALING)
	for (k = X; k <= Z; k++) {
		m_calNext.m_vecV[k] = 0.5F * m_vecA[k];
	}

	// compute the scaled geomagnetic field strength B (in uT but scaled by FMATRIXSCALING)
	m_calNext.m_sB = sqrtf(m_vecA[3] + m_calNext.m_vecV[X] * m_calNext.m_vecV[X] +
			m_calNext.m_vecV[Y] * m_calNext.m_vecV[Y] + m_calNext.m_vecV[Z] * m_calNext.m_vecV[Z]);

	// calculate the trial fit error (percent) normalized to number of measurements
	// and scaled geomagnetic field strength
	m_errFitNext = sqrtf(fE / float(m_samps.m_cSamp)) * 100.0F /
			(2.0F * m_calNext.m_sB * m_calNext.m_sB);

	// correct the hard iron estimate for FMATRIXSCALING and the offsets applied (result in uT)
	for (k = X; k <= Z; k++) {
		m_calNext.m_vecV[k] = m_calNext.m_vecV[k] * Mag::s_sBDefault + BpOffset[k];
	}

	// correct the geomagnetic field strength B to correct scaling (result in uT)
	m_calNext.m_sB *= Mag::s_sBDefault;
}




// 7 element calibration using direct eigen-decomposition
void CFitter::UpdateCalibration7EIG()
{
	float det;					// matrix determinant
	float fscaling;				// set to FUTPERCOUNT * FMATRIXSCALING
	float ftmp;					// scratch variable
	int i, j, k, m, n;			// loop counters

	// compute fscaling to reduce multiplications later
	fscaling = 1.0F / Mag::s_sBDefault;

	// the offsets are guaranteed to be set from the first element but to avoid compiler error
	SPoint BpOffset = m_samps.m_aSamp[0].m_pntRaw;

	// zero the on and above diagonal elements of the 7x7 symmetric measurement matrix m_matA
	for (m = 0; m < 7; m++) {
		for (n = m; n < 7; n++) {
			m_matA[m][n] = 0.0F;
		}
	}

	// place from MINEQUATIONS to MAXEQUATIONS entries into product matrix m_matA
	for (const auto & samp: m_samps)
	{
		const SPoint& BpCur = samp.m_pntRaw;

		// apply the offset and scaling and store in m_vecA
		for (k = X; k <= Z; k++) {
			m_vecA[k + 3] = (BpCur[k] - BpOffset[k]) * fscaling;
			m_vecA[k] = m_vecA[k + 3] * m_vecA[k + 3];
		}

		// accumulate the on-and above-diagonal terms of
		// m_matA=Sigma{m_vecA^T * m_vecA}
		// with the exception of m_matA[6][6] which will sum to the number
		// of measurements and remembering that m_vecA[6] equals 1.0F
		// update the right hand column [6] of m_matA except for m_matA[6][6]
		for (m = 0; m < 6; m++) {
			m_matA[m][6] += m_vecA[m];
		}
		// update the on and above diagonal terms except for right hand column 6
		for (m = 0; m < 6; m++) {
			for (n = m; n < 6; n++) {
				m_matA[m][n] += m_vecA[m] * m_vecA[n];
			}
		}
	}

	// finally set the last element m_matA[6][6] to the number of measurements
	m_matA[6][6] = float(m_samps.m_cSamp);

	// copy the above diagonal elements of m_matA to below the diagonal
	for (m = 1; m < 7; m++) {
		for (n = 0; n < m; n++) {
			m_matA[m][n] = m_matA[n][m];
		}
	}

	// set tmpA7x1 to the unsorted eigenvalues and m_matB to the unsorted eigenvectors of m_matA
	eigencompute(m_matA, m_vecA, m_matB, 7);

	// find the smallest eigenvalue
	j = 0;
	for (i = 1; i < 7; i++) {
		if (m_vecA[i] < m_vecA[j]) {
			j = i;
		}
	}

	// set ellipsoid matrix A to the solution vector with smallest eigenvalue,
	// compute its determinant and the hard iron offset (scaled and offset)
	f3x3matrixAeqScalar(m_A, 0.0F);
	det = 1.0F;
	for (k = X; k <= Z; k++) {
		m_A[k][k] = m_matB[k][j];
		det *= m_A[k][k];
		m_calNext.m_vecV[k] = -0.5F * m_matB[k + 3][j] / m_A[k][k];
	}

	// negate A if it has negative determinant
	if (det < 0.0F) {
		f3x3matrixAeqMinusA(m_A);
		m_matB[6][j] = -m_matB[6][j];
		det = -det;
	}

	// set ftmp to the square of the trial geomagnetic field strength B
	// (counts times FMATRIXSCALING)
	ftmp = -m_matB[6][j];
	for (k = X; k <= Z; k++) {
		ftmp += m_A[k][k] * m_calNext.m_vecV[k] * m_calNext.m_vecV[k];
	}

	// calculate the trial normalized fit error as a percentage
	m_errFitNext = 50.0F *
		sqrtf(fabs(m_vecA[j]) / float(m_samps.m_cSamp)) / fabs(ftmp);

	// normalize the ellipsoid matrix A to unit determinant
	f3x3matrixAeqAxScalar(m_A, powf(det, -(s_sOneThird)));

	// convert the geomagnetic field strength B into uT for normalized
	// soft iron matrix A and normalize
	m_calNext.m_sB = sqrtf(fabs(ftmp)) * Mag::s_sBDefault * powf(det, -(s_sOneSixth));

	// compute trial m_cal.m_matWInv from the square root of A also with normalized
	// determinant and hard iron offset in uT
	m_calNext.m_matWInv = SMatrix3();

	for (k = X; k <= Z; k++) {
		m_calNext.m_matWInv[k][k] = sqrtf(fabs(m_A[k][k]));
		m_calNext.m_vecV[k] = m_calNext.m_vecV[k] * Mag::s_sBDefault + BpOffset[k];
	}
}




// 10 element calibration using direct eigen-decomposition
void CFitter::UpdateCalibration10EIG()
{
	float det;					// matrix determinant
	float fscaling;				// set to FUTPERCOUNT * FMATRIXSCALING
	float ftmp;					// scratch variable
	int i, j, k, m, n;			// loop counters

	// compute fscaling to reduce multiplications later
	fscaling = 1.0F / Mag::s_sBDefault;

	// the offsets are guaranteed to be set from the first element but to avoid compiler error
	SPoint BpOffset = m_samps.m_aSamp[0].m_pntRaw;

	// zero the on and above diagonal elements of the 10x10 symmetric measurement matrix m_matA
	for (m = 0; m < 10; m++) {
		for (n = m; n < 10; n++) {
			m_matA[m][n] = 0.0F;
		}
	}

	// sum between MINEQUATIONS to MAXEQUATIONS entries into the 10x10 product matrix m_matA
	for (const auto & samp : m_samps) {
		const SPoint& BpCur = samp.m_pntRaw;

		// apply the fixed offset and scaling and enter into m_vecA[6-8]
		for (k = X; k <= Z; k++) {
			m_vecA[k + 6] = (BpCur[k] - BpOffset[k]) * fscaling;
		}

		// compute measurement vector elements m_vecA[0-5] from m_vecA[6-8]
		m_vecA[0] = m_vecA[6] * m_vecA[6];
		m_vecA[1] = 2.0F * m_vecA[6] * m_vecA[7];
		m_vecA[2] = 2.0F * m_vecA[6] * m_vecA[8];
		m_vecA[3] = m_vecA[7] * m_vecA[7];
		m_vecA[4] = 2.0F * m_vecA[7] * m_vecA[8];
		m_vecA[5] = m_vecA[8] * m_vecA[8];

		// accumulate the on-and above-diagonal terms of m_matA=Sigma{m_vecA^T * m_vecA}
		// with the exception of m_matA[9][9] which equals the number of measurements
		// update the right hand column [9] of m_matA[0-8][9] ignoring m_matA[9][9]
		for (m = 0; m < 9; m++) {
			m_matA[m][9] += m_vecA[m];
		}
		// update the on and above diagonal terms of m_matA ignoring right hand column 9
		for (m = 0; m < 9; m++) {
			for (n = m; n < 9; n++) {
				m_matA[m][n] += m_vecA[m] * m_vecA[n];
			}
		}
	}

	// set the last element m_matA[9][9] to the number of measurements
	m_matA[9][9] = float(m_samps.m_cSamp);

	// copy the above diagonal elements of symmetric product matrix m_matA to below the diagonal
	for (m = 1; m < 10; m++) {
		for (n = 0; n < m; n++) {
			m_matA[m][n] = m_matA[n][m];
		}
	}

	// set m_vecA to the unsorted eigenvalues and m_matB to the unsorted
	// normalized eigenvectors of m_matA
	eigencompute(m_matA, m_vecA, m_matB, 10);

	// set ellipsoid matrix A from elements of the solution vector column j with
	// smallest eigenvalue
	j = 0;
	for (i = 1; i < 10; i++) {
		if (m_vecA[i] < m_vecA[j]) {
			j = i;
		}
	}
	m_A[0][0] = m_matB[0][j];
	m_A[0][1] = m_A[1][0] = m_matB[1][j];
	m_A[0][2] = m_A[2][0] = m_matB[2][j];
	m_A[1][1] = m_matB[3][j];
	m_A[1][2] = m_A[2][1] = m_matB[4][j];
	m_A[2][2] = m_matB[5][j];

	// negate entire solution if A has negative determinant
	det = f3x3matrixDetA(m_A);
	if (det < 0.0F) {
		f3x3matrixAeqMinusA(m_A);
		m_matB[6][j] = -m_matB[6][j];
		m_matB[7][j] = -m_matB[7][j];
		m_matB[8][j] = -m_matB[8][j];
		m_matB[9][j] = -m_matB[9][j];
		det = -det;
	}

	// compute the inverse of the ellipsoid matrix
	f3x3matrixAeqInvSymB(m_invA, m_A);

	// compute the trial hard iron vector in offset bit counts times FMATRIXSCALING
	for (k = X; k <= Z; k++) {
		m_calNext.m_vecV[k] = 0.0F;
		for (m = X; m <= Z; m++) {
			m_calNext.m_vecV[k] += m_invA[k][m] * m_matB[m + 6][j];
		}
		m_calNext.m_vecV[k] *= -0.5F;
	}

	// compute the trial geomagnetic field strength B in bit counts times FMATRIXSCALING
	m_calNext.m_sB = sqrtf(fabs(m_A[0][0] * m_calNext.m_vecV[X] * m_calNext.m_vecV[X] +
			2.0F * m_A[0][1] * m_calNext.m_vecV[X] * m_calNext.m_vecV[Y] +
			2.0F * m_A[0][2] * m_calNext.m_vecV[X] * m_calNext.m_vecV[Z] +
			m_A[1][1] * m_calNext.m_vecV[Y] * m_calNext.m_vecV[Y] +
			2.0F * m_A[1][2] * m_calNext.m_vecV[Y] * m_calNext.m_vecV[Z] +
			m_A[2][2] * m_calNext.m_vecV[Z] * m_calNext.m_vecV[Z] - m_matB[9][j]));

	// calculate the trial normalized fit error as a percentage
	m_errFitNext = 50.0F * sqrtf(
		fabs(m_vecA[j]) / float(m_samps.m_cSamp)) /
		(m_calNext.m_sB * m_calNext.m_sB);

	// correct for the measurement matrix offset and scaling and
	// get the computed hard iron offset in uT
	for (k = X; k <= Z; k++) {
		m_calNext.m_vecV[k] = m_calNext.m_vecV[k] * Mag::s_sBDefault + BpOffset[k];
	}

	// convert the trial geomagnetic field strength B into uT for
	// un-normalized soft iron matrix A
	m_calNext.m_sB *= Mag::s_sBDefault;

	// normalize the ellipsoid matrix A to unit determinant and
	// correct B by root of this multiplicative factor
	f3x3matrixAeqAxScalar(m_A, powf(det, -(s_sOneThird)));
	m_calNext.m_sB *= powf(det, -(s_sOneSixth));

	// compute m_calNext.m_matWInv from the square root of fA (both with normalized determinant)
	// set m_vecA to the unsorted eigenvalues and m_matB to the unsorted eigenvectors of m_matA
	// where m_matA holds the 3x3 matrix fA in its top left elements
	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			m_matA[i][j] = m_A[i][j];
		}
	}
	eigencompute(m_matA, m_vecA, m_matB, 3);

	// set m_matB to be eigenvectors . diag(sqrt(sqrt(eigenvalues))) =
	//   m_matB . diag(sqrt(sqrt(m_vecA))
	for (j = 0; j < 3; j++) { // loop over columns j
		ftmp = sqrtf(sqrtf(fabs(m_vecA[j])));
		for (i = 0; i < 3; i++) { // loop over rows i
			m_matB[i][j] *= ftmp;
		}
	}

	// set m_calNext.m_matWInv to eigenvectors * diag(sqrt(eigenvalues)) * eigenvectors^T =
	//   m_matB * m_matB^T = sqrt(fA) (guaranteed symmetric)
	// loop over rows
	for (i = 0; i < 3; i++) {
		// loop over on and above diagonal columns
		for (j = i; j < 3; j++) {
			m_calNext.m_matWInv[i][j] = 0.0F;
			// accumulate the matrix product
			for (k = 0; k < 3; k++) {
				m_calNext.m_matWInv[i][j] += m_matB[i][k] * m_matB[j][k];
			}
			// copy to below diagonal element
			m_calNext.m_matWInv[j][i] = m_calNext.m_matWInv[i][j];
		}
	}
}

void CFitter::CSampleSet::Recalibrate(const Mag::SCal & cal)
{
	memset(m_mpRegionCSamp, 0, sizeof(m_mpRegionCSamp));

	for (int iSamp = 0; iSamp < m_cSamp; ++iSamp)
	{
		m_aSamp[iSamp].Calibrate(cal);
		m_mpRegionCSamp[m_aSamp[iSamp].m_region]++;
	}
}

void CFitter::SSample::Calibrate(const Mag::SCal & cal)
{
	float x = m_pntRaw.x - cal.m_vecV[0];
	float y = m_pntRaw.y - cal.m_vecV[1];
	float z = m_pntRaw.z - cal.m_vecV[2];

	m_pntCal.x = x * cal.m_matWInv[0][0] + y * cal.m_matWInv[0][1] + z * cal.m_matWInv[0][2];
	m_pntCal.y = x * cal.m_matWInv[1][0] + y * cal.m_matWInv[1][1] + z * cal.m_matWInv[1][2];
	m_pntCal.z = x * cal.m_matWInv[2][0] + y * cal.m_matWInv[2][1] + z * cal.m_matWInv[2][2];

	x = m_pntCal.x;
	y = m_pntCal.y;
	z = m_pntCal.z;

	m_sB = sqrtf(x * x + y * y + z * z);

	if (m_sB > 0.0f)
	{
		m_region = RegionFromXyz(
					m_pntCal.x / m_sB,
					m_pntCal.y / m_sB,
					m_pntCal.z / m_sB);
	}
	else
	{
		m_region = REGION_Default;
	}
}

} // namespace Sphere
} // namespace libcalib
