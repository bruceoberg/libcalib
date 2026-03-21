#include "libcalib/calibrator.h"
#include "sphere.h"

#include <math.h>
#include <string.h>

namespace libcalib
{

namespace Sphere
{

using namespace Quality;

void CFitter::UpdateQuality()
{
	if (m_fHasQuality)
		return;

	UpdateErrGaps();
	UpdateErrVariance();
	UpdateErrWobble();

	m_fHasQuality = true;
}

bool CFitter::AreErrorsOk() const
{
	if (m_errGaps >= s_errGapsOkMin)
		return false;
	if (m_errVariance >= s_errVarianceOkMin)
		return false;
	if (m_errWobble >= s_errWobbleOkMin)
		return false;
	if (m_errFit >= s_errFitOkMin)
		return false;

	return true;
}

bool CFitter::AreErrorsBad() const
{
	if (m_errGaps <= s_errGapsBadMax)
		return false;
	if (m_errVariance <= s_errVarianceBadMax)
		return false;
	if (m_errWobble <= s_errWobbleBadMax)
		return false;
	if (m_errFit <= s_errFitBadMax)
		return false;

	return true;
}

// How many surface gaps

void CFitter::UpdateErrGaps()
{
	m_errGaps = 0.0f;

	for (int iRegion = REGION_Min; iRegion < REGION_Max; iRegion++)
	{
		REGION region = REGION(iRegion);
		int cSamp = m_samps.CSampFromRegion(region);

		if (cSamp == 0) {
			m_errGaps += 1.0f;
		} else if (cSamp == 1) {
			m_errGaps += 0.2f;
		} else if (cSamp == 2) {
			m_errGaps += 0.01f;
		}
	}
}

// Variance in magnitude

void CFitter::UpdateErrVariance()
{
	if (m_samps.FIsEmpty())
	{
		m_errVariance = s_errMax;
		return;
	}

	float sBSum = 0.0f;
	int cSamp = 0;
	for (const auto & samp : m_samps) {
		sBSum += samp.m_sB;
		cSamp += 1;
	}

	float sBAverage = sBSum / cSamp;

	float sVariance = 0.0f;
	for (const auto & samp : m_samps) {
		float dSB = samp.m_sB - sBAverage;
		sVariance += dSB * dSB;
	}

	sVariance /= cSamp;

	m_errVariance = sqrtf(sVariance) / sBAverage * s_errMax;
}

// Offset of piecewise average data from ideal sphere surface

void CFitter::UpdateErrWobble()
{
	if (m_samps.FIsEmpty())
	{
		m_errWobble = s_errMax;
		return;
	}

	float sBSum = 0.0f;
	int cSamp = 0;
	for (const auto & samp : m_samps) {
		sBSum += samp.m_sB;
		cSamp += 1;
	}

	float sBAverage = sBSum / cSamp;

	// compute per-region sums locally

	SPoint mpRegionSum[REGION_Max] = {};
	for (const auto & samp : m_samps)
	{
		mpRegionSum[samp.m_region].x += samp.m_pntCal.x;
		mpRegionSum[samp.m_region].y += samp.m_pntCal.y;
		mpRegionSum[samp.m_region].z += samp.m_pntCal.z;
	}

	float xoff = 0.0f;
	float yoff = 0.0f;
	float zoff = 0.0f;
	int cRegionHit = 0;

	for (int iRegion = REGION_Min; iRegion < REGION_Max; iRegion++)
	{
		REGION region = REGION(iRegion);
		int cSampRegion = m_samps.CSampFromRegion(region);
		if (cSampRegion > 0)
		{
			float x = mpRegionSum[region].x / cSampRegion;
			float y = mpRegionSum[region].y / cSampRegion;
			float z = mpRegionSum[region].z / cSampRegion;

			const SPoint & pntAnchor = PntAnchorFromRegion(region);

			xoff += x - pntAnchor.x * sBAverage;
			yoff += y - pntAnchor.y * sBAverage;
			zoff += z - pntAnchor.z * sBAverage;

			cRegionHit++;
		}
	}

	if (cRegionHit == 0)
	{
		m_errWobble = s_errMax;
		return;
	}

	xoff /= cRegionHit;
	yoff /= cRegionHit;
	zoff /= cRegionHit;

	m_errWobble = sqrtf(xoff * xoff + yoff * yoff + zoff * zoff) / sBAverage * s_errMax;
}

} // namespace Sphere

} // namespace libcalib
