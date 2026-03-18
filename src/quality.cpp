#include "libcalib/calibrator.h"
#include "sphere.h"

#include <math.h>
#include <string.h>

namespace libcalib
{

MagQuality::MagQuality()
: m_errGaps(s_errMax)
, m_errVariance(s_errMax)
, m_errWobble(s_errMax)
, m_errFit(s_errMax)
, m_isValid(false)
{
}

void MagQuality::Ensure(const CSphereFitter & sphitter)
{
	if (m_isValid)
		return;

	m_errGaps = ErrGaps(sphitter);
	m_errVariance = ErrVariance(sphitter);
	m_errWobble = ErrWobble(sphitter);
	m_errFit = sphitter.m_errFit;

	m_isValid = true;
}

bool MagQuality::AreErrorsOk() const
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

bool MagQuality::AreErrorsBad() const
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

float MagQuality::ErrGaps(const CSphereFitter & sphitter)
{
	float error = 0.0f;

	for (int region = 0; region < REGION_Max; region++) {
		int cSamp = sphitter.m_samps.CSampFromRegion(static_cast<REGION>(region));

		if (cSamp == 0) {
			error += 1.0f;
		} else if (cSamp == 1) {
			error += 0.2f;
		} else if (cSamp == 2) {
			error += 0.01f;
		}
	}

	return error;
}

// Variance in magnitude

float MagQuality::ErrVariance(const CSphereFitter & sphitter)
{
	if (sphitter.m_samps.CSamp() == 0)
		return s_errMax;

	float sum = 0.0f;
	for (int i = 0; i < sphitter.m_samps.CSamp(); i++) {
		sum += sphitter.m_samps.Samp(i).m_field;
	}

	float mean = sum / float(sphitter.m_samps.CSamp());

	float variance = 0.0f;
	for (int i = 0; i < sphitter.m_samps.CSamp(); i++) {
		float diff = sphitter.m_samps.Samp(i).m_field - mean;
		variance += diff * diff;
	}

	variance /= float(sphitter.m_samps.CSamp());

	return sqrtf(variance) / mean * s_errMax;
}

// Offset of piecewise average data from ideal sphere surface

float MagQuality::ErrWobble(const CSphereFitter & sphitter)
{
	if (sphitter.m_samps.CSamp() == 0)
		return s_errMax;

	float sum = 0.0f;
	for (int i = 0; i < sphitter.m_samps.CSamp(); i++) {
		sum += sphitter.m_samps.Samp(i).m_field;
	}

	float radius = sum / float(sphitter.m_samps.CSamp());

	// compute per-region sums locally

	SPoint mpRegionSum[REGION_Max] = {};
	for (int i = 0; i < sphitter.m_samps.CSamp(); i++)
	{
		const MagSample & samp = sphitter.m_samps.Samp(i);
		mpRegionSum[samp.m_region].x += samp.m_pntCal.x;
		mpRegionSum[samp.m_region].y += samp.m_pntCal.y;
		mpRegionSum[samp.m_region].z += samp.m_pntCal.z;
	}

	float xoff = 0.0f;
	float yoff = 0.0f;
	float zoff = 0.0f;
	int cRegionHit = 0;

	for (int region = 0; region < REGION_Max; region++)
	{
		int cSamp = sphitter.m_samps.CSampFromRegion(static_cast<REGION>(region));
		if (cSamp > 0)
		{
			float x = mpRegionSum[region].x / float(cSamp);
			float y = mpRegionSum[region].y / float(cSamp);
			float z = mpRegionSum[region].z / float(cSamp);

			const SPoint & pntAnchor = Sphere::PntAnchorFromRegion(region);

			xoff += x - pntAnchor.x * radius;
			yoff += y - pntAnchor.y * radius;
			zoff += z - pntAnchor.z * radius;

			cRegionHit++;
		}
	}

	if (cRegionHit == 0)
		return s_errMax;

	xoff /= float(cRegionHit);
	yoff /= float(cRegionHit);
	zoff /= float(cRegionHit);

	return sqrtf(xoff * xoff + yoff * yoff + zoff * zoff) / radius * s_errMax;
}

} // namespace libcalib
