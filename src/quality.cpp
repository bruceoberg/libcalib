#include "libcalib.h"
#include "sphere_partition.h"

#include <math.h>
#include <string.h>

namespace libcalib
{

SpherePartition::SpherePartition()
{
	static_assert(s_aCollar[0].m_cRegion == 1);

	m_mpRegionAnchor[0].x = 0.0f;
	m_mpRegionAnchor[0].y = 0.0f;
	m_mpRegionAnchor[0].z = 1.0f;

	SPoint * pAnchor = &m_mpRegionAnchor[1];

	// NOTE: skipping first and last collars (the poles);

	for (int iCollar = 1; iCollar < s_cCollar - 1; ++iCollar)
	{
		auto & collar = s_aCollar[iCollar];
		float latMiddle = (collar.m_latMin + collar.m_latMax) / 2.0f;
		float radiusMiddle = cosf(latMiddle);
		float zMiddle = sinf(latMiddle);
		float dLongitude = (M_PI * 2.0 / float(collar.m_cRegion));

		for (int subregion = 0; subregion < collar.m_cRegion; ++subregion)
		{
			float longitude = (float(subregion) + 0.5f) * dLongitude;

			// negation here is to flip regions into a winding order that matches
			// the subregion from longitude calculation in RegionFromXyz().

			pAnchor->x = -cosf(longitude) * radiusMiddle;
			pAnchor->y = -sinf(longitude) * radiusMiddle;
			pAnchor->z = zMiddle;

			++pAnchor;
		}
	}

	static_assert(s_aCollar[s_cCollar - 1].m_cRegion == 1);

	m_mpRegionAnchor[REGION_Max - 1].x = 0.0f;
	m_mpRegionAnchor[REGION_Max - 1].y = 0.0f;
	m_mpRegionAnchor[REGION_Max - 1].z = -1.0f;

	//for (int region = 0; region < REGION_Max; ++region)
	//{
	//	const auto & anchor = m_mpRegionAnchor[region];
	//	assert(region == RegionFromXyz(anchor.x, anchor.y, anchor.z));
	//}
}

int SpherePartition::RegionFromXyz(float x, float y, float z)
{
	float latitude = (M_PI / 2.0) - atan2f(sqrtf(x * x + y * y), z);

	int regionCur = 0;

	for (auto collar : s_aCollar)
	{
		if (latitude >= collar.m_latMin)
		{
			if (collar.m_cRegion <= 1)
				return regionCur;

			float longitude = atan2f(y, x) + M_PI;
			int subregion = floorf(float(collar.m_cRegion) * longitude / (M_PI * 2.0));
			subregion = std::clamp(subregion, 0, collar.m_cRegion - 1);

			return regionCur + subregion;
		}

		regionCur += collar.m_cRegion;
	}

	return 0;
}

SpherePartition g_sphere_partition;

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

			float xi = g_sphere_partition.m_mpRegionAnchor[region].x * radius;
			float yi = g_sphere_partition.m_mpRegionAnchor[region].y * radius;
			float zi = g_sphere_partition.m_mpRegionAnchor[region].z * radius;

			xoff += x - xi;
			yoff += y - yi;
			zoff += z - zi;

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
