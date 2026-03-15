#pragma once

#include "libcalib_common.h"
#include "libcalib_magcal.h"

#include <algorithm>
#include <math.h>

namespace libcalib
{

// a class which breaks a sphere into REGION_Max partitions of roughly equal
//	size and radius. details here:
//		https://etna.mcs.kent.edu/vol.25.2006/pp309-327.dir/pp309-327.html
// sphere equations....
//  area of unit sphere = 4*pi
//  area of unit sphere cap = 2*pi*h  h = cap height
//  latitude of unit sphere cap = arcsin(1 - h)

struct SpherePartition
{
	SpherePartition();

	SPoint	m_mpRegionAnchor[REGION_Max];

	int RegionFromXyz(float x, float y, float z);

private:
	struct Collar
	{
		int		m_cRegion;	// number of regions
		float	m_latMax;	// upper/lower latitude in radians
		float	m_latMin;
	};


	static constexpr int s_cRegionPole =		1;
	static constexpr int s_cRegionTemperate =	15;
	static constexpr int s_cRegionTropic =		34;

	static constexpr float s_degNorthPole =		90.0f;
	static constexpr float s_degTemperateMax =	78.52;
	static constexpr float s_degTropicMax =		42.84;
	static constexpr float s_degEquator =		0.0f;
	static constexpr float s_degTropicMin =		-s_degTropicMax;
	static constexpr float s_degTemperateMin =	-s_degTemperateMax;
	static constexpr float s_degSouthPole =		-s_degNorthPole;

	static constexpr float s_radNorthPole =		RadFromDeg(s_degNorthPole);
	static constexpr float s_radTemperateMax =	RadFromDeg(s_degTemperateMax);
	static constexpr float s_radTropicMax =		RadFromDeg(s_degTropicMax);
	static constexpr float s_radEquator =		RadFromDeg(s_degEquator);
	static constexpr float s_radTropicMin =		RadFromDeg(s_degTropicMin);
	static constexpr float s_radTemperateMin =	RadFromDeg(s_degTemperateMin);
	static constexpr float s_radSouthPole =		RadFromDeg(s_degSouthPole);

	static constexpr Collar s_aCollar[] =
	{
		// m_cRegion			m_latMax			m_latMin
		{ s_cRegionPole,		s_radNorthPole,		s_radTemperateMax },
		{ s_cRegionTemperate,	s_radTemperateMax,	s_radTropicMax },
		{ s_cRegionTropic,		s_radTropicMax,		s_radEquator },
		{ s_cRegionTropic,		s_radEquator,		s_radTropicMin },
		{ s_cRegionTemperate,	s_radTropicMin,		s_radTemperateMin },
		{ s_cRegionPole,		s_radTemperateMin,	s_radSouthPole },
	};
	static constexpr int s_cCollar = DIM(s_aCollar);

	static_assert(REGION_Max == (
					s_aCollar[0].m_cRegion +
					s_aCollar[1].m_cRegion +
					s_aCollar[2].m_cRegion +
					s_aCollar[3].m_cRegion +
					s_aCollar[4].m_cRegion +
					s_aCollar[5].m_cRegion));
	static_assert(s_cCollar == 6);
};

extern SpherePartition g_sphere_partition;

} // namespace libcalib
