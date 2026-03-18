#include "sphere.h"

#include <algorithm>
#include <math.h>

namespace libcalib
{
namespace Sphere
{
namespace Partition
{

// Sphere::Partition breaks a sphere into REGION_Max partitions of roughly equal
//	size and radius. details here:
//		https://etna.mcs.kent.edu/vol.25.2006/pp309-327.dir/pp309-327.html
// sphere equations....
//  area of unit sphere = 4*pi
//  area of unit sphere cap = 2*pi*h  h = cap height
//  latitude of unit sphere cap = arcsin(1 - h)

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

// precomputed anchor points for each region, used by quality metrics.
// see SetSphereAnchors() below for the code that originally generated this table.

static const SPoint s_mpRegionPntAnchor[REGION_Max] =
{
	{  0.00000000e+00f,  0.00000000e+00f,  1.00000000e+00f },	// north pole
	{ -4.78985995e-01f, -1.01811618e-01f,  8.71898413e-01f },	// temperate+[0]
	{ -3.96164954e-01f, -2.87830681e-01f,  8.71898413e-01f },	// temperate+[1]
	{ -2.44843394e-01f, -4.24081236e-01f,  8.71898413e-01f },	// temperate+[2]
	{ -5.11861891e-02f, -4.87004280e-01f,  8.71898413e-01f },	// temperate+[3]
	{  1.51321560e-01f, -4.65719819e-01f,  8.71898413e-01f },	// temperate+[4]
	{  3.27664465e-01f, -3.63908231e-01f,  8.71898413e-01f },	// temperate+[5]
	{  4.47351187e-01f, -1.99173540e-01f,  8.71898413e-01f },	// temperate+[6]
	{  4.89686817e-01f,  4.28097806e-08f,  8.71898413e-01f },	// temperate+[7]
	{  4.47351158e-01f,  1.99173614e-01f,  8.71898413e-01f },	// temperate+[8]
	{  3.27664375e-01f,  3.63908231e-01f,  8.71898413e-01f },	// temperate+[9]
	{  1.51321590e-01f,  4.65719819e-01f,  8.71898413e-01f },	// temperate+[10]
	{ -5.11863939e-02f,  4.87004250e-01f,  8.71898413e-01f },	// temperate+[11]
	{ -2.44843364e-01f,  4.24081236e-01f,  8.71898413e-01f },	// temperate+[12]
	{ -3.96165073e-01f,  2.87830532e-01f,  8.71898413e-01f },	// temperate+[13]
	{ -4.78985965e-01f,  1.01811647e-01f,  8.71898413e-01f },	// temperate+[14]
	{ -9.26957190e-01f, -8.58952403e-02f,  3.65201771e-01f },	// tropic+[0]
	{ -8.95390868e-01f, -2.54760653e-01f,  3.65201771e-01f },	// tropic+[1]
	{ -8.33332956e-01f, -4.14950490e-01f,  3.65201771e-01f },	// tropic+[2]
	{ -7.42896914e-01f, -5.61009705e-01f,  3.65201771e-01f },	// tropic+[3]
	{ -6.27162397e-01f, -6.87964380e-01f,  3.65201771e-01f },	// tropic+[4]
	{ -4.90070701e-01f, -7.91491270e-01f,  3.65201771e-01f },	// tropic+[5]
	{ -3.36290151e-01f, -8.68064880e-01f,  3.65201771e-01f },	// tropic+[6]
	{ -1.71057701e-01f, -9.15077567e-01f,  3.65201771e-01f },	// tropic+[7]
	{  4.06921714e-08f, -9.30928409e-01f,  3.65201771e-01f },	// tropic+[8]
	{  1.71057671e-01f, -9.15077567e-01f,  3.65201771e-01f },	// tropic+[9]
	{  3.36290121e-01f, -8.68064880e-01f,  3.65201771e-01f },	// tropic+[10]
	{  4.90070641e-01f, -7.91491330e-01f,  3.65201771e-01f },	// tropic+[11]
	{  6.27162516e-01f, -6.87964320e-01f,  3.65201771e-01f },	// tropic+[12]
	{  7.42896914e-01f, -5.61009705e-01f,  3.65201771e-01f },	// tropic+[13]
	{  8.33332956e-01f, -4.14950430e-01f,  3.65201771e-01f },	// tropic+[14]
	{  8.95390809e-01f, -2.54760712e-01f,  3.65201771e-01f },	// tropic+[15]
	{  9.26957190e-01f, -8.58952031e-02f,  3.65201771e-01f },	// tropic+[16]
	{  9.26957250e-01f,  8.58951434e-02f,  3.65201771e-01f },	// tropic+[17]
	{  8.95390868e-01f,  2.54760653e-01f,  3.65201771e-01f },	// tropic+[18]
	{  8.33333015e-01f,  4.14950371e-01f,  3.65201771e-01f },	// tropic+[19]
	{  7.42896914e-01f,  5.61009705e-01f,  3.65201771e-01f },	// tropic+[20]
	{  6.27162337e-01f,  6.87964439e-01f,  3.65201771e-01f },	// tropic+[21]
	{  4.90070552e-01f,  7.91491389e-01f,  3.65201771e-01f },	// tropic+[22]
	{  3.36290300e-01f,  8.68064821e-01f,  3.65201771e-01f },	// tropic+[23]
	{  1.71057731e-01f,  9.15077567e-01f,  3.65201771e-01f },	// tropic+[24]
	{ -1.11012097e-08f,  9.30928409e-01f,  3.65201771e-01f },	// tropic+[25]
	{ -1.71057746e-01f,  9.15077567e-01f,  3.65201771e-01f },	// tropic+[26]
	{ -3.36289912e-01f,  8.68065000e-01f,  3.65201771e-01f },	// tropic+[27]
	{ -4.90070552e-01f,  7.91491389e-01f,  3.65201771e-01f },	// tropic+[28]
	{ -6.27162397e-01f,  6.87964439e-01f,  3.65201771e-01f },	// tropic+[29]
	{ -7.42896914e-01f,  5.61009705e-01f,  3.65201771e-01f },	// tropic+[30]
	{ -8.33333015e-01f,  4.14950371e-01f,  3.65201771e-01f },	// tropic+[31]
	{ -8.95390809e-01f,  2.54760832e-01f,  3.65201771e-01f },	// tropic+[32]
	{ -9.26957190e-01f,  8.58953446e-02f,  3.65201771e-01f },	// tropic+[33]
	{ -9.26957190e-01f, -8.58952403e-02f, -3.65201771e-01f },	// tropic-[0]
	{ -8.95390868e-01f, -2.54760653e-01f, -3.65201771e-01f },	// tropic-[1]
	{ -8.33332956e-01f, -4.14950490e-01f, -3.65201771e-01f },	// tropic-[2]
	{ -7.42896914e-01f, -5.61009705e-01f, -3.65201771e-01f },	// tropic-[3]
	{ -6.27162397e-01f, -6.87964380e-01f, -3.65201771e-01f },	// tropic-[4]
	{ -4.90070701e-01f, -7.91491270e-01f, -3.65201771e-01f },	// tropic-[5]
	{ -3.36290151e-01f, -8.68064880e-01f, -3.65201771e-01f },	// tropic-[6]
	{ -1.71057701e-01f, -9.15077567e-01f, -3.65201771e-01f },	// tropic-[7]
	{  4.06921714e-08f, -9.30928409e-01f, -3.65201771e-01f },	// tropic-[8]
	{  1.71057671e-01f, -9.15077567e-01f, -3.65201771e-01f },	// tropic-[9]
	{  3.36290121e-01f, -8.68064880e-01f, -3.65201771e-01f },	// tropic-[10]
	{  4.90070641e-01f, -7.91491330e-01f, -3.65201771e-01f },	// tropic-[11]
	{  6.27162516e-01f, -6.87964320e-01f, -3.65201771e-01f },	// tropic-[12]
	{  7.42896914e-01f, -5.61009705e-01f, -3.65201771e-01f },	// tropic-[13]
	{  8.33332956e-01f, -4.14950430e-01f, -3.65201771e-01f },	// tropic-[14]
	{  8.95390809e-01f, -2.54760712e-01f, -3.65201771e-01f },	// tropic-[15]
	{  9.26957190e-01f, -8.58952031e-02f, -3.65201771e-01f },	// tropic-[16]
	{  9.26957250e-01f,  8.58951434e-02f, -3.65201771e-01f },	// tropic-[17]
	{  8.95390868e-01f,  2.54760653e-01f, -3.65201771e-01f },	// tropic-[18]
	{  8.33333015e-01f,  4.14950371e-01f, -3.65201771e-01f },	// tropic-[19]
	{  7.42896914e-01f,  5.61009705e-01f, -3.65201771e-01f },	// tropic-[20]
	{  6.27162337e-01f,  6.87964439e-01f, -3.65201771e-01f },	// tropic-[21]
	{  4.90070552e-01f,  7.91491389e-01f, -3.65201771e-01f },	// tropic-[22]
	{  3.36290300e-01f,  8.68064821e-01f, -3.65201771e-01f },	// tropic-[23]
	{  1.71057731e-01f,  9.15077567e-01f, -3.65201771e-01f },	// tropic-[24]
	{ -1.11012097e-08f,  9.30928409e-01f, -3.65201771e-01f },	// tropic-[25]
	{ -1.71057746e-01f,  9.15077567e-01f, -3.65201771e-01f },	// tropic-[26]
	{ -3.36289912e-01f,  8.68065000e-01f, -3.65201771e-01f },	// tropic-[27]
	{ -4.90070552e-01f,  7.91491389e-01f, -3.65201771e-01f },	// tropic-[28]
	{ -6.27162397e-01f,  6.87964439e-01f, -3.65201771e-01f },	// tropic-[29]
	{ -7.42896914e-01f,  5.61009705e-01f, -3.65201771e-01f },	// tropic-[30]
	{ -8.33333015e-01f,  4.14950371e-01f, -3.65201771e-01f },	// tropic-[31]
	{ -8.95390809e-01f,  2.54760832e-01f, -3.65201771e-01f },	// tropic-[32]
	{ -9.26957190e-01f,  8.58953446e-02f, -3.65201771e-01f },	// tropic-[33]
	{ -4.78985995e-01f, -1.01811618e-01f, -8.71898413e-01f },	// temperate-[0]
	{ -3.96164954e-01f, -2.87830681e-01f, -8.71898413e-01f },	// temperate-[1]
	{ -2.44843394e-01f, -4.24081236e-01f, -8.71898413e-01f },	// temperate-[2]
	{ -5.11861891e-02f, -4.87004280e-01f, -8.71898413e-01f },	// temperate-[3]
	{  1.51321560e-01f, -4.65719819e-01f, -8.71898413e-01f },	// temperate-[4]
	{  3.27664465e-01f, -3.63908231e-01f, -8.71898413e-01f },	// temperate-[5]
	{  4.47351187e-01f, -1.99173540e-01f, -8.71898413e-01f },	// temperate-[6]
	{  4.89686817e-01f,  4.28097806e-08f, -8.71898413e-01f },	// temperate-[7]
	{  4.47351158e-01f,  1.99173614e-01f, -8.71898413e-01f },	// temperate-[8]
	{  3.27664375e-01f,  3.63908231e-01f, -8.71898413e-01f },	// temperate-[9]
	{  1.51321590e-01f,  4.65719819e-01f, -8.71898413e-01f },	// temperate-[10]
	{ -5.11863939e-02f,  4.87004250e-01f, -8.71898413e-01f },	// temperate-[11]
	{ -2.44843364e-01f,  4.24081236e-01f, -8.71898413e-01f },	// temperate-[12]
	{ -3.96165073e-01f,  2.87830532e-01f, -8.71898413e-01f },	// temperate-[13]
	{ -4.78985965e-01f,  1.01811647e-01f, -8.71898413e-01f },	// temperate-[14]
	{  0.00000000e+00f,  0.00000000e+00f, -1.00000000e+00f },	// south pole
};

// This is the original code that computed s_mpRegionPntAnchor at runtime.
// It is retained here so the table can be regenerated and verified if needed.

void SetSphereAnchors(SPoint (&mpRegionAnchor)[REGION_Max])
{
	static_assert(s_aCollar[0].m_cRegion == 1);

	mpRegionAnchor[0].x = 0.0f;
	mpRegionAnchor[0].y = 0.0f;
	mpRegionAnchor[0].z = 1.0f;

	SPoint * pAnchor = &mpRegionAnchor[1];

	// NOTE(claude): skipping first and last collars (the poles)

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

	mpRegionAnchor[REGION_Max - 1].x = 0.0f;
	mpRegionAnchor[REGION_Max - 1].y = 0.0f;
	mpRegionAnchor[REGION_Max - 1].z = -1.0f;
}

} // namespace Partition

using namespace Partition;

int RegionFromXyz(float x, float y, float z)
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

const SPoint & PntAnchorFromRegion(int region)
{
	return s_mpRegionPntAnchor[region];
}

} // namespace Sphere
} // namespace libcalib
