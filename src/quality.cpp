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

// build by claude here: https://claude.ai/share/245c0e7a-1ec9-45c6-bbbc-6041f983a5cb

static const int s_mpRowColIRegionSpiral[10][10] =
{
    {  72,  73,  74,  75,  76,  77,  78,  79,  80,  81 },
    {  71,  42,  43,  44,  45,  46,  47,  48,  49,  82 },
    {  70,  41,  20,  21,  22,  23,  24,  25,  50,  83 },
    {  69,  40,  19,   6,   7,   8,   9,  26,  51,  84 },
    {  68,  39,  18,   5,   0,   1,  10,  27,  52,  85 },
    {  67,  38,  17,   4,   3,   2,  11,  28,  53,  86 },
    {  66,  37,  16,  15,  14,  13,  12,  29,  54,  87 },
    {  65,  36,  35,  34,  33,  32,  31,  30,  55,  88 },
    {  64,  63,  62,  61,  60,  59,  58,  57,  56,  89 },
    {  99,  98,  97,  96,  95,  94,  93,  92,  91,  90 },
};
static_assert(REGION_Max == 100);

// inverse of s_mpRowColIRegionSpiral

static const int s_mpIRegionRowColSpiral[REGION_Max][2] =
{
    {  4,  4 },  // region 0
    {  4,  5 },  // region 1
    {  5,  5 },  // region 2
    {  5,  4 },  // region 3
    {  5,  3 },  // region 4
    {  4,  3 },  // region 5
    {  3,  3 },  // region 6
    {  3,  4 },  // region 7
    {  3,  5 },  // region 8
    {  3,  6 },  // region 9
    {  4,  6 },  // region 10
    {  5,  6 },  // region 11
    {  6,  6 },  // region 12
    {  6,  5 },  // region 13
    {  6,  4 },  // region 14
    {  6,  3 },  // region 15
    {  6,  2 },  // region 16
    {  5,  2 },  // region 17
    {  4,  2 },  // region 18
    {  3,  2 },  // region 19
    {  2,  2 },  // region 20
    {  2,  3 },  // region 21
    {  2,  4 },  // region 22
    {  2,  5 },  // region 23
    {  2,  6 },  // region 24
    {  2,  7 },  // region 25
    {  3,  7 },  // region 26
    {  4,  7 },  // region 27
    {  5,  7 },  // region 28
    {  6,  7 },  // region 29
    {  7,  7 },  // region 30
    {  7,  6 },  // region 31
    {  7,  5 },  // region 32
    {  7,  4 },  // region 33
    {  7,  3 },  // region 34
    {  7,  2 },  // region 35
    {  7,  1 },  // region 36
    {  6,  1 },  // region 37
    {  5,  1 },  // region 38
    {  4,  1 },  // region 39
    {  3,  1 },  // region 40
    {  2,  1 },  // region 41
    {  1,  1 },  // region 42
    {  1,  2 },  // region 43
    {  1,  3 },  // region 44
    {  1,  4 },  // region 45
    {  1,  5 },  // region 46
    {  1,  6 },  // region 47
    {  1,  7 },  // region 48
    {  1,  8 },  // region 49
    {  2,  8 },  // region 50
    {  3,  8 },  // region 51
    {  4,  8 },  // region 52
    {  5,  8 },  // region 53
    {  6,  8 },  // region 54
    {  7,  8 },  // region 55
    {  8,  8 },  // region 56
    {  8,  7 },  // region 57
    {  8,  6 },  // region 58
    {  8,  5 },  // region 59
    {  8,  4 },  // region 60
    {  8,  3 },  // region 61
    {  8,  2 },  // region 62
    {  8,  1 },  // region 63
    {  8,  0 },  // region 64
    {  7,  0 },  // region 65
    {  6,  0 },  // region 66
    {  5,  0 },  // region 67
    {  4,  0 },  // region 68
    {  3,  0 },  // region 69
    {  2,  0 },  // region 70
    {  1,  0 },  // region 71
    {  0,  0 },  // region 72
    {  0,  1 },  // region 73
    {  0,  2 },  // region 74
    {  0,  3 },  // region 75
    {  0,  4 },  // region 76
    {  0,  5 },  // region 77
    {  0,  6 },  // region 78
    {  0,  7 },  // region 79
    {  0,  8 },  // region 80
    {  0,  9 },  // region 81
    {  1,  9 },  // region 82
    {  2,  9 },  // region 83
    {  3,  9 },  // region 84
    {  4,  9 },  // region 85
    {  5,  9 },  // region 86
    {  6,  9 },  // region 87
    {  7,  9 },  // region 88
    {  8,  9 },  // region 89
    {  9,  9 },  // region 90
    {  9,  8 },  // region 91
    {  9,  7 },  // region 92
    {  9,  6 },  // region 93
    {  9,  5 },  // region 94
    {  9,  4 },  // region 95
    {  9,  3 },  // region 96
    {  9,  2 },  // region 97
    {  9,  1 },  // region 98
    {  9,  0 },  // region 99
};
static_assert(DIM(s_mpIRegionRowColSpiral) == REGION_Max);

// generated by this routine from claude code.

// Builds a 10x10 clockwise spiral mapping of region indices onto a display grid.
// Region 0 (north pole) lands at [4][4]; the spiral unwinds right, down, left, up.
// [4][4]=0, [4][5]=1, [5][5]=2, [5][4]=3, ...
static void BuildSpiralMap(int mpRowColIRegion[10][10])
{
	// Clockwise spiral directions: right, down, left, up
	enum DIR
	{
		DIR_Right,
		DIR_Down,
		DIR_Left,
		DIR_Up,

		DIR_Max,
		DIR_Nil = -1
	};

	static const int s_mpDirDRow[] = {  0,  1,  0, -1 }; static_assert(DIM(s_mpDirDRow) == DIR_Max);
	static const int s_mpDirDCol[] = {  1,  0, -1,  0 }; static_assert(DIM(s_mpDirDCol) == DIR_Max);

	memset(mpRowColIRegion, -1, 10 * 10 * sizeof(int));

	int row = 4, col = 4;
	DIR dir = DIR_Right;		// start moving right
	int cStep = 1;				// steps before next turn
	int cDirChange = 0;
	int iRegionCur = 0;

	mpRowColIRegion[row][col] = iRegionCur++;

	while (iRegionCur < 100)
	{
		for (int i = 0; i < cStep && iRegionCur < 100; i++)
		{
			row += s_mpDirDRow[dir];
			col += s_mpDirDCol[dir];
			if (row >= 0 && row < 10 && col >= 0 && col < 10)
				mpRowColIRegion[row][col] = iRegionCur;
			iRegionCur++;
		}
		dir = DIR((dir + 1) % DIR_Max);
		if (++cDirChange % 2 == 0)
			cStep++;
	}
}

// code to build an inverse map if you need one

static void BuildInverseMap(
    const int mpPixelRegion[10][10],
    int mpRegionRowCol[REGION_Max][2])
{
    for (int row = 0; row < 10; row++)
    {
        for (int col = 0; col < 10; col++)
        {
            int region = mpPixelRegion[row][col];
            if (region >= 0 && region < REGION_Max)
            {
                mpRegionRowCol[region][0] = row;
                mpRegionRowCol[region][1] = col;
            }
        }
    }
}

// better spiral made by hand. tries to estimate an orange peel projection
// with the north pole in the center and preserving a few longitudinal lines.

static const int s_mpRowColIRegionPeel[10][10] =
{
    {  97,	90,	68,	67,	66,	65,	64,	63,	89,	96, },
    {  91,	69,	32,	31,	30,	29,	27,	62,	61,	88, },
    {  70,	34,	33,	8,	7,	28,	26,	25,	60,	59, },
    {  71,	36,	35,	9,	6,	5,	4,	23,	24,	58, },
    {  38,	37,	11,	10,	0,	2,	3,	21,	22,	57, },
    {  40,	39,	12,	13,	14,	1,	19,	20,	55,	56, },
    {  72,	41,	42,	44,	15,	16,	17,	18,	54,	87, },
    {  73,	43,	45,	46,	47,	48,	51,	52,	53,	86, },
    {  92,	74,	75,	77,	79,	49,	50,	84,	85,	95, },
    {  98,	93,	76,	78,	80,	81,	82,	83,	94,	99, },
};
static_assert(REGION_Max == 100);

// inverse of my peel map

static const int s_mpIRegionRowColPeel[REGION_Max][2] =
{
    {  4,  4 },  // region 0
    {  5,  5 },  // region 1
    {  4,  5 },  // region 2
    {  4,  6 },  // region 3
    {  3,  6 },  // region 4
    {  3,  5 },  // region 5
    {  3,  4 },  // region 6
    {  2,  4 },  // region 7
    {  2,  3 },  // region 8
    {  3,  3 },  // region 9
    {  4,  3 },  // region 10
    {  4,  2 },  // region 11
    {  5,  2 },  // region 12
    {  5,  3 },  // region 13
    {  5,  4 },  // region 14
    {  6,  4 },  // region 15
    {  6,  5 },  // region 16
    {  6,  6 },  // region 17
    {  6,  7 },  // region 18
    {  5,  6 },  // region 19
    {  5,  7 },  // region 20
    {  4,  7 },  // region 21
    {  4,  8 },  // region 22
    {  3,  7 },  // region 23
    {  3,  8 },  // region 24
    {  2,  7 },  // region 25
    {  2,  6 },  // region 26
    {  1,  6 },  // region 27
    {  2,  5 },  // region 28
    {  1,  5 },  // region 29
    {  1,  4 },  // region 30
    {  1,  3 },  // region 31
    {  1,  2 },  // region 32
    {  2,  2 },  // region 33
    {  2,  1 },  // region 34
    {  3,  2 },  // region 35
    {  3,  1 },  // region 36
    {  4,  1 },  // region 37
    {  4,  0 },  // region 38
    {  5,  1 },  // region 39
    {  5,  0 },  // region 40
    {  6,  1 },  // region 41
    {  6,  2 },  // region 42
    {  7,  1 },  // region 43
    {  6,  3 },  // region 44
    {  7,  2 },  // region 45
    {  7,  3 },  // region 46
    {  7,  4 },  // region 47
    {  7,  5 },  // region 48
    {  8,  5 },  // region 49
    {  8,  6 },  // region 50
    {  7,  6 },  // region 51
    {  7,  7 },  // region 52
    {  7,  8 },  // region 53
    {  6,  8 },  // region 54
    {  5,  8 },  // region 55
    {  5,  9 },  // region 56
    {  4,  9 },  // region 57
    {  3,  9 },  // region 58
    {  2,  9 },  // region 59
    {  2,  8 },  // region 60
    {  1,  8 },  // region 61
    {  1,  7 },  // region 62
    {  0,  7 },  // region 63
    {  0,  6 },  // region 64
    {  0,  5 },  // region 65
    {  0,  4 },  // region 66
    {  0,  3 },  // region 67
    {  0,  2 },  // region 68
    {  1,  1 },  // region 69
    {  2,  0 },  // region 70
    {  3,  0 },  // region 71
    {  6,  0 },  // region 72
    {  7,  0 },  // region 73
    {  8,  1 },  // region 74
    {  8,  2 },  // region 75
    {  9,  2 },  // region 76
    {  8,  3 },  // region 77
    {  9,  3 },  // region 78
    {  8,  4 },  // region 79
    {  9,  4 },  // region 80
    {  9,  5 },  // region 81
    {  9,  6 },  // region 82
    {  9,  7 },  // region 83
    {  8,  7 },  // region 84
    {  8,  8 },  // region 85
    {  7,  9 },  // region 86
    {  6,  9 },  // region 87
    {  1,  9 },  // region 88
    {  0,  8 },  // region 89
    {  0,  1 },  // region 90
    {  1,  0 },  // region 91
    {  8,  0 },  // region 92
    {  9,  1 },  // region 93
    {  9,  8 },  // region 94
    {  8,  9 },  // region 95
    {  0,  9 },  // region 96
    {  0,  0 },  // region 97
    {  9,  0 },  // region 98
    {  9,  9 },  // region 99
};
static_assert(DIM(s_mpIRegionRowColPeel) == REGION_Max);

} // namespace Sphere

} // namespace libcalib
