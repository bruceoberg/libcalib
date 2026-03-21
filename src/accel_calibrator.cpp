#include "libcalib/accel_calibrator.h"

namespace libcalib
{
namespace Accel
{

void CCalibrator::Reset()
{
	m_fStarted = false;
	m_fDone = false;
	m_face = FACE_Front;
	m_cSampFace = 0;
	m_pntFaceSum = SPoint();
	for (int i = 0; i < FACE_Max; ++i)
		m_mpFacePntAvg[i] = SPoint();
	m_mat = SMatrix3();
	m_vecBias = SPoint();
}

void CCalibrator::Start()
{
	Reset();
	m_fStarted = true;
}

void CCalibrator::Cancel()
{
	Reset();
}

void CCalibrator::OnSample(const SSample & samp)
{
	if (!m_fStarted || m_fDone)
		return;

	m_pntFaceSum.x += samp.m_pntAccel.x;
	m_pntFaceSum.y += samp.m_pntAccel.y;
	m_pntFaceSum.z += samp.m_pntAccel.z;
	++m_cSampFace;

	if (m_cSampFace >= s_cSampPerFace)
	{
		float sInv = 1.0f / float(s_cSampPerFace);
		m_mpFacePntAvg[m_face] = SPoint(
			m_pntFaceSum.x * sInv,
			m_pntFaceSum.y * sInv,
			m_pntFaceSum.z * sInv);

		m_pntFaceSum = SPoint();
		m_cSampFace = 0;
		m_face = FACE(m_face + 1);

		if (m_face >= FACE_Max)
		{
			Solve();
			m_fDone = true;
		}
	}
}

void CCalibrator::Continue()
{
	// no-op — host advances faces by calling OnSample with data from the next orientation
}

bool CCalibrator::FIsDone() const
{
	return m_fDone;
}

float CCalibrator::UDone() const
{
	float u = m_face / float(FACE_Max);
	if (u > 1.0f)
		u = 1.0f;
	return u;
}

const char * CCalibrator::PChzInstructions() const
{
	if (!m_fStarted || m_fDone)
		return nullptr;

	static const char * s_mpFacePchz[FACE_Max] =
	{
		"Hold with Front face down",	// FACE_Front
		"Hold with Back face down",		// FACE_Back
		"Hold with Left face down",		// FACE_Left
		"Hold with Right face down",	// FACE_Right
		"Hold with Top face down",		// FACE_Top
		"Hold with Bottom face down",	// FACE_Bottom
	};

	return s_mpFacePchz[m_face];
}

void CCalibrator::GetCalibration(SMatrix3 * pMat, SPoint * pVecBias) const
{
	*pMat = m_mat;
	*pVecBias = m_vecBias;
}

// ST DT0053 closed-form 6-position accelerometer calibration.

void CCalibrator::Solve()
{
	const SPoint & pntXAxisPos = m_mpFacePntAvg[FACE_Front];
	const SPoint & pntXAxisNeg = m_mpFacePntAvg[FACE_Back];
	const SPoint & pntYAxisPos = m_mpFacePntAvg[FACE_Left];
	const SPoint & pntYAxisNeg = m_mpFacePntAvg[FACE_Right];
	const SPoint & pntZAxisPos = m_mpFacePntAvg[FACE_Top];
	const SPoint & pntZAxisNeg = m_mpFacePntAvg[FACE_Bottom];

	// Offsets (gains cancel in opposite-face sums)

	float dXBias = (pntXAxisPos.x + pntXAxisNeg.x) * 0.5f;
	float dYBias = (pntYAxisPos.y + pntYAxisNeg.y) * 0.5f;
	float dZBias = (pntZAxisPos.z + pntZAxisNeg.z) * 0.5f;

	m_vecBias = SPoint(dXBias, dYBias, dZBias);

	// Gains and cross-gains (subtract bias from positive-face average)

	m_mat = SMatrix3(
		SPoint(pntXAxisPos.x - dXBias, pntYAxisPos.x - dXBias, pntZAxisPos.x - dXBias),	// row 0: X gain, YtoX, ZtoX
		SPoint(pntXAxisPos.y - dYBias, pntYAxisPos.y - dYBias, pntZAxisPos.y - dYBias),	// row 1: XtoY, Y gain, ZtoY
		SPoint(pntXAxisPos.z - dZBias, pntYAxisPos.z - dZBias, pntZAxisPos.z - dZBias));	// row 2: XtoZ, YtoZ, Z gain
}

} // namespace Accel
} // namespace libcalib
