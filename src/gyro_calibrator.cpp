#include "libcalib/gyro_calibrator.h"

namespace libcalib
{
namespace Gyro
{

void CCalibrator::Reset()
{
	m_fStarted = false;
	m_fDone = false;
	m_cSamp = 0;
	m_pntSum = SPoint();
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

	m_pntSum.x += samp.m_pntGyro.x;
	m_pntSum.y += samp.m_pntGyro.y;
	m_pntSum.z += samp.m_pntGyro.z;
	++m_cSamp;

	if (m_cSamp >= s_cSampRequired)
	{
		float sInv = 1.0f / float(s_cSampRequired);
		m_vecBias = SPoint(m_pntSum.x * sInv, m_pntSum.y * sInv, m_pntSum.z * sInv);
		m_fDone = true;
	}
}

void CCalibrator::Continue()
{
	// no-op — gyro calibration is a single uninterrupted collection step
}

bool CCalibrator::FIsDone() const
{
	return m_fDone;
}

float CCalibrator::UDone() const
{
	float u = m_cSamp / float(s_cSampRequired);
	if (u > 1.0f)
		u = 1.0f;
	return u;
}

const char * CCalibrator::PChzInstructions() const
{
	if (m_fStarted && !m_fDone)
		return "Hold device completely still";
	return nullptr;
}

void CCalibrator::GetCalibration(SPoint * pVecBias) const
{
	*pVecBias = m_vecBias;
}

} // namespace Gyro
} // namespace libcalib
