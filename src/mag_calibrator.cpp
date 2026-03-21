#include "libcalib/mag_calibrator.h"

#include <math.h>

namespace libcalib
{
namespace Mag
{

void CCalibrator::Reset()
{
	m_fStarted = false;

	m_fitter.Reset();

	m_current_orientation = SQuat();

	m_force_orientation_countdown = s_force_orientation_countdown_max;

	m_ahrs.Reset();
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
	AddSample(samp);
}

void CCalibrator::Continue()
{
	// no-op — mag calibration has no discrete user-driven steps
}

bool CCalibrator::FIsDone() const
{
	return m_fStarted && m_fitter.AreErrorsOk();
}

float CCalibrator::UDone() const
{
	float u = 1.0f - m_fitter.ErrGaps() / 100.0f;
	if (u < 0.0f)
		u = 0.0f;
	if (u > 1.0f)
		u = 1.0f;
	return u;
}

const char * CCalibrator::PChzInstructions() const
{
	if (m_fStarted && !FIsDone())
		return "Rotate device in all directions";
	return nullptr;
}

void CCalibrator::GetCalibration(Mag::SCal * pCal) const
{
	*pCal = m_fitter.m_cal;
}

void CCalibrator::AddSample(const SSample & samp)
{
	SPoint pntMagCal;

	m_fitter.AddSample(samp.m_pntMag, &pntMagCal);

	float sMagChange = 0.0f;

	if (m_fitter.FHasNewCalibration(&sMagChange)) {
		//printf("magdiff = %.2f\n", magdiff);
		if (sMagChange > 0.8f) {
			m_ahrs.Reset();
			m_force_orientation_countdown = s_force_orientation_countdown_max;
		}
	}

	if (m_force_orientation_countdown > 0) {
		if (--m_force_orientation_countdown == 0) {
			//printf("delayed forcible orientation reset\n");
			m_ahrs.Reset();
		}
	}

	m_ahrs.AddSample(samp.m_pntAccel, samp.m_pntGyro, pntMagCal, m_fitter);
	m_ahrs.Read(&m_current_orientation);
}

} // namespace Mag
} // namespace libcalib
