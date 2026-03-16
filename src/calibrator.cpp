#include "libcalib.h"

#include <math.h>

namespace libcalib
{

void Calibrator::Reset()
{
	m_magcal.Reset();

	m_current_orientation = SQuat();

	m_force_orientation_countdown = s_force_orientation_countdown_max;

	m_ahrs.Reset();
}

void Calibrator::AddSample(const SSample & samp)
{
	SPoint pntMagCal;

	m_magcal.AddSample(samp.m_pntMag, &pntMagCal);

	float sMagChange = 0.0f;

	if (m_magcal.FHasNewCalibration(&sMagChange)) {
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

	m_ahrs.AddSample(samp.m_pntAccel, samp.m_pntGyro, pntMagCal, m_magcal);
	m_ahrs.Read(&m_current_orientation);
}

} // namespace libcalib
