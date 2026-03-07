#include "libcalib.h"

#include <math.h>

namespace libcalib
{

void Calibrator::reset()
{
	m_magcal.reset();

	m_current_orientation = SQuat();

	m_force_orientation_countdown = s_force_orientation_countdown_max;

	m_ahrs.Reset();
}

void Calibrator::add_raw_data(const int16_t(&data)[9])
{
	const SPoint pntAccel(
					SAccelFromS16(data[0]),
					SAccelFromS16(data[1]),
					SAccelFromS16(data[2]));
	const SPoint pntGyro(
					SGyroFromS16(data[3]),
					SGyroFromS16(data[4]),
					SGyroFromS16(data[5]));
	const SPoint pntMagRaw(
					SMagFromS16(data[6]),
					SMagFromS16(data[7]),
					SMagFromS16(data[8]));
	SPoint pntMagCal;

	m_magcal.AddMagPoint(pntMagRaw, &pntMagCal);

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

	m_ahrs.AddSample(pntAccel, pntGyro, pntMagCal, m_magcal);
	m_ahrs.Read(&m_current_orientation);
}

} // namespace libcalib
