#pragma once

#include "libcalib_common.h"
#include "libcalib_magcal.h"
#include "libcalib_fusion.h"
#include "libcalib_mahony.h"
#include "libcalib_nxp.h"

namespace libcalib
{

class Calibrator
{
public:
	static Calibrator & Ensure()
	{
		static Calibrator s_calib;
		return s_calib;
	}

	Calibrator()
		{ Reset(); }

	void Reset();
	void AddSample(const SSample & samp);

	MagCalibrator m_magcal;
	SQuat m_current_orientation;

private:
	int m_force_orientation_countdown;  // countdown to reseting m_ahrs in add_raw_data()

	CFusion m_ahrs;			// Altitude Heading Reference System (could be Mahoney/Nxp/Fusion)

	static const int s_force_orientation_countdown_max = 240;
};

} // namespace libcalib
