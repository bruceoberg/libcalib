#pragma once

#include "libcalib/common.h"
#include "libcalib/fitter.h"
#include "libcalib/fusion.h"
#include "libcalib/mahony.h"
#include "libcalib/nxp.h"

namespace libcalib
{
namespace Mag
{

class CCalibrator
{
public:
	static CCalibrator & Ensure()
	{
		static CCalibrator s_calib;
		return s_calib;
	}

	CCalibrator()
		{ Reset(); }

	void Reset();
	void AddSample(const SSample & samp);

	Sphere::CFitter m_fitter;
	SQuat m_current_orientation;

private:
	int m_force_orientation_countdown;  // countdown to reseting m_ahrs in add_raw_data()

	CFusion m_ahrs;			// Altitude Heading Reference System (could be Mahoney/Nxp/Fusion)

	static const int s_force_orientation_countdown_max = 240;
};

} // namespace Mag
} // namespace libcalib
