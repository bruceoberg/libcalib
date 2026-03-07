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
	Calibrator()
		{ reset(); }

	void reset();
	void add_raw_data(const int16_t(&data)[9]);

	MagCalibrator m_magcal;
	SQuat m_current_orientation;

private:
	int m_force_orientation_countdown;  // countdown to reseting m_ahrs in add_raw_data()

	CNxp m_nxp;	
	CFusion m_fusion;
	CMahony m_mahony;

	CMahony m_ahrs;			// Altitude Heading Reference System (could be Mahoney/Nxp/Fusion)

	static const int s_force_orientation_countdown_max = 240;
};

} // namespace libcalib
