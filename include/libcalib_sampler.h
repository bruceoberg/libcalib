#pragma once

#include "libcalib_common.h"

namespace libcalib
{

struct SSampler
{
	static const int s_cSample = 4;

	struct SAccel
	{
		float Gp[3];           // slow (typically 25Hz) averaged readings (g)
		float GpFast[3];       // fast (typically 200Hz) readings (g)
	};

	struct SMag
	{
		float Bc[3];           // slow (typically 25Hz) averaged calibrated readings (uT)
		float BcFast[3];       // fast (typically 200Hz) calibrated readings (uT)
	};

	struct SGyro
	{
		float Yp[3];                           // slow (typically 25Hz) averaged gyro sensor readings (deg/s)
		float YpFast[s_cSample][3];     // fast (typically 200Hz) readings (deg/s)
	};

	SSampler()
		{ Reset(); }

	void Reset();
	void AddSample(const SPoint & pntAccel, const SPoint & pntGyro, const SPoint & pntMagCal);
	bool FIsFull() const
		{ return m_iSampleCur >= s_cSample; }

	int m_iSampleCur;         // countdown of oversampling in add_raw_data()

	SAccel m_accel;
	SMag   m_mag;
	SGyro  m_gyro;
};

} // namespace libcalib
