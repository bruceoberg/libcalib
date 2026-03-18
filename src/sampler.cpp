#include "libcalib/sampler.h"

#include <math.h>

namespace libcalib
{

void SSampler::Reset()
{
	m_iSampleCur = 0;

	m_accel = { 0 };
	m_mag = { 0 };
	m_gyro = { 0 };
}

void SSampler::AddSample(
	const SPoint & pntAccel,
	const SPoint & pntGyro,
	const SPoint & pntMagCal)
{
	if (FIsFull()) {
		Reset();
	}

	m_accel.GpFast[0] = pntAccel.x;
	m_accel.GpFast[1] = pntAccel.y;
	m_accel.GpFast[2] = pntAccel.z;
	m_accel.Gp[0] += pntAccel.x;
	m_accel.Gp[1] += pntAccel.y;
	m_accel.Gp[2] += pntAccel.z;

	m_gyro.Yp[0] += pntGyro.x;
	m_gyro.Yp[1] += pntGyro.y;
	m_gyro.Yp[2] += pntGyro.z;
	m_gyro.YpFast[m_iSampleCur][0] = pntGyro.x;
	m_gyro.YpFast[m_iSampleCur][1] = pntGyro.y;
	m_gyro.YpFast[m_iSampleCur][2] = pntGyro.z;

	m_mag.BcFast[0] = pntMagCal.x;
	m_mag.BcFast[1] = pntMagCal.y;
	m_mag.BcFast[2] = pntMagCal.z;
	m_mag.Bc[0] += pntMagCal.x;
	m_mag.Bc[1] += pntMagCal.y;
	m_mag.Bc[2] += pntMagCal.z;

	++m_iSampleCur;

	if (FIsFull()) {
		static const float s_rAveraging = 1.0 / float(s_cSample);

		m_accel.Gp[0] *= s_rAveraging;
		m_accel.Gp[1] *= s_rAveraging;
		m_accel.Gp[2] *= s_rAveraging;
		m_gyro.Yp[0] *= s_rAveraging;
		m_gyro.Yp[1] *= s_rAveraging;
		m_gyro.Yp[2] *= s_rAveraging;
		m_mag.Bc[0] *= s_rAveraging;
		m_mag.Bc[1] *= s_rAveraging;
		m_mag.Bc[2] *= s_rAveraging;
	}
}

}