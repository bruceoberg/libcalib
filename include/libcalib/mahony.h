#pragma once

#include "libcalib/common.h"
#include "libcalib/ahrs.h"
#include "libcalib/sampler.h"

namespace libcalib
{

class CMahony : public IAhrs
{
public:
			CMahony()
				{ Reset(); }

	void	Reset() override;
    void    AddSample(
				const SPoint & pntAccel,
				const SPoint & pntGyro,
				const SPoint & pntMag,
				const Sphere::CFitter & fitter) override;
    void    Read(SQuat * pQuat) const override
			{
				pQuat->q0 = m_q0;
				pQuat->q1 = m_q1;
				pQuat->q2 = m_q2;
				pQuat->q3 = m_q3;
			}

protected:
	void 	UpdateSamples(
				const SSampler::SAccel & accel,
				const SSampler::SGyro & gyro,
				const SSampler::SMag & mag,
				const Sphere::CFitter & fitter);
	void	UpdateSample(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
	void	UpdateSampleNoMag(float gx, float gy, float gz, float ax, float ay, float az);

	SSampler
			m_sampler;

	float	m_twoKp;		// 2 * proportional gain (Kp)
	float	m_twoKi;		// 2 * integral gain (Ki)

	float	m_q0;			// quaternion of sensor frame relative to auxiliary frame
	float	m_q1;
	float	m_q2;
	float	m_q3;

	float	m_integralFBx;	// integral error terms scaled by Ki
	float	m_integralFBy;
	float	m_integralFBz;

	bool	m_reset_next_update;
};

} // namespace libcalib
