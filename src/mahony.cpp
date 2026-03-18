#include "libcalib_mahony.h"
#include "libcalib_magcal.h"

#include <math.h>

namespace libcalib
{
	void CMahony::AddSample(
		const SPoint &pntAccel,
		const SPoint &pntGyro,
		const SPoint &pntMag,
		const CSphereFitter &sphitter)
	{
		m_sampler.AddSample(pntAccel, pntGyro, pntMag);

		if (m_sampler.FIsFull())
		{
			UpdateSamples(
				m_sampler.m_accel,
				m_sampler.m_gyro,
				m_sampler.m_mag,
				sphitter);
		}
	}

	void CMahony::UpdateSamples(
		const SSampler::SAccel & accel,
		const SSampler::SGyro & gyro,
		const SSampler::SMag & mag,
		const CSphereFitter &sphitter)
	{
		constexpr int X = 0;
		constexpr int Y = 1;
		constexpr int Z = 2;

		for (int i = 0; i < m_sampler.s_cSample; i++)
		{
			if (sphitter.FHasSolution())
			{
				UpdateSample(
					RadFromDeg(gyro.YpFast[i][X]),
					RadFromDeg(gyro.YpFast[i][Y]),
					RadFromDeg(gyro.YpFast[i][Z]),
					accel.Gp[X],
					accel.Gp[Y],
					accel.Gp[Z],
					mag.Bc[X],
					mag.Bc[Y],
					mag.Bc[Z]);
			}
			else
			{
				// Use IMU algorithm if magnetometer measurement invalid
				// (avoids NaN in magnetometer normalisation)
				UpdateSampleNoMag(
					RadFromDeg(gyro.YpFast[i][X]),
					RadFromDeg(gyro.YpFast[i][Y]),
					RadFromDeg(gyro.YpFast[i][Z]),
					accel.Gp[X],
					accel.Gp[Y],
					accel.Gp[Z]);
			}
		}
	}


	// Madgwick's implementation of Mayhony's AHRS algorithm.
	// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
	//
	// From the x-io website "Open-source resources available on this website are
	// provided under the GNU General Public Licence unless an alternative licence
	// is provided in source."
	//
	// Date			Author			Notes
	// 29/09/2011	SOH Madgwick    Initial release
	// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
	//
	// Algorithm paper:
	// http://ieeexplore.ieee.org/xpl/login.jsp?tp=&arnumber=4608934&url=http%3A%2F%2Fieeexplore.ieee.org%2Fstamp%2Fstamp.jsp%3Ftp%3D%26arnumber%3D4608934

	//----------------------------------------------------------------------------------------------
	// Definitions

	constexpr float twoKpDef = (2.0f * 0.02f); // 2 * proportional gain
	constexpr float twoKiDef = (2.0f * 0.0f);  // 2 * integral gain

	constexpr float INV_SAMPLE_RATE = (1.0f / SENSORFS);

	// switched from tricky "fast" inverse square root to builtin because the trick is slow now.
	//	https://en.wikipedia.org/wiki/Fast_inverse_square_root#Obsolescence
	// float invSqrt(float x);

	void CMahony::Reset()
	{
		m_sampler.Reset();

		m_twoKp = twoKpDef; // 2 * proportional gain (Kp)
		m_twoKi = twoKiDef; // 2 * integral gain (Ki)

		m_q0 = 1.0f;
		m_q1 = 0.0f;
		m_q2 = 0.0f;
		m_q3 = 0.0f;

		// TODO: set a flag to immediately capture
		// magnetic orientation on next update

		m_integralFBx = 0.0f;
		m_integralFBy = 0.0f;
		m_integralFBz = 0.0f;
	}

	//----------------------------------------------------------------------------------------------
	// AHRS algorithm update

	void CMahony::UpdateSample(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
	{
		float norm;
		float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
		float hx, hy, bx, bz;
		float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
		float halfex, halfey, halfez;
		float qa, qb, qc;

		// Compute feedback only if accelerometer measurement valid
		// (avoids NaN in accelerometer normalisation)
		if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
		{

			// Normalise accelerometer measurement
			norm = sqrtf(ax * ax + ay * ay + az * az);
			ax /= norm;
			ay /= norm;
			az /= norm;

			// Normalise magnetometer measurement
			norm = sqrtf(mx * mx + my * my + mz * mz);
			mx /= norm;
			my /= norm;
			mz /= norm;

			// Auxiliary variables to avoid repeated arithmetic
			q0q0 = m_q0 * m_q0;
			q0q1 = m_q0 * m_q1;
			q0q2 = m_q0 * m_q2;
			q0q3 = m_q0 * m_q3;
			q1q1 = m_q1 * m_q1;
			q1q2 = m_q1 * m_q2;
			q1q3 = m_q1 * m_q3;
			q2q2 = m_q2 * m_q2;
			q2q3 = m_q2 * m_q3;
			q3q3 = m_q3 * m_q3;

			// Reference direction of Earth's magnetic field
			hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
			hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
			bx = sqrtf(hx * hx + hy * hy);
			bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

			// Estimated direction of gravity and magnetic field
			halfvx = q1q3 - q0q2;
			halfvy = q0q1 + q2q3;
			halfvz = q0q0 - 0.5f + q3q3;
			halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
			halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
			halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

			// Error is sum of cross product between estimated direction
			// and measured direction of field vectors
			halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
			halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
			halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

			// Compute and apply integral feedback if enabled
			if (m_twoKi > 0.0f)
			{
				// integral error scaled by Ki
				m_integralFBx += m_twoKi * halfex * INV_SAMPLE_RATE;
				m_integralFBy += m_twoKi * halfey * INV_SAMPLE_RATE;
				m_integralFBz += m_twoKi * halfez * INV_SAMPLE_RATE;
				gx += m_integralFBx; // apply integral feedback
				gy += m_integralFBy;
				gz += m_integralFBz;
			}
			else
			{
				m_integralFBx = 0.0f; // prevent integral windup
				m_integralFBy = 0.0f;
				m_integralFBz = 0.0f;
			}

			// printf("err =  %.3f, %.3f, %.3f\n", halfex, halfey, halfez);

			// Apply proportional feedback
			if (m_reset_next_update)
			{
				gx += 2.0f * halfex;
				gy += 2.0f * halfey;
				gz += 2.0f * halfez;
				m_reset_next_update = 0;
			}
			else
			{
				gx += m_twoKp * halfex;
				gy += m_twoKp * halfey;
				gz += m_twoKp * halfez;
			}
		}

		// Integrate rate of change of quaternion
		gx *= (0.5f * INV_SAMPLE_RATE); // pre-multiply common factors
		gy *= (0.5f * INV_SAMPLE_RATE);
		gz *= (0.5f * INV_SAMPLE_RATE);
		qa = m_q0;
		qb = m_q1;
		qc = m_q2;
		m_q0 += (-qb * gx - qc * gy - m_q3 * gz);
		m_q1 += (qa * gx + qc * gz - m_q3 * gy);
		m_q2 += (qa * gy - qb * gz + m_q3 * gx);
		m_q3 += (qa * gz + qb * gy - qc * gx);

		// Normalise quaternion
		norm = sqrtf(m_q0 * m_q0 + m_q1 * m_q1 + m_q2 * m_q2 + m_q3 * m_q3);
		m_q0 /= norm;
		m_q1 /= norm;
		m_q2 /= norm;
		m_q3 /= norm;
	}

	//---------------------------------------------------------------------------------------------
	// IMU algorithm update

	void CMahony::UpdateSampleNoMag(float gx, float gy, float gz, float ax, float ay, float az)
	{
		float norm;
		float halfvx, halfvy, halfvz;
		float halfex, halfey, halfez;
		float qa, qb, qc;

		// Compute feedback only if accelerometer measurement valid
		// (avoids NaN in accelerometer normalisation)
		if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
		{

			// Normalise accelerometer measurement
			norm = sqrtf(ax * ax + ay * ay + az * az);
			ax /= norm;
			ay /= norm;
			az /= norm;

			// Estimated direction of gravity and vector perpendicular to magnetic flux
			halfvx = m_q1 * m_q3 - m_q0 * m_q2;
			halfvy = m_q0 * m_q1 + m_q2 * m_q3;
			halfvz = m_q0 * m_q0 - 0.5f + m_q3 * m_q3;

			// Error is sum of cross product between estimated and measured direction of gravity
			halfex = (ay * halfvz - az * halfvy);
			halfey = (az * halfvx - ax * halfvz);
			halfez = (ax * halfvy - ay * halfvx);

			// Compute and apply integral feedback if enabled
			if (m_twoKi > 0.0f)
			{
				// integral error scaled by Ki
				m_integralFBx += m_twoKi * halfex * INV_SAMPLE_RATE;
				m_integralFBy += m_twoKi * halfey * INV_SAMPLE_RATE;
				m_integralFBz += m_twoKi * halfez * INV_SAMPLE_RATE;
				gx += m_integralFBx; // apply integral feedback
				gy += m_integralFBy;
				gz += m_integralFBz;
			}
			else
			{
				m_integralFBx = 0.0f; // prevent integral windup
				m_integralFBy = 0.0f;
				m_integralFBz = 0.0f;
			}

			// Apply proportional feedback
			gx += m_twoKp * halfex;
			gy += m_twoKp * halfey;
			gz += m_twoKp * halfez;
		}

		// Integrate rate of change of quaternion
		gx *= (0.5f * INV_SAMPLE_RATE); // pre-multiply common factors
		gy *= (0.5f * INV_SAMPLE_RATE);
		gz *= (0.5f * INV_SAMPLE_RATE);
		qa = m_q0;
		qb = m_q1;
		qc = m_q2;
		m_q0 += (-qb * gx - qc * gy - m_q3 * gz);
		m_q1 += (qa * gx + qc * gz - m_q3 * gy);
		m_q2 += (qa * gy - qb * gz + m_q3 * gx);
		m_q3 += (qa * gz + qb * gy - qc * gx);

		// Normalise quaternion
		norm = sqrtf(m_q0 * m_q0 + m_q1 * m_q1 + m_q2 * m_q2 + m_q3 * m_q3);
		m_q0 /= norm;
		m_q1 /= norm;
		m_q2 /= norm;
		m_q3 /= norm;
	}

} // namespace libcalib
