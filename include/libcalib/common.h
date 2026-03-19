#pragma once

#include <stdint.h>

namespace libcalib
{

#ifndef M_PI
// Source: http://www.geom.uiuc.edu/~huberty/math5337/groupe/digits.html
constexpr float M_PI = 3.141592653589793238462643383279502884197169399375105820974944592307816406;
#endif

constexpr float RadFromDeg(float deg)
{
	return (M_PI * deg) / 180.0f;
}

constexpr float DegFromRad(float rad)
{
	return (180.0f * rad) / M_PI;
}

constexpr float GFromMPerSecSq(float mPerSecSq)
{
	return mPerSecSq / 9.80665f;
}

constexpr float MPerSecSqFromG(float g)
{
	return g * 9.80665f;
}

template<uint32_t N, class T>
constexpr uint32_t DIM(T(&)[N]) { return N; }


constexpr int SENSORFS = 100;

struct SPoint
{
	SPoint(
		float i_x = 0.0f,
		float i_y = 0.0f,
		float i_z = 0.0f)
	: x(i_x), y(i_y), z(i_z)
		{ ; }

	float x;
	float y;
	float z;

	float & operator[](int i)
		{ return (&x)[i]; }
	const float & operator[](int i) const
		{ return (&x)[i]; }
};
static_assert(sizeof(SPoint) == 3 * sizeof(float));

struct SQuat
{
	SQuat(
		float i_q0 = 1.0f,
		float i_q1 = 0.0f,
		float i_q2 = 0.0f,
		float i_q3 = 0.0f)
	: q0(i_q0), q1(i_q1), q2(i_q2), q3(i_q3)
		{ ; }

	float q0; // w
	float q1; // x
	float q2; // y
	float q3; // z
};

struct SMatrix3
{
	SMatrix3(
		SPoint i_vecX = SPoint(1.0f, 0.0f, 0.0f),
		SPoint i_vecY = SPoint(0.0f, 1.0f, 0.0f),
		SPoint i_vecZ = SPoint(0.0f, 0.0f, 1.0f))
	: vecX(i_vecX), vecY(i_vecY), vecZ(i_vecZ)
		{ ; }

	SPoint vecX;
	SPoint vecY;
	SPoint vecZ;

	SPoint & operator[](int i)
		{ return (&vecX)[i]; }
	const SPoint & operator[](int i) const
		{ return (&vecX)[i]; }
};
static_assert(sizeof(SMatrix3) == 3 * sizeof(SPoint));

struct SSample	// tag = samp
{
	SPoint m_pntAccel;	// accelerometer (g)
	SPoint m_pntGyro;	// gyroscope (deg/s)
	SPoint m_pntMag;	// magnetometer (uT)
};

namespace Mag
{
	constexpr float s_sBDefault = 50.0f;

struct SCal // tag = cal
{
			SCal()
			: m_vecV(0.0f, 80.0f, 0.0f),	// Initial guess from MotionCal code
			  m_matWInv(),
			  m_sB(s_sBDefault)
				{ ; }

	SPoint	m_vecV;		// hard iron offset x, y, z, (uT)
	SMatrix3
			m_matWInv;	// inverse soft iron matrix
	float	m_sB;		// geomagnetic field magnitude (uT)
};

}

} // namespace libcalib
