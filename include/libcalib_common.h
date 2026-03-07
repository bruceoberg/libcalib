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

// conversion ratios for s16 line samples.

constexpr float SAccelFromS16(int16_t s)
{
	return float(s) * 0.0001220703125F;  // = 1/8192
}

constexpr float SMagFromS16(int16_t s)
{
	return float(s) * 0.1F;
}

constexpr float SGyroFromS16(int16_t s)
{
	return float(s) * 0.0625F;
}

} // namespace libcalib
