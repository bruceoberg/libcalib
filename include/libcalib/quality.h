#pragma once

#include "libcalib/common.h"

namespace libcalib
{

namespace Sphere {

namespace Quality
{
	constexpr float s_errMax = 100.0f;

	// these taken from MyFrame::OnTimer() in the original MotionCal app.
	//	(https://github.com/PaulStoffregen/MotionCal/blob/master/gui.cpp)

	// errors must all be below these values for calibration to be considered ok

	constexpr float s_errGapsOkMin = 15.0f;
	constexpr float s_errVarianceOkMin = 4.5f;
	constexpr float s_errWobbleOkMin = 4.0f;
	constexpr float s_errFitOkMin = 5.0f;

	// if errors all go above these values, the calibration is not ok anymore.

	constexpr float s_errGapsBadMax = 20.0f;
	constexpr float s_errVarianceBadMax = 5.0f;
	constexpr float s_errWobbleBadMax = 5.0f;
	constexpr float s_errFitBadMax = 6.0f;

};

} // namespace Sphere

} // namespace libcalib
