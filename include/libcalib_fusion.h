#pragma once

#include "libcalib_common.h"
#include "libcalib_ahrs.h"

namespace libcalib
{

class CFusion : public IAhrs
{
public:
			CFusion()
				{ Reset(); }

	void	Reset() override
				{ ; }
    void    AddSample(
				const SPoint & pntAccel,
				const SPoint & pntMag,
				const SPoint & pntGyro,
				const MagCalibrator & magcal) override
				{ ; }
    void    Read(SQuat * pQuat) const override
				{ ; }

protected:
};

} // namespace libcalib
