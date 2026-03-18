#pragma once

// IAhrs implementation wrapping the xioTechnologies Fusion
// library (Sebastian Madgwick).  Unlike CMahony/CAhrsNxp, CFusion does NOT
// oversample: FusionAhrs is designed to be called at the full sensor rate, so
// AddSample feeds the filter on every call with no SSampler accumulation.
//
#include "libcalib/common.h"
#include "libcalib/ahrs.h"

#define LIBCALIB_HAS_FUSION __has_include(<Fusion.h>)

#if LIBCALIB_HAS_FUSION
#include <Fusion.h>
#endif

namespace libcalib
{

class CSphereFitter;

class CFusion : public IAhrs  // tag: fuse
{
public:
				CFusion();

	void		Reset() override;
	void		AddSample(
					const SPoint & pntAccel,
					const SPoint & pntGyro,
					const SPoint & pntMag,
					const CSphereFitter & sphitter) override;
	void		Read(SQuat * pQuat) const override;

private:
#if LIBCALIB_HAS_FUSION
	FusionAhrs	m_fusionahrs;	// Fusion filter state
#endif
};

} // namespace libcalib