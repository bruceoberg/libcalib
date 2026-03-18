#pragma once

#include "libcalib/common.h"
#include "libcalib/magcal.h"

namespace libcalib
{

namespace Sphere
{

const SPoint & PntAnchorFromRegion(int region);

int RegionFromXyz(float x, float y, float z);

} // namespace Sphere

} // namespace libcalib
