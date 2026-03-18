#pragma once

// IAhrs interface for all AHRS implementations.
// Concrete subclasses: AhrsNxp, AhrsMahony, AhrsFusion.
// Calibrator holds a non-owning IAhrs* and calls through this interface;
// implementations own their internal state and oversampler (if any).

#include "libcalib/common.h"

namespace libcalib
{

class CSphereFitter;

class IAhrs  // tag: ahrs
{
public:
    virtual         ~IAhrs() = default;

    // Reset to a known initial state.  Call when re-initializing the sensor
    // pipeline or when magnetic calibration changes significantly.
    virtual void    Reset() = 0;

    // Feed one raw sensor sample.  Called once per sensor interrupt at SENSORFS.
    // Implementations that oversample (Nxp, Mahony) accumulate internally and
    // run their filter update every SSampler::s_cSample calls; Fusion runs every call.
    // sphitter — current magnetometer calibration state; implementations query
    //            FHasSolution() and m_cal_B directly rather than receiving extracted scalars.
    virtual void    AddSample(
                        const SPoint & pntAccel,
                        const SPoint & pntGyro,
                        const SPoint & pntMag,
                        const CSphereFitter & sphitter) = 0;

    // Copy the most recently computed orientation into *pQ.
    // Safe to call after every AddSample; oversampling implementations return
    // the last computed quaternion between filter updates.
    virtual void    Read(SQuat * pQuat) const = 0;
};

} // namespace libcalib