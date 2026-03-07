#pragma once

// libcalib_ahrs.h — IAhrs interface for all AHRS implementations.
// Concrete subclasses: AhrsNxp, AhrsMahony, AhrsFusion.
// Calibrator holds a non-owning IAhrs* and calls through this interface;
// implementations own their internal state and oversampler (if any).

#include "libcalib_common.h"

namespace libcalib
{

class MagCalibrator;  // libcalib_magcal.h

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
    // magcal — current magnetometer calibration state; implementations query
    //          m_isValid and m_cal_B directly rather than receiving extracted scalars.
    virtual void    AddSample(
                        const SPoint & accel,
                        const SPoint & mag,
                        const SPoint & gyro,
                        const MagCalibrator & magcal) = 0;

    // Copy the most recently computed orientation into *pQ.
    // Safe to call after every AddSample; oversampling implementations return
    // the last computed quaternion between filter updates.
    virtual void    Read(SQuat * pQuat) const = 0;
};

} // namespace libcalib