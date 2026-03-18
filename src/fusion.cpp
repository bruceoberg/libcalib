#include "libcalib/fusion.h"
#include "libcalib/magcal.h"

namespace libcalib
{

#if LIBCALIB_HAS_FUSION

// ISM330DHCX gyro full-scale range configured in motion.cpp.
constexpr float k_gDegPerSecMax = 4000.0f;

// Fusion convention: accelerometer in g, gyroscope in deg/s, magnetometer in
// arbitrary consistent units (uT works fine).  Sample period in seconds.
constexpr float k_gSamplePeriod = (1.0f / SENSORFS);

// FusionAhrsSettings defaults are reasonable; we only need to override the
// gyroscope range to match the ISM330DHCX ±4000 dps configuration.
static const FusionAhrsSettings s_settings =
{
    .convention             = FusionConventionNwu,
    .gain                   = 0.5f,             // proportional feedback gain; tune as needed
    .gyroscopeRange         = k_gDegPerSecMax,
    .accelerationRejection  = 10.0f,            // degrees
    .magneticRejection      = 10.0f,            // degrees
    .recoveryTriggerPeriod  = 5 * (int)SENSORFS,// 5 s at sensor rate
};

CFusion::CFusion()
{
    FusionAhrsInitialise(&m_fusionahrs);
    FusionAhrsSetSettings(&m_fusionahrs, &s_settings);
}

void CFusion::Reset()
{
    FusionAhrsReset(&m_fusionahrs);
}

void CFusion::AddSample(
    const SPoint & pntAccel,
    const SPoint & pntGyro,
    const SPoint & pntMag,
    const CSphereFitter & sphitter)
{
    // Fusion takes deg/s directly — no conversion needed for gyro.
    // Accelerometer in g, magnetometer in uT (or any consistent unit).

    const FusionVector gyro  = { pntGyro.x,  pntGyro.y,  pntGyro.z  };
    const FusionVector accel = { pntAccel.x, pntAccel.y, pntAccel.z };

    if (sphitter.FHasSolution())
    {
        const FusionVector mag = { pntMag.x, pntMag.y, pntMag.z };
        FusionAhrsUpdate(&m_fusionahrs, gyro, accel, mag, k_gSamplePeriod);
    }
    else
    {
        FusionAhrsUpdateNoMagnetometer(&m_fusionahrs, gyro, accel, k_gSamplePeriod);
    }
}

void CFusion::Read(SQuat * pQuat) const
{
    // FusionQuaternion is { w, x, y, z }; SQuat is { q0, q1, q2, q3 } = { w, x, y, z }.

    const FusionQuaternion q = FusionAhrsGetQuaternion(&m_fusionahrs);
    pQuat->q0 = q.element.w;
    pQuat->q1 = q.element.x;
    pQuat->q2 = q.element.y;
    pQuat->q3 = q.element.z;
}

#else // LIBCALIB_HAS_FUSION

// Stubs: libcalib was built without the Fusion target.  Any call here is a
// configuration error — assert loudly rather than silently doing nothing.

CFusion::CFusion()                                              { __builtin_trap(); }
void CFusion::Reset()                                           { __builtin_trap(); }
void CFusion::AddSample(const SPoint &, const SPoint &,
                        const SPoint &, const CSphereFitter &) { __builtin_trap(); }
void CFusion::Read(SQuat *) const                              { __builtin_trap(); }

#endif // LIBCALIB_HAS_FUSION

} // namespace libcalib
