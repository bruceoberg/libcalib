#pragma once

#include "libcalib/gyro_calibrator.h"
#include "libcalib/accel_calibrator.h"
#include "libcalib/mag_calibrator.h"

namespace libcalib
{

// Orchestrates the three calibration phases in sequence: gyro, accel, mag.
// Owns the three sub-calibrators by value and exposes an ICalibrator-style
// interface, but does not inherit ICalibrator — it spans all three phases.

class CCoordinator
{
public:
	enum PHASE
	{
		PHASE_Gyro,
		PHASE_Accel,
		PHASE_Mag,
		PHASE_Done,
		PHASE_Nil = -1,
	};

	CCoordinator()
		{ Reset(); }

	// ICalibrator-style interface

	void Start();
	void StartAtPhase(PHASE phase);		// reset from phase onward, start there
	void Cancel();
	void OnSample(const SSample & samp);
	void Continue();

	bool FIsDone() const;
	float UDone() const;
	const char * PChzInstructions() const;

	// Phase accessor

	PHASE Phase() const					{ return m_phase; }

	// Const sub-calibrator access — for UI metrics (quality, per-face progress, etc.)

	const Gyro::CCalibrator & GyroCalib() const		{ return m_gyrocal; }
	const Accel::CCalibrator & AccelCalib() const	{ return m_accelcal; }
	const Mag::CCalibrator & MagCalib() const		{ return m_magcal; }

	// Typed result getters — delegate to sub-calibrators

	void GetGyroCalib(SPoint * pVecBias) const;
	void GetAccelCalib(SMatrix3 * pMat, SPoint * pVecBias) const;
	void GetMagCalib(Mag::SCal * pCal) const;

private:
	void Reset();

	PHASE m_phase;

	Gyro::CCalibrator m_gyrocal;
	Accel::CCalibrator m_accelcal;
	Mag::CCalibrator m_magcal;
};

} // namespace libcalib
