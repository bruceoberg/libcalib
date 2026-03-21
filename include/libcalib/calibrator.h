#pragma once

#include "libcalib/common.h"

namespace libcalib
{

// Pure abstract interface for a calibration state machine.
// Hosts (SensorCal, kitelite) drive the calibrator by calling OnSample()
// with each new sample and querying progress / instructions for UI feedback.

class ICalibrator
{
public:
	virtual ~ICalibrator() = default;

	virtual void Start() = 0;					// begin or restart the calibration sequence
	virtual void Cancel() = 0;					// abort the current sequence
	virtual void OnSample(const SSample & samp) = 0;	// advance with one new sample
	virtual void Continue() = 0;

	virtual bool FIsDone() const = 0;			// true when calibration is complete
	virtual float UDone() const = 0;			// progress 0..1
	virtual const char * PChzInstructions() const = 0;	// human-readable current step, or nullptr when not active
};

} // namespace libcalib
