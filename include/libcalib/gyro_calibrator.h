#pragma once

#include "libcalib/calibrator.h"
#include "libcalib/common.h"

namespace libcalib
{
namespace Gyro
{

class CCalibrator : public ICalibrator
{
public:
	CCalibrator()
		{ Reset(); }

	// ICalibrator

	void Start() override;
	void Cancel() override;
	void OnSample(const SSample & samp) override;
	void Continue() override;

	bool FIsDone() const override;
	float UDone() const override;
	const char * PChzInstructions() const override;

	// Typed result getter

	void GetCalibration(SPoint * pVecBias) const;

private:
	void Reset();

	bool m_fStarted;
	bool m_fDone;
	int m_cSamp;
	SPoint m_pntSum;
	SPoint m_vecBias;

	static const int s_cSampRequired = 200;
};

} // namespace Gyro
} // namespace libcalib
