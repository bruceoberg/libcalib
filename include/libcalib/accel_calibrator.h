#pragma once

#include "libcalib/calibrator.h"
#include "libcalib/common.h"

namespace libcalib
{
namespace Accel
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

	void GetCalibration(SMatrix3 * pMat, SPoint * pVecBias) const;

private:
	enum FACE
	{
		FACE_Front,		// +X down
		FACE_Back,		// -X down
		FACE_Left,		// +Y down
		FACE_Right,		// -Y down
		FACE_Top,		// +Z down
		FACE_Bottom,	// -Z down

		FACE_Max,
	};

	void Reset();
	void Solve();

	bool m_fStarted;
	bool m_fDone;
	FACE m_face;
	int m_cSampFace;
	SPoint m_pntFaceSum;
	SPoint m_mpFacePntAvg[FACE_Max];
	SMatrix3 m_mat;
	SPoint m_vecBias;

	static const int s_cSampPerFace = 50;
};

} // namespace Accel
} // namespace libcalib
