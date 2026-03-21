#include "libcalib/coordinator.h"

namespace libcalib
{

void CCoordinator::Reset()
{
	m_phase = PHASE_Nil;
}

void CCoordinator::Start()
{
	m_gyrocal.Cancel();
	m_accelcal.Cancel();
	m_magcal.Cancel();

	m_phase = PHASE_Gyro;
	m_gyrocal.Start();
}

void CCoordinator::Cancel()
{
	m_gyrocal.Cancel();
	m_accelcal.Cancel();
	m_magcal.Cancel();

	m_phase = PHASE_Nil;
}

void CCoordinator::OnSample(const SSample & samp)
{
	if (m_phase == PHASE_Nil || m_phase == PHASE_Done)
		return;

	switch (m_phase)
	{
	case PHASE_Gyro:
		m_gyrocal.OnSample(samp);
		if (m_gyrocal.FIsDone())
		{
			m_phase = PHASE_Accel;
			m_accelcal.Start();
		}
		break;

	case PHASE_Accel:
		m_accelcal.OnSample(samp);
		if (m_accelcal.FIsDone())
		{
			m_phase = PHASE_Mag;
			m_magcal.Start();
		}
		break;

	case PHASE_Mag:
		m_magcal.OnSample(samp);
		if (m_magcal.FIsDone())
		{
			m_phase = PHASE_Done;
		}
		break;

	default:
		break;
	}
}

void CCoordinator::Continue()
{
	if (m_phase == PHASE_Nil || m_phase == PHASE_Done)
		return;

	switch (m_phase)
	{
	case PHASE_Gyro:	m_gyrocal.Continue();	break;
	case PHASE_Accel:	m_accelcal.Continue();	break;
	case PHASE_Mag:		m_magcal.Continue();	break;
	default:			break;
	}
}

bool CCoordinator::FIsDone() const
{
	return m_phase == PHASE_Done;
}

float CCoordinator::UDone() const
{
	float iPhase;
	float uSub;

	switch (m_phase)
	{
	case PHASE_Gyro:	iPhase = 0.0f; uSub = m_gyrocal.UDone();	break;
	case PHASE_Accel:	iPhase = 1.0f; uSub = m_accelcal.UDone();	break;
	case PHASE_Mag:		iPhase = 2.0f; uSub = m_magcal.UDone();	break;
	case PHASE_Done:	iPhase = 3.0f; uSub = 0.0f;				break;
	default:			return 0.0f;
	}

	return (iPhase + uSub) / 3.0f;
}

const char * CCoordinator::PChzInstructions() const
{
	switch (m_phase)
	{
	case PHASE_Gyro:	return m_gyrocal.PChzInstructions();
	case PHASE_Accel:	return m_accelcal.PChzInstructions();
	case PHASE_Mag:		return m_magcal.PChzInstructions();
	default:			return nullptr;
	}
}

void CCoordinator::GetGyroCalib(SPoint * pVecBias) const
{
	m_gyrocal.GetCalibration(pVecBias);
}

void CCoordinator::GetAccelCalib(SMatrix3 * pMat, SPoint * pVecBias) const
{
	m_accelcal.GetCalibration(pMat, pVecBias);
}

void CCoordinator::GetMagCalib(Mag::SCal * pCal) const
{
	m_magcal.GetCalibration(pCal);
}

} // namespace libcalib
