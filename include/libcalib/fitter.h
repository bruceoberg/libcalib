#pragma once

#include "libcalib/common.h"
#include "libcalib/quality.h"

namespace libcalib
{

class Calibrator;

namespace Sphere
{

enum REGION
{
	REGION_Max = 100,
	REGION_Nil = -1,
};

// magnetic calibration & buffer structure

struct CFitter
{
	static const int s_cSampMax = 650; // Freescale's lib needs at least 392

	enum SOLVER
	{
		SOLVER_4Inv,	// 4 element matrix inversion calibration
		SOLVER_7Eig,	// 7 element eigenpair calibration
		SOLVER_10Eig,	// 10 element eigenpair calibration

		SOLVER_Max,
		SOLVER_Min = 0,
		SOLVER_Nil = -1
	};

	struct SSample
	{
		typedef uint32_t ID;

				SSample() = default;

				SSample(
					const SPoint & pntRaw,
					const Mag::SCal & cal)
				: m_pntRaw(pntRaw),
				  m_pntCal(),
				  m_field(),
				  m_region(REGION_Nil),
				  m_id()
					{ Calibrate(cal); }

		void	Calibrate(const Mag::SCal & cal);

		SPoint	m_pntRaw;	// raw sample
		SPoint	m_pntCal;	// calibrated sample
		float	m_field;	// length of calibrated sample
		REGION	m_region;	// sphere partition region (0..REGION_Max-1)
		ID		m_id;		// monotonically increasing ID assigned at insertion; lower = older
	};

	// encapsulates the sample buffer and per-region bookkeeping

	class CSampleSet
	{
	public:
				CSampleSet();

		void	AddSample(CFitter * pFitter, const SSample & samp);
		void	Recalibrate(const Mag::SCal & cal);

		REGION	RegionMostPopulated() const;
		int		ISampOldestInRegion(REGION region) const;

		int		CSamp() const
					{ return m_cSamp; }
		const SSample &
				Samp(int i) const
					{ return m_aSamp[i]; }
		int		CSampFromRegion(REGION region) const
					{ return m_mpRegionCSamp[region]; }

	private:
		SSample
				m_aSamp[s_cSampMax];
		int16_t	m_cSamp;
		int		m_mpRegionCSamp[REGION_Max];
		SSample::ID
				m_idNext;
	};

			CFitter();

	void	Reset()
				{ *this = CFitter(); }
	void	AddSample(const SPoint & BpFast, SPoint * pBcFast);
	bool	FHasNewCalibration(float * pSMadDiff);
	bool	FHasSolution() const
				{ return m_solver != SOLVER_Nil; }

	void	EnsureQuality()
				{ m_quality.Ensure(*this); }

	bool	AreErrorsOk() const
				{ return m_quality.AreErrorsOk(); }
	bool	AreErrorsBad() const
		{ return m_quality.AreErrorsBad(); }

	float	ErrGaps() const
				{ return m_quality.m_errGaps; }
	float	ErrVariance() const
				{ return m_quality.m_errVariance; }
	float	ErrWobble() const
				{ return m_quality.m_errWobble; }
	float	ErrFit() const
				{ return m_quality.m_errFit; }

	Mag::SCal
			m_cal;				// current calibration
	float	m_errFit;			// current fit error %
	SOLVER	m_solver;			// currently used solver
	CSampleSet
			m_samps;			// sample buffer with region bookkeeping

private:
	friend class ::libcalib::Calibrator;

	int		ISampFieldOutlier(REGION regionIncoming);

	void	UpdateCalibration4INV();
	void	UpdateCalibration7EIG();
	void	UpdateCalibration10EIG();

	Mag::SCal
			m_calNext;				// trial calibration
	float	m_calNext_B;			// trial value of geomagnetic field magnitude in uT
	float	m_errFitNext;			// trial value of fit error %
	float	m_errFitNextAged;		// current fit error % (grows automatically with age)
	float	m_A[3][3];				// ellipsoid matrix A
	float	m_invA[3][3];			// inverse of ellipsoid matrix A
	float	m_matA[10][10];			// scratch 10x10 matrix used by calibration algorithms
	float	m_matB[10][10];			// scratch 10x10 matrix used by calibration algorithms
	float	m_vecA[10];				// scratch 10x1 vector used by calibration algorithms
	float	m_vecB[4];				// scratch 4x1 vector used by calibration algorithms

	int		m_discard_count;		// ISampFieldOutlier() counter for choosing field strength discards
	int		m_new_wait_count;		// number of times FHasNewCalibration() had been called without doing any work

	SQuality
			m_quality;

	static constexpr int
			s_new_wait_count_max = 20; // in FHasNewCalibration() only do work after this many calls
};

} // namespace Sphere
} // namespace libcalib
