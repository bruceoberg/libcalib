#include "libcalib/protocol.h"

namespace libcalib
{
namespace Protocol
{

// --- CManager ---

CManager::CManager(VER ver)
: m_ver(ver),
  m_verRemote(VER_Nil),
  m_pWriter(nullptr),
  m_pReader(nullptr),
  m_pReceiver(nullptr),
  m_textp(),
  m_calPending(),
  m_fHasCal1(false),
  m_binp()
{
}

void CManager::Init(IWriter * pWriter, IReader * pReader, IReceiver * pReceiver)
{
	m_pWriter = pWriter;
	m_pReader = pReader;
	m_pReceiver = pReceiver;

	// reset detection state so re-detection is possible after port change
	m_verRemote = VER_Nil;
	m_textp.Reset();
	m_binp.Reset();
	m_calPending = Mag::SCal();
	m_fHasCal1 = false;
}

// --- Update: drain reader, feed ver-appropriate parser ---

void CManager::Update()
{
	if (m_pReader == nullptr)
		return;

	uint8_t aBuf[256];
	for (;;)
	{
		size_t cB = m_pReader->CbRead(sizeof(aBuf), aBuf);
		if (cB == 0)
			break;

		if (m_verRemote == VER_Nil)
		{
			// detection phase: feed both parsers until one matches

			Text::CParser::LINEK linek = m_textp.LinekFeedBytes(cB, aBuf);
			if (linek != Text::CParser::LINEK_None)
			{
				// text line received — remote is VER_Imucal (device sending samples)
				m_verRemote = VER_Imucal;
				DispatchLine(linek);
				continue;
			}

			if (m_binp.FFeedBytes(cB, aBuf))
			{
				// binary packet received — remote is VER_MotionCal (host sending cal)
				m_verRemote = VER_MotionCal;
				if (m_pReceiver != nullptr)
					m_pReceiver->OnMagCal(m_binp.Cal());
				continue;
			}
		}
		else if (m_verRemote == VER_Imucal)
		{
			// remote sends text lines (Raw:, Uni:, Cal1:, Cal2:)
			Text::CParser::LINEK linek = m_textp.LinekFeedBytes(cB, aBuf);
			if (linek != Text::CParser::LINEK_None)
				DispatchLine(linek);
		}
		else if (m_verRemote == VER_MotionCal)
		{
			// remote sends binary calibration packets
			if (m_binp.FFeedBytes(cB, aBuf))
			{
				if (m_pReceiver != nullptr)
					m_pReceiver->OnMagCal(m_binp.Cal());
			}
		}
	}
}

// --- line parse dispatch ---

void CManager::DispatchLine(Text::CParser::LINEK linek)
{
	if (m_pReceiver == nullptr)
		return;

	switch (linek)
	{
	case Text::CParser::LINEK_Raw:
	case Text::CParser::LINEK_Uni:
		m_pReceiver->OnSample(m_textp.Samp());
		break;

	case Text::CParser::LINEK_Cal1:
		{
			const float * pG = m_textp.PGCal();
			// pG[0..2] = accel offsets (unused for now)
			// pG[3..5] = gyro offsets (unused for now)
			// pG[6..8] = mag hard iron xyz
			// pG[9]    = field strength
			m_calPending = Mag::SCal();
			m_calPending.m_vecV.x = pG[6];
			m_calPending.m_vecV.y = pG[7];
			m_calPending.m_vecV.z = pG[8];
			m_calPending.m_sB     = pG[9];
			m_fHasCal1 = true;
		}
		break;

	case Text::CParser::LINEK_Cal2:
		{
			if (!m_fHasCal1)
				break;

			const float * pG = m_textp.PGCal();
			// Cal2: row-major flat: XX, XY, XZ, YX, YY, YZ, ZX, ZY, ZZ
			m_calPending.m_matWInv.vecX.x = pG[0];
			m_calPending.m_matWInv.vecX.y = pG[1];
			m_calPending.m_matWInv.vecX.z = pG[2];
			m_calPending.m_matWInv.vecY.x = pG[3];
			m_calPending.m_matWInv.vecY.y = pG[4];
			m_calPending.m_matWInv.vecY.z = pG[5];
			m_calPending.m_matWInv.vecZ.x = pG[6];
			m_calPending.m_matWInv.vecZ.y = pG[7];
			m_calPending.m_matWInv.vecZ.z = pG[8];

			m_fHasCal1 = false;
			m_pReceiver->OnMagCal(m_calPending);
		}
		break;

	default:
		break;
	}
}

// --- SendSensorData (VER_Imucal only) ---

void CManager::SendSensorData(
	float xAccel, float yAccel, float zAccel,
	float xGyro,  float yGyro,  float zGyro,
	float xMag,   float yMag,   float zMag)
{
	if (m_ver != VER_Imucal || m_pWriter == nullptr)
		return;

	Text::EmitSensorData(m_pWriter, xAccel, yAccel, zAccel, xGyro, yGyro, zGyro, xMag, yMag, zMag);
}

// --- SendMagCal ---

void CManager::SendMagCal(const Mag::SCal & cal)
{
	if (m_pWriter == nullptr)
		return;

	if (m_ver == VER_MotionCal)
	{
		Binary::EmitCalibration(m_pWriter, cal);
		return;
	}

	if (m_ver != VER_Imucal)
		return;

	Text::EmitMagCal(m_pWriter, cal);
}

} // namespace Protocol
} // namespace libcalib
